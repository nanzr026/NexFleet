"""Per-AMR DDS coordination session; every peer computes the same decisions.

Task issuer supplies membership and a navigation graph, never priorities,
paths, winners, or motion commands. All graph edges are checked against A*.
Step barriers use measured arrival, not assumed equal robot velocities.
"""
import hashlib
import json
import math
import time
from std_msgs.msg import String
from .coordination_search import ConfigurationSearch, point_segment_distance
from .lacam_pibt import ConflictDetector, PIBTResolver


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()[:20]


class CoordinationSession:
    def __init__(self, node, task, route):
        self.n = node
        spec = task['coordination']
        self.run = spec['run']
        self.ids = tuple(sorted(spec['members']))
        self.index = self.ids.index(node.robot_id)
        self.nodes = [tuple(map(float, p)) for p in spec['nodes']]
        self.edges = [tuple(e) for e in spec['edges']]
        if not 1 < len(self.ids) <= 3 or not 1 < len(self.nodes) <= 100:
            raise ValueError('Unsupported coordination graph size')
        for a,b in self.edges:
            if not node.astar.has_line_of_sight(self.nodes[a],self.nodes[b]):
                raise ValueError('Coordination edge intersects occupancy map')
        self.graph_hash = digest([self.ids,self.nodes,self.edges])
        self.initial = min(range(len(self.nodes)), key=lambda i: math.dist(self.nodes[i],(node.x,node.y)))
        self.goal = min(range(len(self.nodes)), key=lambda i: math.dist(self.nodes[i],(task['pickup_x'],task['pickup_y'])))
        if math.dist(self.nodes[self.initial],(node.x,node.y))>.3 or math.dist(self.nodes[self.goal],(task['pickup_x'],task['pickup_y']))>.01:
            raise ValueError('Session endpoints must match graph vertices')
        self.route = route
        self.priority = PIBTResolver.get_robot_priority(node.robot_id,node.battery_level,task.get('priority','NORMAL'))
        self.peers = {}
        self.sequence = 0
        self.previous = None
        self.signature = None
        self.q = self.target = None
        self.solver = None
        self.arrived = False
        self.done = False
        self.failed = False
        self.allow = False
        self.fallback = []
        self.seen = {}
        self.stall_since = None
        self.suspected = False
        self.last_progress = node.pose_sim_time
        self.progress_position = (node.x,node.y)
        self.created = node.pose_sim_time
        self.last_tick = self.created
        self.metrics = dict(conflicts=0,replans=0,pibt_resolutions=0,lacam_activations=0,
                            deadlocks=0,waiting_time=0.,steps=0,inheritances=0,backtracks=0)
        self.event('SESSION', graph=self.graph_hash, members=self.ids)

    def event(self, label, **data):
        event = dict(robot_id=self.n.robot_id,run=self.run,event=label,
                     simulation_time=self.n.pose_sim_time,sequence=self.sequence,**data)
        self.n.coordination_events.publish(String(data=json.dumps(event)))
        self.n.get_logger().info(label+' '+json.dumps(event,sort_keys=True))

    def receive(self, rid, data):
        if data.get('run')==self.run and data.get('graph')==self.graph_hash and rid in self.ids:
            self.peers[rid]=(data,time.monotonic())

    def state(self):
        return dict(robot_id=self.n.robot_id,run=self.run,graph=self.graph_hash,
                    initial=self.initial,goal=self.goal,priority=self.priority,route=self.route,
                    sequence=self.sequence,signature=self.signature,previous=self.previous,
                    target=self.target,arrived=self.arrived,done=self.done,failed=self.failed,
                    pose=[self.n.x,self.n.y],simulation_time=self.n.pose_sim_time,metrics=self.metrics)

    def all_states(self):
        now=time.monotonic()
        result={self.n.robot_id:self.state()}
        for rid in self.ids:
            if rid==self.n.robot_id:continue
            value=self.peers.get(rid)
            if not value or now-value[1]>2.0 or abs(self.n.pose_sim_time-value[0]['simulation_time'])>2.0:
                return None
            result[rid]=value[0]
        return result

    def initialize(self, states):
        self.last_progress=self.n.pose_sim_time
        self.q=tuple(states[r]['initial'] for r in self.ids)
        goals=[states[r]['goal'] for r in self.ids]
        priorities=[states[r]['priority'] for r in self.ids]
        self.solver=ConfigurationSearch(self.nodes,self.edges,goals,priorities)
        # Resample physical-length A* routes onto a common nominal time base.
        def sample(path):
            result=[path[0]]
            for a,b in zip(path,path[1:]):
                count=max(1,math.ceil(math.dist(a,b)/.175))
                result.extend([(a[0]+(b[0]-a[0])*k/count,a[1]+(b[1]-a[1])*k/count) for k in range(1,count+1)])
            return result
        for i,r in enumerate(self.ids):
            for other in self.ids[:i]:
                conflict,k,a,b=ConflictDetector(1.3).detect_path_conflict(
                    sample(states[r]['route']),sample(states[other]['route']))
                if conflict:
                    self.metrics['conflicts']+=1
                    self.event('PIBT CONFLICT',robots=[r,other],type='predicted_unsafe_separation',
                               location=[a,b],timestep=k,predicted_time=k*.5)
        self.event('PIBT PRIORITY',priorities=dict(zip(self.ids,priorities)))
        self.seen[self.q]=1
        self.plan_step()

    def plan_step(self):
        if self.q==self.solver.goals:
            self.done=True
            self.allow=False
            self.event('COORDINATION COMPLETED',metrics=self.metrics)
            return
        using_fallback=bool(self.fallback)
        next_q=self.fallback.pop(0) if using_fallback else self.solver.pibt(self.q)
        self.method='LaCAM' if self.fallback or getattr(self,'fallback_active',False) else 'PIBT'
        if next_q is None or (not using_fallback and self.seen.get(next_q,0)>=2):
            next_q=self.q
        self.previous=self.signature
        self.sequence+=1
        self.target=next_q
        self.signature=digest([self.run,self.sequence,self.q,self.target])
        self.arrived=False
        self.allow=False
        self.last_progress=self.n.pose_sim_time
        self.metrics['steps']+=1
        self.metrics['inheritances']=self.solver.inheritances
        self.metrics['backtracks']=self.solver.backtracks
        rerouted=(self.target[self.index] != self.q[self.index] and
                  self.solver.distances[self.index].get(self.target[self.index],math.inf) >=
                  self.solver.distances[self.index].get(self.q[self.index],math.inf))
        if rerouted:
            self.metrics['replans']+=1
        self.event('PIBT DECISION' if self.method=='PIBT' else 'LaCAM DECISION',
                   configuration=self.q,next_configuration=self.target,
                   action='WAIT' if self.q[self.index]==self.target[self.index] else ('REROUTE' if rerouted else 'PROCEED'),
                   method=self.method,signature=self.signature)

    def update(self):
        now=self.n.pose_sim_time
        delta=max(0.,min(1.,now-self.last_tick))
        self.last_tick=now
        if not self.allow:self.metrics['waiting_time']+=delta
        self.allow=False
        if self.done or self.failed:return
        states=self.all_states()
        if not states or any(s.get('failed') for s in states.values()):return
        if self.solver is None:
            self.initialize(states)
            return
        if math.dist((self.n.x,self.n.y),self.progress_position)>.12:
            self.progress_position=(self.n.x,self.n.y)
            self.last_progress=now
        target=self.nodes[self.target[self.index]]
        if math.dist((self.n.x,self.n.y),target)<.14:
            self.arrived=True
        agreement=all(s['sequence']==self.sequence and s['signature']==self.signature for s in states.values())
        completed= self.arrived and all(
            (s['sequence']==self.sequence and s['signature']==self.signature and s['arrived'])
            or (s['sequence']==self.sequence+1 and s['previous']==self.signature)
            for s in states.values())
        if completed:
            if self.target==self.q:
                if self.stall_since is None:self.stall_since=now
                duration=now-self.stall_since
                if duration>=4 and not self.suspected:
                    self.suspected=True
                    self.event('DEADLOCK SUSPECTED',reason='Repeated configuration and no safe PIBT successor')
                if duration<8:return
                self.metrics['deadlocks']+=1
                self.event('DEADLOCK CONFIRMED',duration=duration)
                path,expansions=self.solver.lacam(self.q)
                self.metrics['lacam_activations']+=1
                self.event('RESOLUTION METHOD',method='LaCAM + PIBT',expansions=expansions,success=bool(path))
                if not path:
                    self.failed=True
                    self.event('COORDINATION FAILED',reason='Bounded LaCAM exhausted; robots remain stopped')
                    return
                self.fallback=path[1:]
                self.fallback_active=True
                self.metrics['replans']+=1
                self.event('NEW ROUTE',configurations=path)
                self.stall_since=None
            else:
                self.stall_since=None
                self.q=self.target
                self.seen[self.q]=self.seen.get(self.q,0)+1
                if self.method=='PIBT':
                    self.metrics['pibt_resolutions']+=1
                    self.event('PIBT RESOLUTION',configuration=self.q)
            self.plan_step()
            return
        if not agreement or self.arrived:return
        # Check the entire remaining segment against current peer positions.
        # This tolerates arbitrary unequal turn/translation times between barriers.
        for rid in self.ids:
            if rid==self.n.robot_id:continue
            if point_segment_distance(states[rid]['pose'],(self.n.x,self.n.y),target)<1.4:
                return
        if now-self.last_progress>45:
            self.failed=True
            self.event('DEADLOCK CONFIRMED',reason='Physical motion stalled for 45 simulation seconds; stopped')
            self.metrics['deadlocks']+=1
            return
        self.allow=True
