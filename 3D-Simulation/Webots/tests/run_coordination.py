"""Create fresh physical scenarios and run decentralized agents and evidence probe."""
import argparse,json,math,os,re,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'Webots/logs'
def scenario(name):
    if name=='B':
        nodes=[(5,-6),(5,-4),(5,-2),(3,-6),(5,-8)]
        edges=[(0,1),(1,2),(0,3),(0,4)]
        starts=[0,2];goals=[2,4];yaws=[math.pi/2,-math.pi/2]
    else:
        nodes=[(x,y) for y in (-12,-10,-8) for x in (0,2,4)]
        edges=[(i,j) for i,a in enumerate(nodes) for j,b in enumerate(nodes[:i]) if math.dist(a,b)==2]
        starts=[3,1];goals=[5,7];yaws=[0,math.pi/2]
        if name=='C':starts+=[5];goals+=[3];yaws+=[math.pi]
    return nodes,edges,starts,goals,yaws

def make_world(name):
    nodes,edges,starts,goals,yaws=scenario(name)
    world=(ROOT/'Webots/worlds/bel_warehouse_ros.wbt').read_text()
    colors=['.08 .42 .72','.12 .65 .46','.91 .36 .12']
    for i in range(3):
        x,y=nodes[starts[i]] if i<len(starts) else (15.8,-2)
        yaw=yaws[i] if i<len(yaws) else math.pi
        block=f'DEF AMR{i+1:02} AMR {{\n translation {x} {y} 0\n rotation 0 0 1 {yaw}\n name "AMR{i+1:02}"\n color {colors[i]}\n controller "amr_ros"\n}}'
        world=re.sub(r'DEF AMR'+f'{i+1:02}'+r' AMR \{[^}]*\}',lambda m:block,world)
    world=world.replace('controller "ros_observer"','controller "coordination_observer"')
    path=ROOT/f'Webots/worlds/coordination_{name}.wbt'
    path.write_text(world)
    return path

def probe(name,limit):
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('coordination_test_probe')
    nodes,edges,starts,goals,yaws=scenario(name)
    ids=[f'amr_{i+1:02}' for i in range(len(starts))]
    physics=[];events=[];results={};states={}
    node.create_subscription(String,'/validation/physics',lambda m:physics.append(json.loads(m.data)),10)
    node.create_subscription(String,'/task/result',lambda m:results.update({json.loads(m.data)['robot_id']:json.loads(m.data)}),10)
    pubs={};reservations={}
    for rid in ids:
        node.create_subscription(String,f'/{rid}/coordination_event',lambda m:events.append(json.loads(m.data)),100)
        node.create_subscription(String,f'/{rid}/telemetry',lambda m,r=rid:states.update({r:json.loads(m.data)}),10)
        node.create_subscription(String,f'/{rid}/reservation',lambda m,r=rid:reservations.update({r:json.loads(m.data)}),10)
        pubs[rid]=node.create_publisher(String,f'/{rid}/assigned_task',10)
    def spin(until,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if until():return
        raise RuntimeError('Physical scenario timeout')
    report=dict(scenario=name,status='FAIL')
    try:
        spin(lambda:physics and all(
            states.get(r,{}).get('bootstrap_ready') and r in reservations and
            math.dist((states[r]['x'],states[r]['y']),nodes[starts[i]])<.25
            for i,r in enumerate(ids)),360)
        initial=physics[-1];run=f'coordination_{name}_{time.time_ns()}'
        spec=dict(run=run,members=ids,nodes=nodes,edges=edges)
        for i,rid in enumerate(ids):
            x,y=nodes[goals[i]]
            pubs[rid].publish(String(data=json.dumps(dict(task_id=run+'_'+rid,kind='navigate',
                              pickup_x=x,pickup_y=y,coordination=spec))))
        spin(lambda:all(reservations.get(r,{}).get('run')==run for r in ids),15)
        spin(lambda:all(r in results for r in ids)
             or (physics and physics[-1]['collision_episodes']>0)
             or any(reservations.get(r,{}).get('failed') for r in ids)
             or any(e['event']=='COORDINATION FAILED' for e in events),limit)
        assert physics[-1]['collision_episodes']==0,'Physical contact detected; aborting scenario'
        assert all(results.get(r,{}).get('status')=='COMPLETED' for r in ids),results
        final=physics[-1]
        assert final['collision_episodes']==0,final
        assert final['minimum_separation']>1.15,final
        assert any(e['event']=='PIBT CONFLICT' for e in events),events
        assert any(e['event']=='PIBT RESOLUTION' for e in events),events
        if name=='B':assert any(e['event']=='RESOLUTION METHOD' and e['success'] for e in events)
        metrics={}
        for i,rid in enumerate(ids):
            track=[p['poses'][rid][:2] for p in physics if p['time']>=initial['time']]
            length=sum(math.dist(a,b) for a,b in zip(track,track[1:]))
            assert math.dist(track[-1],nodes[goals[i]])<.25,(rid,track[-1])
            complete=[e for e in events if e['robot_id']==rid and e['event']=='COORDINATION COMPLETED'][-1]
            metrics[rid]=dict(complete['metrics'],physical_path_length=length,
                              travel_time=complete['simulation_time']-initial['time'],
                              route_efficiency=math.dist(track[0],nodes[goals[i]])/(length+math.dist(track[-1],nodes[goals[i]])),
                              final_position=track[-1],task_completion=True)
        report.update(status='PASS',metrics=metrics,minimum_separation=final['minimum_separation'],
                      collisions=final['collision_episodes'])
    except Exception as error:
        report['error']=str(error)
        raise
    finally:
        report.update(events=events,results=results,physics=physics)
        (OUT/f'coordination_{name}_result.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('events','physics')},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('scenario',choices=['A','B','C'])
    parser.add_argument('--probe',action='store_true')
    parser.add_argument('--limit',type=float,default=1200)
    args=parser.parse_args()
    if args.probe:return probe(args.scenario,args.limit)
    world=make_world(args.scenario)
    processes=[];logs=[]
    try:
        env=dict(os.environ,ROS_DOMAIN_ID='27',LP_NUM_THREADS='2',OMP_NUM_THREADS='2')
        commands=[
            ['/usr/local/bin/webots','--batch','--minimize','--no-rendering','--mode=fast','--stdout','--stderr','--port=1237',str(world)],
            ['ros2','launch','bel_amr_ros','webots_fleet.launch.py']]
        for label,command in zip(['world','fleet'],commands):
            log=(OUT/f'coordination_{args.scenario}_{label}.log').open('w');logs.append(log)
            processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        probe_process=subprocess.Popen(['python3',__file__,args.scenario,'--probe','--limit',str(args.limit)],env=env,start_new_session=True)
        processes.append(probe_process)
        code=probe_process.wait()
        raise SystemExit(code)
    finally:
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in processes:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
        for log in logs:log.close()
if __name__=='__main__':main()
