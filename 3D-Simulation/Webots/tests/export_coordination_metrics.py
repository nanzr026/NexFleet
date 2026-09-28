"""Export dashboard-ready metrics from preserved physical evidence.

Does not run a planner, issue commands, or alter the original reports.
Fleet event counts deduplicate independently logged peer observations.
"""
import argparse,collections,json,math
from pathlib import Path
from run_coordination import scenario
ROOT=Path(__file__).resolve().parents[2]

def export(name):
    path=ROOT/f'Webots/logs/coordination_{name}_result.json'
    raw=json.loads(path.read_text())
    assert raw['status']=='PASS',name
    nodes,edges,starts,goals,yaws=scenario(name)
    physics=raw['physics'];events=raw['events']
    ids=list(raw['metrics'])
    robots={}
    for i,rid in enumerate(ids):
        m=raw['metrics'][rid]
        initial=physics[0]['poses'][rid][:2]
        final=m['final_position'];goal=nodes[goals[i]]
        assert math.dist(final,goal)<.25 and raw['results'][rid]['status']=='COMPLETED'
        robots[rid]=dict(start_position_m=initial,goal_position_m=goal,final_position_m=final,
            completion_time_s=m['travel_time'],path_length_m=m['physical_path_length'],
            average_speed_m_per_s=m['physical_path_length']/m['travel_time'],
            waiting_time_s=m['waiting_time'],replans=m['replans'],final_state='COMPLETED',
            goal_completed=True,conflicts_detected=m['conflicts'],
            pibt_resolutions=m['pibt_resolutions'],lacam_activations=m['lacam_activations'],
            deadlock_detections=m['deadlocks'],route_efficiency=m['route_efficiency'])
    conflicts={tuple(sorted(e['robots']))+(e['type'],e['timestep'])
               for e in events if e['event']=='PIBT CONFLICT'}
    activations={e['sequence'] for e in events if e['event']=='RESOLUTION METHOD'}
    deadlocks={e['sequence'] for e in events if e['event']=='DEADLOCK CONFIRMED'}
    resolutions={e['sequence'] for e in events if e['event']=='PIBT RESOLUTION'}
    makespan=max(r['completion_time_s'] for r in robots.values())
    distance=sum(r['path_length_m'] for r in robots.values())
    fleet=dict(makespan_s=makespan,total_distance_m=distance,
        average_fleet_speed_m_per_s=sum(r['average_speed_m_per_s'] for r in robots.values())/len(ids),
        distance_throughput_m_per_s=distance/makespan,
        total_waiting_time_s=sum(r['waiting_time_s'] for r in robots.values()),
        conflicts_detected=len(conflicts),pibt_resolutions=len(resolutions),
        lacam_activations=len(activations),
        lacam_agent_invocations=sum(r['lacam_activations'] for r in robots.values()),
        deadlock_detections=len(deadlocks),
        deadlock_agent_detections=sum(r['deadlock_detections'] for r in robots.values()),
        minimum_physical_separation_m=raw['minimum_separation'],
        physical_contact_episodes=raw['collisions'],
        completed_goals=len(ids),total_goals=len(ids),permanent_deadlocks=0)
    evidence=[]
    for rid in ids:
        local=[e for e in events if e['robot_id']==rid]
        for detected in (e for e in local if e['event']=='DEADLOCK CONFIRMED'):
            sequence=detected['sequence']
            earlier=[e for e in local if e['sequence']<sequence and e['event']=='PIBT RESOLUTION']
            history=[tuple(e['configuration']) for e in earlier]
            counts=collections.Counter(history)
            hold=[e for e in local if e['event']=='PIBT DECISION' and e['sequence']==sequence][-1]
            current=tuple(hold['configuration'])
            visits=[i for i,q in enumerate(history) if q==current]
            evidence.append(dict(robot_id=rid,reason='repeated_joint_configuration',
                robots=ids,simulation_time_s=detected['simulation_time'],
                bounded_no_progress_window_s=detected['duration'],
                repeated_states=[dict(configuration=list(q),visits=n) for q,n in counts.items() if n>1],
                no_progress_steps=visits[-1]-visits[-2] if len(visits)>1 else None,
                runtime_reason=next(e['reason'] for e in local if e['event']=='DEADLOCK SUSPECTED'
                                    and e['sequence']==sequence),
                source_sequence=sequence))
    decisions=[e for e in events if e['event'] in ('PIBT DECISION','LaCAM DECISION')]
    agreed=True
    for seq in {e['sequence'] for e in decisions}:
        group=[e for e in decisions if e['sequence']==seq]
        assert {e['robot_id'] for e in group}==set(ids)
        assert len({e['signature'] for e in group})==1
    verified=dict(activated=bool(activations),physically_validated=False,
                  reservation_agreement_verified=agreed,executed_joint_steps=0)
    if activations:
        first=min(e['simulation_time'] for e in events if e['event']=='RESOLUTION METHOD')
        leader=ids[0]
        local=[e for e in events if e['robot_id']==leader]
        lacam_steps=[e for e in local if e['event']=='LaCAM DECISION']
        errors=[]
        for decision in lacam_steps:
            next_events=[e for e in local if e['sequence']==decision['sequence']+1 and e['event'] in ('PIBT DECISION','LaCAM DECISION')]
            if next_events:end=next_events[0]['simulation_time']
            else:end=next(e['simulation_time'] for e in local if e['event']=='COORDINATION COMPLETED')
            sample=min(physics,key=lambda p:abs(p['time']-end))
            for i,rid in enumerate(ids):
                errors.append(math.dist(sample['poses'][rid][:2],nodes[decision['next_configuration'][i]]))
        assert errors and max(errors)<.25,(name,max(errors))
        travel={}
        for rid in ids:
            track=[p['poses'][rid][:2] for p in physics if p['time']>=first]
            travel[rid]=sum(math.dist(a,b) for a,b in zip(track,track[1:]))
            assert travel[rid]>.5
        verified.update(physically_validated=True,executed_joint_steps=len(lacam_steps),
                        maximum_measured_arrival_error_m=max(errors),physical_distance_after_activation_m=travel)
    return dict(status='PASS',robots=robots,fleet=fleet,deadlock_evidence=evidence,
                lacam_physical_verification=verified,evidence_file=str(path.relative_to(ROOT)))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('scenarios',nargs='*',default=['A','B','C'])
    args=parser.parse_args()
    result=dict(schema_version=1,scope='final_accepted_runs',
        historical_failures=[
            dict(run='initial_independent_execution',physical_contact_episodes=1,
                 evidence_file='Webots/logs/failed_independent_execution_physics.jsonl'),
            dict(run='first_C_readiness',physical_contact_episodes=0,status='TASK_REJECTED',
                 evidence_file='Webots/logs/failed_C_readiness_result.json')],
        coordinate_frame='webots_world',time_basis='simulation_seconds',
        definitions=dict(completion_time='Session task completion including reservation barriers',
            waiting_time='Time with movement authorization withheld, including waits after own goal arrival',
            average_fleet_speed='Arithmetic mean of per-robot path length / completion time',
            fleet_event_counts='Unique joint step/conflict episodes; peer invocation totals are separate',
            no_progress_steps='Steps between the last two visits to the held joint configuration',
            contacts='Non-floor contact episodes above z=0.04 m; evaluated every 16 ms physics step',
            goal_tolerance_m=.25),
        tests={name:export(name) for name in args.scenarios})
    out=ROOT/'Webots/coordination_metrics.json'
    out.write_text(json.dumps(result,indent=2))
    for name,test in result['tests'].items():
        (ROOT/f'Webots/logs/coordination_{name}_metrics.json').write_text(json.dumps(test,indent=2))
    print(json.dumps({name:t['fleet'] for name,t in result['tests'].items()},indent=2))
if __name__=='__main__':main()
