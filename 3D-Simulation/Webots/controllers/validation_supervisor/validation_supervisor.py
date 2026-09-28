"""Read-only physical measurements; never assigns tasks or moves robots."""
import json,math,os
from pathlib import Path
from controller import Supervisor
s=Supervisor()
step=int(s.getBasicTimeStep())
root=Path(__file__).resolve().parents[2]
out=root/'logs'
out.mkdir(exist_ok=True)
robots={name:s.getFromDef(name) for name in ('AMR01','AMR02','AMR03')}
history=[]
def sample():
    states={}
    for name,node in robots.items():
        p=node.getPosition();r=node.getOrientation()
        states[name]={'position':p,'yaw':math.atan2(r[3],r[0])}
    return {'time':s.getTime(),'robots':states}
def angle(x):return math.atan2(math.sin(x),math.cos(x))
saved=False
last_progress=-1
while s.step(step)!=-1:
    history.append(sample())
    if int(s.getTime())!=last_progress:
        last_progress=int(s.getTime())
        print('MOTION',last_progress,history[-1]['robots'],flush=True)
    if not saved and s.getTime()>1:
        s.exportImage(str(out/'warehouse_overview.png'),95)
        saved=True
    if s.getTime()>=17:
        def near(t):return min(history,key=lambda h:abs(h['time']-t))['robots']
        start,forward,left,right,stop,end=[near(t) for t in (2,8,11,14,15,17)]
        results={}
        for name in robots:
            p0,p1=start[name]['position'],forward[name]['position']
            yaw=start[name]['yaw']
            projection=(p1[0]-p0[0])*math.cos(yaw)+(p1[1]-p0[1])*math.sin(yaw)
            turn_left=angle(left[name]['yaw']-forward[name]['yaw'])
            turn_right=angle(right[name]['yaw']-left[name]['yaw'])
            drift=math.dist(stop[name]['position'],end[name]['position'])
            results[name]={'forward_m':projection,'left_rad':turn_left,'right_rad':turn_right,'stop_drift_m':drift,
                'pass':projection>1.2 and turn_left>.9 and turn_right<-.9 and drift<.03}
        report={'status':'PASS' if all(r['pass'] for r in results.values()) else 'FAIL','robots':results,'history':history}
        (out/'motion_result.json').write_text(json.dumps(report,indent=2))
        s.exportImage(str(out/'motion_complete.png'),95)
        print(json.dumps({k:v for k,v in report.items() if k!='history'}),flush=True)
        if os.environ.get('BEL_WEBOTS_AUTOTEST')=='1':s.simulationQuit(0 if report['status']=='PASS' else 1)
        else:s.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        break
