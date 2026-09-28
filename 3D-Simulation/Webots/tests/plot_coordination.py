"""Plot measured Supervisor trajectories and separation from completed test reports."""
import json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
colors={'amr_01':'#2176ae','amr_02':'#15966a','amr_03':'#df7828'}
fig,axes=plt.subplots(2,3,figsize=(13,8),gridspec_kw={'height_ratios':[1.5,1]})
for column,name in enumerate(('A','B','C')):
    report=json.loads((ROOT/f'Webots/logs/coordination_{name}_result.json').read_text())
    assert report['status']=='PASS'
    data=report['physics'];active=list(report['metrics'])
    top,bottom=axes[:,column]
    for rid in active:
        points=[d['poses'][rid] for d in data]
        top.plot([p[0] for p in points],[p[1] for p in points],color=colors[rid],lw=2,label=rid.upper())
        top.scatter(*points[0][:2],marker='o',s=38,color=colors[rid])
        top.scatter(*points[-1][:2],marker='*',s=100,color=colors[rid])
    top.set_aspect('equal',adjustable='datalim')
    top.set_xlabel('World x (m)');top.set_ylabel('World y (m)')
    top.set_title(f'Test {name}: '+{'A':'crossing / detour','B':'opposing aisle / LaCAM','C':'three AMRs'}[name])
    top.grid(alpha=.2);top.legend(fontsize=8)
    timestamps=[d['time']-data[0]['time'] for d in data]
    separation=[min(math.dist(d['poses'][a][:2],d['poses'][b][:2])
                    for i,a in enumerate(active) for b in active[:i]) for d in data]
    bottom.plot(timestamps,separation,color='#323e4c',lw=1.4)
    bottom.axhline(1.3,color='#b14843',ls='--',lw=1,label='Planning clearance')
    activations=[e for e in report['events'] if e['event']=='RESOLUTION METHOD' and e['robot_id']==active[0]]
    for e in activations:
        bottom.axvline(e['simulation_time']-data[0]['time'],color='#9652a1',ls=':',label='LaCAM activated')
    bottom.set_ylim(bottom=0)
    bottom.set_xlabel('Simulation time (s)');bottom.set_ylabel('Nearest pair distance (m)')
    bottom.set_title(f"Minimum {report['minimum_separation']:.2f} m; contacts {report['collisions']}",fontsize=10)
    bottom.grid(alpha=.2);bottom.legend(fontsize=8)
fig.suptitle('Physical Webots validation — measured motion, final acceptance runs',fontsize=14)
fig.tight_layout()
fig.savefig(ROOT/'Webots/coordination_validation.png',dpi=160)
