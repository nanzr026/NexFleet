"""Run the complete workflow from a short approach, preserving measured evidence."""
import json, time, math
from pathlib import Path
import rclpy
from verify_demo import Probe, set_pose
rclpy.init()
n=Probe()
report={}
try:
    n.wait(lambda:n.telemetry.get('amr_02',{}).get('bootstrap_ready') and
        n.telemetry.get('amr_02',{}).get('state')=='IDLE' and
        'pallet_box_mobile' in n.poses,120,'Waiting for idle detached controller')
    set_pose('pallet_box_mobile',4.4161,14.6952,0.17)
    set_pose('amr_02',4.4161,17.0,0.10)
    start=time.monotonic()
    n.wait(lambda:time.monotonic()-start>4,15,'Settling local setup')
    initial=n.poses['pallet_box_mobile']
    report['result']=n.task('pickup',(4.4161,14.6952),delivery=(8.0,18.5),auction=True,timeout=600)
    samples=[h for h in n.history if h['telemetry'].get('amr_02',{}).get('state')=='MOVING_TO_PICKUP']
    report['approach_drift']=max(math.dist(h['poses']['pallet_box_mobile'],initial) for h in samples)
    assert report['approach_drift']<0.15
    assert report['result']['carry_verified']
    report['status']='PASS'
except Exception as e:
    report.update(status='FAIL',error=str(e))
    raise
finally:
    report.update(history=n.history,final_poses=n.poses,bids=n.bids,claims=n.claims)
    (Path(__file__).resolve().parents[1]/'demo_logs/local_workflow_result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2),flush=True)
    n.destroy_node()
    rclpy.shutdown()
