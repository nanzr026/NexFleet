"""Reproduce baseline from fresh initial poses, without editing saved user worlds."""
import json,math,os,re,signal,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'Webots/logs'
env=dict(os.environ,ROS_DOMAIN_ID='26',LP_NUM_THREADS='2',OMP_NUM_THREADS='2',BEL_WEBOTS_AUTOTEST='1')
def world(ros):
    text=(ROOT/'Webots/worlds/bel_warehouse_ros.wbt').read_text()
    for i,y in enumerate((-10,-6,-2),1):
        controller='amr_ros' if ros else 'amr_motion_test'
        block=f'DEF AMR{i:02} AMR {{ translation 15.8 {y} 0 rotation 0 0 1 {math.pi} name "AMR{i:02}" controller "{controller}" }}'
        text=re.sub(r'DEF AMR'+f'{i:02}'+r' AMR \{[^}]*\}',lambda m:block,text)
    if not ros:text=text.replace('controller "ros_observer"','controller "validation_supervisor"')
    path=ROOT/('Webots/worlds/regression_ros.wbt' if ros else 'Webots/worlds/regression_motion.wbt')
    path.write_text(text)
    return path
def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
def launch(path,label):
    log=(OUT/label).open('w')
    p=subprocess.Popen(['/usr/local/bin/webots','--batch','--minimize','--no-rendering','--mode=fast','--stdout','--stderr','--port=1235',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return p,log
def main():
    sim,log=launch(world(False),'regression_motion.log')
    try:assert sim.wait(timeout=360)==0
    finally:stop(sim);log.close()
    sim,log=launch(world(True),'regression_interface.log')
    try:
        subprocess.run(['python3',str(ROOT/'Webots/tests/verify_ros_interface.py')],env=env,check=True,timeout=900)
        subprocess.run(['python3',str(ROOT/'Webots/tests/verify_lidar_stop.py')],env=env,check=True,timeout=900)
    finally:stop(sim);log.close()
    sim,log=launch(world(True),'regression_navigation_world.log')
    fleet_log=(OUT/'regression_navigation_fleet.log').open('w')
    fleet=subprocess.Popen(['ros2','launch','bel_amr_ros','webots_fleet.launch.py'],env=env,stdout=fleet_log,stderr=subprocess.STDOUT,start_new_session=True)
    try:subprocess.run(['python3',str(ROOT/'Webots/tests/verify_navigation.py')],env=env,check=True,timeout=1200)
    finally:stop(fleet);stop(sim);log.close();fleet_log.close()
    print('Physical baseline regressions completed',flush=True)
if __name__=='__main__':main()
