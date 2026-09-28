"""Start warehouse validation with physics/LiDAR intact and unused RGB cameras omitted."""
import os, subprocess, time, signal
from pathlib import Path
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
logs=root/'demo_logs'
# Ensure the previous server has exited; never start competing publishers.
pidfile=logs/'headless.pid'
if pidfile.exists():
    pid=int(pidfile.read_text())
    try:
        os.kill(pid,0)
    except ProcessLookupError:
        pass
    else:
        raise SystemExit('Stop the existing Gazebo server first')
models=logs/'validation_models'
for rid in ('amr_01','amr_02','amr_03'):
    folder=models/rid
    folder.mkdir(parents=True,exist_ok=True)
    src=root/'Gazebo Harmonic/models'/rid
    tree=ET.parse(src/'model.sdf')
    for link in tree.findall('.//link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type')=='camera':
                link.remove(sensor)
    tree.write(folder/'model.sdf')
    (folder/'model.config').write_bytes((src/'model.config').read_bytes())
env=dict(os.environ)
env['GZ_SIM_RESOURCE_PATH']=str(models)+':'+str(root/'Gazebo Harmonic/models')+':'+env.get('GZ_SIM_RESOURCE_PATH','')
env['LP_NUM_THREADS']='2'
env['OMP_NUM_THREADS']='2'
path=logs/'headless.log'
if path.exists():path.rename(logs/f'headless-{time.time_ns()}.log')
with path.open('w') as f:
    p=subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(root/'Gazebo Harmonic/worlds/openrobotics_warehouse.sdf')],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
pidfile.write_text(str(p.pid))
print('headless validation',p.pid,flush=True)
