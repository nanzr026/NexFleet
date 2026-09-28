"""Isolated Gazebo carry test; leaves the warehouse runtime untouched."""
import os, subprocess, time, pathlib, xml.etree.ElementTree as ET, json, re, signal
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'demo_logs'/'local_physics'
OUT.mkdir(exist_ok=True)
env=dict(os.environ,GZ_PARTITION='bel_local_carry_diagnostic')
sdf=ET.Element('sdf',version='1.9')
world=ET.SubElement(sdf,'world',name='carry_lab')
ET.SubElement(world,'gravity').text='0 0 -9.8'
physics=ET.SubElement(world,'physics',name='physics',type='ignored')
ET.SubElement(physics,'max_step_size').text=os.environ.get('CARRY_STEP','0.01')
ET.SubElement(physics,'real_time_factor').text='1'
for name,filename in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster')]:
    ET.SubElement(world,'plugin',name='gz::sim::systems::'+name,filename='gz-sim-'+filename+'-system')
ground=ET.SubElement(world,'model',name='ground')
ET.SubElement(ground,'static').text='true'
link=ET.SubElement(ground,'link',name='ground')
c=ET.SubElement(link,'collision',name='ground')
g=ET.SubElement(c,'geometry')
plane=ET.SubElement(g,'plane')
ET.SubElement(plane,'normal').text='0 0 1'
ET.SubElement(plane,'size').text='100 100'
for name,path,pose in [('amr_02','amr_02','0 0 0.01 0 0 0'),('pallet_box_mobile','product_01_mobile','1.2 0 0.17 0 0 0')]:
    model=ET.parse(ROOT/'Gazebo Harmonic/models'/path/'model.sdf').getroot().find('model')
    model.set('name',name)
    for elem in list(model.findall('pose')): model.remove(elem)
    ET.SubElement(model,'pose').text=pose
    for link in model.findall('link'):
        for elem in list(link.findall('sensor'))+list(link.findall('visual')): link.remove(elem)
    world.append(model)
ET.indent(sdf)
path=OUT/'world.sdf'
ET.ElementTree(sdf).write(path)
def cmd(args,timeout=10):
    return subprocess.run(['gz']+args,env=env,capture_output=True,text=True,timeout=timeout)
def publish(topic,typ,value):
    r=cmd(['topic','-t',topic,'-m',typ,'-p',value])
    if r.returncode: raise RuntimeError(r.stderr)
def sample(label):
    r=cmd(['topic','-e','-t','/world/carry_lab/pose/info','-n','1'],20)
    (OUT/(label+'.txt')).write_text(r.stdout)
    # Parse only model blocks (nested position and orientation braces).
    poses={}
    for block in re.split(r'(?m)^pose \{',r.stdout)[1:]:
        name=re.search(r'name: "([^"]+)"',block)
        pos=re.search(r'position \{([^}]+)',block)
        if name and pos and name[1] in ('amr_02','pallet_box_mobile'):
            poses[name[1]]={k:float(v) for k,v in re.findall(r'([xyz]): ([^\s]+)',pos[1])}
    print(label,poses,flush=True)
    return poses
log=(OUT/'server.log').open('w')
server=subprocess.Popen(['gz','sim','-s','-r',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    time.sleep(5)
    sample('attached_start')
    publish('/model/amr_02/cmd_vel','gz.msgs.Twist','linear: {x: -0.08}')
    time.sleep(7)
    sample('attached_end')
    publish('/model/amr_02/cmd_vel','gz.msgs.Twist','linear: {x: 0}')
    publish('/amr_02/product_01/detach','gz.msgs.Empty','')
    time.sleep(1)
    sample('detached_start')
    publish('/model/amr_02/cmd_vel','gz.msgs.Twist','linear: {x: -0.08}')
    time.sleep(7)
    sample('detached_end')
    publish('/model/amr_02/cmd_vel','gz.msgs.Twist','linear: {x: 0}')
    publish('/amr_02/product_01/attach','gz.msgs.Empty','')
    time.sleep(1)
    sample('reattached_start')
    publish('/model/amr_02/cmd_vel','gz.msgs.Twist','linear: {x: -0.08}')
    time.sleep(7)
    sample('reattached_end')
finally:
    os.killpg(server.pid,signal.SIGINT)
    server.wait(timeout=15)
    log.close()
