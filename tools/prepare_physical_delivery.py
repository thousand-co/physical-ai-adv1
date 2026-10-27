"""Build Gazebo contact-physics inputs from the same CRANE+ model as training."""
import copy
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'ros2_ws/src/final_project_ros'
FRICTION=float(os.environ.get('PHYSICAL_FRICTION','1.0'))
MASS=float(os.environ.get('PHYSICAL_MASS','.03'))
SEED=os.environ.get('PHYSICAL_SEED')
if not .01<=MASS<=.1 or not .3<=FRICTION<=2.:raise ValueError('Physical mass/friction outside experimental limits')
DESC=Path(os.environ.get('CRANE_WS','/home/ubuntu/airobot_ws'))/'src/crane_plus/crane_plus_description'

def box_geometry(parent,size):
    ET.SubElement(ET.SubElement(parent,'geometry'),'box').append(ET.Element('size'))
    parent.find('geometry/box/size').text=' '.join(map(str,size))

def surface(collision,friction=FRICTION):
    s=ET.SubElement(collision,'surface')
    f=ET.SubElement(ET.SubElement(s,'friction'),'ode')
    ET.SubElement(f,'mu').text=str(friction);ET.SubElement(f,'mu2').text=str(friction)
    ode=ET.SubElement(ET.SubElement(s,'contact'),'ode')
    for key,value in [('kp','100000'),('kd','10'),('max_vel','0.1'),('min_depth','0.0002')]:
        ET.SubElement(ode,key).text=value

def make_world():
    tree=ET.parse(PKG/'worlds/simple_delivery.world');world=tree.getroot().find('world')
    world.set('name','physical_delivery')
    for model in list(world.findall('model')):
        if model.get('name') not in ['table','camera_mount']:
            world.remove(model)
    for include in list(world.findall('include')):world.remove(include)
    scene=ET.SubElement(world,'scene')
    ET.SubElement(scene,'ambient').text='.45 .45 .45 1'
    ET.SubElement(scene,'background').text='.14 .17 .21 1'
    physics=world.find('physics')
    ode=ET.SubElement(physics,'ode');solver=ET.SubElement(ode,'solver')
    ET.SubElement(solver,'type').text='quick'
    constraints=ET.SubElement(ode,'constraints')
    ET.SubElement(constraints,'cfm').text='0.000001'
    ET.SubElement(constraints,'erp').text='0.2'
    ET.SubElement(solver,'iters').text='100'
    light=ET.SubElement(world,'light',name='task_light',type='directional')
    ET.SubElement(light,'pose').text='0 0 2 0 0 0';ET.SubElement(light,'diffuse').text='.6 .6 .6 1';ET.SubElement(light,'direction').text='-.2 .2 -1'
    target=np.array([.205,-.025,.071])
    if SEED is not None:target[:2]+=np.random.default_rng(int(SEED)).uniform([-.008,-.015],[.008,.015])
    (PKG/'config/physical_scene.json').write_text(json.dumps(dict(seed=SEED,mass_kg=MASS,friction=FRICTION,target_initial_position=target.tolist()),indent=2))
    for name,xyz,rgba in [('target_blue_00',target.tolist(),[.08,.35,.9,1]),('distractor_red',[.31,.055,.071],[.85,.1,.1,1])]:
        model=ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'pose').text=' '.join(map(str,xyz))+' 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        mass=MASS if name.startswith('target') else .03
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text=str(mass)
        inertia=ET.SubElement(inertial,'inertia')
        for axis in ['ixx','iyy','izz']: ET.SubElement(inertia,axis).text=str(mass*.04**2/6)
        collision=ET.SubElement(link,'collision',name='box_collision');box_geometry(collision,[.04]*3);surface(collision)
        visual=ET.SubElement(link,'visual',name='visual');box_geometry(visual,[.04]*3)
        mat=ET.SubElement(visual,'material')
        for key in ['ambient','diffuse']:ET.SubElement(mat,key).text=' '.join(map(str,rgba))
        if name.startswith('target'):
            sensor=ET.SubElement(link,'sensor',name='box_contacts',type='contact')
            ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='100'
            ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='box_collision'
            plugin=ET.SubElement(sensor,'plugin',name='box_contact_ros',filename='libgazebo_ros_bumper.so')
            ros=ET.SubElement(plugin,'ros');ET.SubElement(ros,'remapping').text='bumper_states:=/grasp/contacts'
            ET.SubElement(plugin,'frame_name').text='world'
    tray=ET.SubElement(world,'model',name='delivery_tray');ET.SubElement(tray,'static').text='true'
    ET.SubElement(tray,'pose').text='.075 -.175 .05 0 0 0';link=ET.SubElement(tray,'link',name='tray')
    pieces=[('floor',[0,0,.005],[.15,.15,.01]),('left',[-.075,0,.0225],[.01,.16,.045]),('right',[.075,0,.0225],[.01,.16,.045]),('front',[0,-.075,.0225],[.14,.01,.045]),('back',[0,.075,.0225],[.14,.01,.045])]
    for name,pos,size in pieces:
        for kind in ['collision','visual']:
            element=ET.SubElement(link,kind,name=name+'_'+kind);ET.SubElement(element,'pose').text=' '.join(map(str,pos))+' 0 0 0';box_geometry(element,size)
            if kind=='collision':surface(element)
            else:
                mat=ET.SubElement(element,'material')
                for key in ['ambient','diffuse']:ET.SubElement(mat,key).text='.85 .65 .15 1'
    # Separate oblique camera shows physical finger/box contact and delivery.
    camera=copy.deepcopy(world.find("model[@name='camera_mount']"));camera.set('name','demo_camera')
    camera.find('pose').text='.66 -.66 .48 0 .45 2.33'
    sensor=camera.find('link/sensor');sensor.set('name','demo_camera_sensor')
    sensor.find('camera/horizontal_fov').text='.95'
    sensor.find('camera/image/width').text='960';sensor.find('camera/image/height').text='720'
    sensor.find('plugin/ros/namespace').text='/demo_camera'
    world.append(camera)
    ET.indent(tree);tree.write(PKG/'worlds/physical_delivery.world')

def make_robot():
    clouds={}
    for name,file in [('fixed','Link4_collision_20201013.stl'),('hand','Hand_collision_20201013.stl')]:
        raw=(DESC/'meshes/collision'/file).read_bytes()
        count=int.from_bytes(raw[80:84],'little')
        triangles=np.frombuffer(raw,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]),count=count,offset=84)
        clouds[name]=np.unique(triangles['vertices'].reshape(-1,3),axis=0)
    np.savez(PKG/'config/gripper_collision_vertices.npz',**clouds)
    config=PKG/'config/physical_controllers.yaml'
    joints=['crane_plus_joint'+str(i) for i in range(1,5)]+['crane_plus_joint_hand']
    controllers={'controller_manager':{'ros__parameters':{'update_rate':500,'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},'crane_plus_arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'},'crane_plus_gripper_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}}}
    for name,names in [('crane_plus_arm_controller',joints[:4]),('crane_plus_gripper_controller',joints[4:])]:
        controllers[name]={'ros__parameters':{'joints':names,'command_interfaces':['effort'],'state_interfaces':['position','velocity'],'gains':{j:{'p':35.,'i':0.,'d':1.5,'i_clamp':0.} for j in names},'state_publish_rate':50.,'action_monitor_rate':20.}}
    config.write_text(yaml.safe_dump(controllers,sort_keys=False))
    text=subprocess.check_output(['xacro',str(DESC/'urdf/crane_plus.urdf.xacro'),'use_gazebo:=true','gz_control_config_package:=crane_plus_control','gz_control_config_file_path:=config/crane_plus_controllers.yaml'],text=True)
    robot=ET.fromstring(text)
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename',str(DESC/mesh.get('filename').split('crane_plus_description/')[-1]))
    for color in robot.findall('material/color'):
        if color.get('rgba','').startswith('0.95'):color.set('rgba','0.7 0.72 0.75 1')
    for joint in robot.findall('joint'):
        dynamics=joint.find('dynamics')
        if dynamics is not None:dynamics.set('damping','.3');dynamics.set('friction','.02')
    for joint in robot.findall('joint'):
        if joint.get('type')=='revolute':
            gazebo=ET.SubElement(robot,'gazebo',reference=joint.get('name'))
            ET.SubElement(gazebo,'implicitSpringDamper').text='true'
    for command in robot.findall('ros2_control/joint/command_interface'):
        command.set('name','effort')
        for child in list(command):command.remove(child)
        ET.SubElement(command,'param',name='min').text='-1.5';ET.SubElement(command,'param',name='max').text='1.5'
    robot.find("gazebo/plugin[@name='gazebo_ros2_control']/parameters").text=str(config)
    for gazebo in robot.findall('gazebo'):
        if gazebo.get('reference'):
            for name in ['mu1','mu2']:
                elem=gazebo.find(name)
                if elem is not None:elem.text=str(FRICTION)
            for key,value in [('kp','100000'),('kd','10'),('maxVel','0.1'),('minDepth','0.0002'),('selfCollide','false')]:ET.SubElement(gazebo,key).text=value
    ET.indent(robot);ET.ElementTree(robot).write(PKG/'config/physical_crane.urdf')
    (PKG/'models/learned_grasp_policy.json').write_text((ROOT/'physics_grasp/results/policy.json').read_text())

if __name__=='__main__':
    make_world();make_robot();print('Prepared physical world, force controllers, CRANE+ URDF, and learned policy')
