"""Learned grasp -> contact-validated transport -> gravity release in Gazebo.

This node only reads object poses. It never calls a state-setting/attachment API.
"""
import json
from pathlib import Path
import time
import subprocess
import shutil
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates, ContactsState
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .physical_plan import ARM_JOINTS,HAND_JOINT,BASE,TRAY,OPEN,fk,solve,grasp_plan,placement_confirmed,box_bounds,gripper_bottom
from .motion_profile import plan_joint_motion

class PhysicalDelivery(Node):
    def __init__(self):
        super().__init__('physical_delivery')
        self.declare_parameter('output','artifacts/physical_delivery/episode.json')
        self.declare_parameter('disable_grip',False)
        self.output=Path(self.get_parameter('output').value);self.output.parent.mkdir(parents=True,exist_ok=True)
        self.disable_grip=bool(self.get_parameter('disable_grip').value)
        self.tray=TRAY.copy()
        share=Path(get_package_share_directory('final_project_ros'))
        vertices=np.load(share/'config/gripper_collision_vertices.npz')
        self.fixed_vertices=vertices['fixed'];self.hand_vertices=vertices['hand']
        self.scene_config=json.loads((share/'config/physical_scene.json').read_text())
        self.parameters=json.loads((share/'models/learned_grasp_policy.json').read_text())['parameters']
        self.joints={};self.models={};self.twists={};self.contact_times={};self.floor_contact_at=-1e6;self.events=[];self.samples=[]
        self.phase='initializing';self.phase_started=self.now();self.wall_started=time.monotonic();self.finished=False
        self.target='target_blue_00';self.detected_at=-1e6;self.goal=None;self.motion_end=0.;self.frames=[];self.frame_times=[];self.latest_frame=None;self.last_frame_at=-1.;self.last_sample=-1.;self.lift_confirmed=False;self.carry_confirmed=False
        self.command_peak_velocity=0.;self.command_peak_acceleration=0.;self.hand_setpoint=None
        self.bridge=CvBridge();self.start_models={}
        self.arm=self.create_publisher(JointTrajectory,'/crane_plus_arm_controller/joint_trajectory',10)
        self.hand=self.create_publisher(JointTrajectory,'/crane_plus_gripper_controller/joint_trajectory',10)
        self.state=self.create_publisher(String,'/task_state',10)
        self.create_subscription(JointState,'/joint_states',self.on_joints,20)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.on_models,10)
        self.create_subscription(ContactsState,'/grasp/contacts',self.on_contacts,qos_profile_sensor_data)
        self.create_subscription(PointStamped,'/target_point',self.on_detection,10)
        self.create_subscription(Image,'/demo_camera/color/image_raw',self.on_frame,qos_profile_sensor_data)
        self.create_timer(.02,self.update)
        self.transition('initializing')

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def on_joints(self,msg):self.joints.update(zip(msg.name,msg.position))
    def on_models(self,msg):
        self.models=dict(zip(msg.name,msg.pose));self.twists=dict(zip(msg.name,msg.twist))
    def on_contacts(self,msg):
        for state in msg.states:
            names=state.collision1_name+' '+state.collision2_name
            if self.target not in names:continue
            if 'delivery_tray' in names and 'floor_collision' in names:self.floor_contact_at=self.now()
            if 'crane_plus_link_hand' in names:self.contact_times['hand']=self.now()
            elif 'crane_plus_link4' in names:self.contact_times['fixed']=self.now()
    def on_detection(self,msg):
        if self.target not in self.models:return
        p=self.models[self.target].position;depth=1.4-p.z
        fx=640/(2*np.tan(1.3962634/2));fy=fx
        u=320-fx*(p.y-.15)/depth;v=240-fy*(p.x-.55)/depth
        if np.hypot(msg.point.x-u,msg.point.y-v)<18:self.detected_at=self.now()
    def on_frame(self,msg):self.latest_frame=self.bridge.imgmsg_to_cv2(msg,'bgr8')
    def object_position(self):
        p=self.models[self.target].position
        return np.array([p.x,p.y,p.z])
    def bilateral(self):return all(self.now()-self.contact_times.get(k,-1e6)<.2 for k in ['hand','fixed'])
    def transition(self,phase,reason=''):
        self.phase=phase;self.phase_started=self.now();self.events.append(dict(state=phase,time=self.now(),reason=reason))
        self.state.publish(String(data=phase));self.get_logger().info('Physical delivery: '+phase+(' '+reason if reason else ''))
    def fail(self,reason):
        self.transition('error',reason);self.finish()
    def move(self,q,hand,phase,minimum=1.):
        start=np.array([self.joints.get(j,0.) for j in ARM_JOINTS]+[self.joints.get(HAND_JOINT,OPEN)])
        if self.goal is not None:start[:4]=self.goal
        if self.hand_setpoint is not None:start[4]=self.hand_setpoint
        target=np.r_[q,hand]
        velocity,acceleration=(.5,.8) if phase=='transporting' else (1.,2.)
        duration,profile=plan_joint_motion(start.tolist(),target.tolist(),velocity,acceleration,minimum_duration=minimum,sample_period=.02)
        for publisher,names,indices in [(self.arm,ARM_JOINTS,list(range(4))),(self.hand,[HAND_JOINT],[4])]:
            trajectory=JointTrajectory();trajectory.joint_names=names
            for seconds,values,velocities,accelerations in profile:
                point=JointTrajectoryPoint();point.positions=[float(values[i]) for i in indices];point.velocities=[float(velocities[i]) for i in indices];point.accelerations=[float(accelerations[i]) for i in indices]
                ns=round(seconds*1e9);point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000);trajectory.points.append(point)
            publisher.publish(trajectory)
        delta=float(np.max(np.abs(target-start)))
        self.command_peak_velocity=max(self.command_peak_velocity,1.875*delta/duration)
        self.command_peak_acceleration=max(self.command_peak_acceleration,5.773503*delta/duration**2)
        self.hand_setpoint=hand
        self.goal=np.array(q);self.motion_end=self.now()+duration;self.transition(phase)
    def ready(self):
        if self.now()<self.motion_end+.25:return False
        error=max(abs(self.joints.get(j,100)-q) for j,q in zip(ARM_JOINTS,self.goal))
        if error<.10:return True
        if self.now()>self.motion_end+4:self.fail('joint_tracking_timeout: '+str(error))
        return False
    def update(self):
        if self.finished:return
        now=self.now()
        if time.monotonic()-self.wall_started>160:
            self.fail('wall_timeout');return
        if self.latest_frame is not None and now-self.last_frame_at>=.099:
            frame=self.latest_frame.copy()
            cv2.rectangle(frame,(0,0),(960,100),(25,25,25),-1)
            cv2.putText(frame,'LEARNED CONTACT GRASP | ROS 2 + GAZEBO DELIVERY',(16,32),0,.65,(255,255,255),2)
            cv2.putText(frame,f'{self.phase} | t={now:.1f}s | two-finger contact={self.bilateral()}',(16,65),0,.65,(240,240,240),1)
            self.frames.append(frame);self.frame_times.append(now);self.last_frame_at=now
        if self.target in self.models and now-self.last_sample>=.05:
            self.samples.append(dict(time=now,phase=self.phase,position=self.object_position().tolist(),bilateral_contact=self.bilateral(),floor_contact=now-self.floor_contact_at<.2,joints=[float(self.joints.get(j,0.)) for j in ARM_JOINTS+[HAND_JOINT]]));self.last_sample=now
        try:self.step()
        except Exception as exc:self.fail(type(exc).__name__+': '+str(exc))
    def step(self):
        if self.phase=='initializing':
            if not all(j in self.joints for j in ARM_JOINTS+[HAND_JOINT]) or self.target not in self.models or self.arm.get_subscription_count()==0 or self.hand.get_subscription_count()==0:return
            self.start_models={name:[p.position.x,p.position.y,p.position.z] for name,p in self.models.items() if name.startswith(('target','distractor'))}
            self.plan=grasp_plan(self.object_position()-BASE,self.parameters)
            self.move(self.plan['lift'],OPEN,'moving_to_start',2.)
        elif self.phase=='moving_to_start' and self.ready():self.transition('waiting_for_object')
        elif self.phase=='waiting_for_object':
            if self.now()-self.detected_at<1.:
                self.move(self.plan['pick'],OPEN,'approaching_target',max(1.,self.plan['move_s']))
            elif self.now()-self.phase_started>10:self.fail('visual_detection_timeout')
        elif self.phase=='approaching_target' and self.ready():
            self.move(self.plan['pick'],OPEN if self.disable_grip else self.plan['close'],'closing_gripper',1.5)
        elif self.phase=='closing_gripper' and self.ready():
            if self.bilateral():self.move(self.plan['lift'],self.plan['close'],'lifting',1.5)
            elif self.now()>self.motion_end+2:self.fail('no_bilateral_contact_at_grasp')
        elif self.phase=='lifting' and self.ready():
            if not self.bilateral() or self.object_position()[2]-BASE[2]<.065:self.fail('lift_not_confirmed');return
            self.lift_confirmed=True;self.transition('holding')
        elif self.phase=='holding':
            if not self.bilateral():self.fail('grasp_lost_during_hold');return
            if self.now()-self.phase_started>=1.:
                q=np.array([self.joints[j] for j in ARM_JOINTS]);p,r=fk(q)
                offset=r.T@(self.object_position()-BASE-p)
                desired=self.object_position()-BASE;desired[2]=max(.16-BASE[2],desired[2]+.025)
                clearance=solve(desired,2.2,q,offset)
                self.move(clearance,self.plan['close'],'clearing_box_rim',2.)
        elif self.phase=='clearing_box_rim' and self.ready():
            orientation=self.models[self.target].orientation
            lower,_=box_bounds(self.object_position(),[orientation.x,orientation.y,orientation.z,orientation.w])
            if not self.bilateral() or lower[2]<.11:self.fail('insufficient_transport_clearance');return
            q=self.goal.copy();box=self.object_position()-BASE
            delta=np.arctan2(self.tray[1]-BASE[1],self.tray[0]-BASE[0])-np.arctan2(box[1],box[0]);q[0]+=delta
            self.move(q,self.plan['close'],'transporting',4.)
        elif self.phase=='transporting':
            if self.object_position()[2]-BASE[2]<.055 or (self.now()-self.phase_started>.3 and not self.bilateral()):self.fail('object_dropped_in_transport');return
            if self.ready():
                self.carry_confirmed=True
                q=np.array([self.joints[j] for j in ARM_JOINTS]);p,r=fk(q)
                self.offset=r.T@(self.object_position()-BASE-p)
                desired=np.r_[self.tray,.105]-BASE
                self.place=solve(desired,2.8,q,self.offset)
                self.move(self.place,self.plan['close'],'lowering_into_box',2.)
        elif self.phase=='lowering_into_box' and self.ready():
            pos=self.object_position()
            if np.max(np.abs(pos[:2]-self.tray))>.045:self.fail('placement_alignment_error');return
            self.move(self.place,-.15,'releasing',2.)
        elif self.phase=='releasing' and self.ready():
            # Move fingers away before judging support and retention in the tray.
            q=np.array([self.joints[j] for j in ARM_JOINTS]);p,_=fk(q)
            bottom=gripper_bottom(q,self.joints[HAND_JOINT],self.fixed_vertices,self.hand_vertices)
            rise=max(.012,.108-bottom)
            retract=solve(p+[0,0,rise],q[1]-q[2]-q[3],q)
            self.move(retract,-.15,'retracting',2.)
        elif self.phase=='retracting' and self.ready():
            q=np.array([self.joints[j] for j in ARM_JOINTS])
            if gripper_bottom(q,OPEN,self.fixed_vertices,self.hand_vertices)<.1:self.fail('fingers_not_clear_of_tray');return
            self.move(self.goal,OPEN,'opening_after_retreat',1.5)
        elif self.phase=='opening_after_retreat' and self.ready():self.transition('verifying_delivery')
        elif self.phase=='verifying_delivery' and self.now()-self.phase_started>1.5:
            pos=self.object_position();twist=self.twists[self.target]
            speed=np.linalg.norm([twist.linear.x,twist.linear.y,twist.linear.z])
            angular_speed=np.linalg.norm([twist.angular.x,twist.angular.y,twist.angular.z])
            if angular_speed>.1:self.fail('placement_still_rotating');return
            red=self.models['distractor_red'].position
            red_delta=np.linalg.norm(np.array([red.x,red.y,red.z])-self.start_models['distractor_red'])
            orientation=self.models[self.target].orientation
            quat=[orientation.x,orientation.y,orientation.z,orientation.w]
            if not placement_confirmed(pos,quat,speed,self.bilateral(),self.now()-self.floor_contact_at<.2):self.fail('placement_not_settled_in_tray');return
            if red_delta>.005:self.fail('distractor_moved');return
            self.transition('done');self.finish()
    def finish(self):
        if self.finished:return
        self.finished=True
        final={name:[p.position.x,p.position.y,p.position.z] for name,p in self.models.items() if name.startswith(('target','distractor'))}
        poses={name:dict(position=[p.position.x,p.position.y,p.position.z],quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]) for name,p in self.models.items() if name.startswith(('target','distractor'))}
        final_twists={name:dict(linear=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z]) for name,t in self.twists.items() if name.startswith(('target','distractor'))}
        report=dict(final_state=self.phase,success=self.phase=='done',engine='Gazebo Classic ODE',grasp='contact friction; learned residual parameters; effort-controlled joints',parameters=self.parameters,scene=self.scene_config,events=self.events,samples=self.samples,initial_model_positions=self.start_models,final_model_positions=final,final_model_poses=poses,final_model_twists=final_twists,floor_contact_confirmed=self.now()-self.floor_contact_at<.2,lift_confirmed=self.lift_confirmed,transport_confirmed=self.carry_confirmed,command_peak_velocity_rad_s=self.command_peak_velocity,command_peak_acceleration_rad_s2=self.command_peak_acceleration,wall_seconds=time.monotonic()-self.wall_started,disable_grip=self.disable_grip)
        if self.frames:
            final_frame=self.frames[-1].copy()
            cv2.rectangle(final_frame,(0,0),(960,100),(25,25,25),-1)
            cv2.putText(final_frame,'LEARNED CONTACT GRASP | ROS 2 + GAZEBO DELIVERY',(16,32),0,.65,(255,255,255),2)
            cv2.putText(final_frame,f'{self.phase} | t={self.now():.1f}s | delivery verified={self.phase=="done"}',(16,65),0,.65,(100,255,100) if self.phase=='done' else (100,100,255),2)
            self.frames.append(final_frame);self.frame_times.append(self.now())
            report['video_sim_start']=self.frame_times[0];report['video_sim_end']=self.frame_times[-1];report['video_fps']=10
        self.output.write_text(json.dumps(report,indent=2))
        if self.frames:
            path=self.output.with_suffix('.mp4');writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),10,(960,720))
            if not writer.isOpened():raise RuntimeError('Cannot open delivery video encoder')
            for stamp in np.arange(self.frame_times[0],self.frame_times[-1]+.1,.1):
                index=min(len(self.frames)-1,max(0,int(np.searchsorted(self.frame_times,stamp,side='right')-1)))
                writer.write(self.frames[index])
            writer.release();cv2.imwrite(str(self.output.with_suffix('.png')),self.frames[-1])
            if shutil.which('ffmpeg'):
                encoded=path.with_name(path.stem+'_encoded.mp4')
                subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(path),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(encoded)],check=True)
                encoded.replace(path)
        self.get_logger().info('Evidence saved: '+str(self.output))

def main(args=None):
    rclpy.init(args=args);node=PhysicalDelivery()
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        if not node.finished:node.transition('interrupted');node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
