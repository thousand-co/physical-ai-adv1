"""Record actual Gazebo camera frames and telemetry, with evidence JSON."""
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.declare_parameter('output', 'artifacts/demo_raw.mp4')
        self.path = Path(self.get_parameter('output').value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280, 720))
        if not self.writer.isOpened():
            raise RuntimeError('Cannot open video output')
        self.frame = None
        self.state = 'starting'
        self.joints = []
        self.models = {}
        self.events = []
        self.sim_seconds = None
        self.previous_arm = None
        self.pregrasp_travel = [0.0] * 4
        self.first_grasp_complete = False
        self.first_joint_sim_seconds = None
        self.pregrasp_sim_seconds = None
        self.started = time.monotonic()
        self.done_at = None
        self.finished = False
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.create_subscription(String, '/task_state', self.on_state, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.create_timer(.1, self.write_frame)

    def on_image(self, msg):
        self.frame = CvBridge().imgmsg_to_cv2(msg, 'bgr8')

    def on_state(self, msg):
        self.state = msg.data
        self.events.append({'wall_seconds': time.monotonic()-self.started,
                            'sim_seconds': self.sim_seconds, 'state': self.state})
        if self.state == 'lifting' and not self.first_grasp_complete:
            self.first_grasp_complete = True
            if self.sim_seconds is not None and self.first_joint_sim_seconds is not None:
                self.pregrasp_sim_seconds = self.sim_seconds - self.first_joint_sim_seconds
        if self.state in ('done', 'error'):
            self.done_at = time.monotonic()

    def on_joints(self, msg):
        self.joints = list(zip(msg.name, msg.position))
        self.sim_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.first_joint_sim_seconds is None:
            self.first_joint_sim_seconds = self.sim_seconds
        positions = dict(self.joints)
        names = ['crane_plus_joint%d' % i for i in range(1, 5)]
        if all(name in positions for name in names):
            arm = [positions[name] for name in names]
            if self.previous_arm is not None and not self.first_grasp_complete:
                for i, (before, after) in enumerate(zip(self.previous_arm, arm)):
                    self.pregrasp_travel[i] += abs(after - before)
            self.previous_arm = arm

    def on_models(self, msg):
        self.models = {name: [p.position.x,p.position.y,p.position.z]
                       for name,p in zip(msg.name,msg.pose) if name.startswith(('target_', 'distractor'))}

    def write_frame(self):
        if self.finished:
            return
        elapsed = time.monotonic()-self.started
        canvas = np.full((720,1280,3), (28,23,18), np.uint8)
        cv2.putText(canvas,'AI COLOR SORTING | ROS 2 + Gazebo + CRANE+', (30,42),0,.9,(255,255,255),2)
        if self.frame is not None:
            # Fixed crop of the work area; actual camera pixels, no reenactment.
            crop = self.frame[245:450,270:460]
            canvas[80:680,30:630] = cv2.resize(crop,(600,600))
        lines = ['LIVE SIMULATION CAMERA', '', 'AI: supervised RGB decision tree',
                 'State: '+self.state, 'Wall time: %.1f s'%elapsed, '',
                 'Kinematic grasp (not contact physics)',
                 'Position: calibrated Gazebo model pose',
                 'Visual detection gates each pick', '', 'Measured joint positions (rad):']
        lines += [name.replace('crane_plus_', '')+': % .3f'%value for name,value in self.joints]
        for i,line in enumerate(lines):
            cv2.putText(canvas,line,(660,100+i*29),0,.51,(225,230,220),1,cv2.LINE_AA)
        self.writer.write(canvas)
        if self.done_at and time.monotonic()-self.done_at>4:
            self.finish()
        elif elapsed>170:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.writer.release()
        self.path.with_suffix('.json').write_text(json.dumps({'events':self.events,
            'final_state':self.state,'final_model_positions':self.models,
            'wall_seconds':time.monotonic()-self.started,
            'pregrasp_sim_seconds': self.pregrasp_sim_seconds,
            'pregrasp_joint_travel_rad': self.pregrasp_travel,
            'pregrasp_total_joint_travel_rad': sum(self.pregrasp_travel)},indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
