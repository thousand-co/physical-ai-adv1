"""Sequential image-based sorting with a scripted conveyor and physical grasp.

State setting is restricted to feeding the current parcel. After belt stop all
object motion is produced by gravity, finger contact and friction.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SetEntityState
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from .physical_delivery import PhysicalDelivery
from .physical_plan import BASE, OPEN, ARM_JOINTS, HAND_JOINT, grasp_plan, placement_confirmed, fk, solve, gripper_bottom
from .learned_color import LearnedColorClassifier, LABELS


class ConveyorSorting(PhysicalDelivery):
    def __init__(self):
        super().__init__()
        share = Path(get_package_share_directory('final_project_ros'))
        self.sequence = json.loads((share/'config/sorting_sequence.json').read_text())
        self.classifier = LearnedColorClassifier(share/'models/color_tree.yml')
        self.index = 0
        self.results = []
        self.target = self.sequence[0]['name']
        self.image = None
        self.image_time = -1e6
        self.votes = []
        self.feed_future = None
        self.feed_x = self.sequence[0]['start_x']
        self.last_feed = self.now()
        self.wall_started_episode = time.monotonic()
        self.parcel_started = time.monotonic()
        self.setter = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_subscription(Image, '/camera/color/image_raw', self.on_color_image, qos_profile_sensor_data)

    def on_color_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image_time = self.now()

    def on_detection(self, msg):
        pass  # Sorting uses all color classes from the stopped parcel's image ROI.

    def on_contacts(self, msg):
        for state in msg.states:
            names = state.collision1_name+' '+state.collision2_name
            if self.target not in names: continue
            if ('delivery_tray' in names or 'reject_tray' in names) and 'floor_collision' in names:
                self.floor_contact_at = self.now()
            if 'crane_plus_link_hand' in names: self.contact_times['hand'] = self.now()
            elif 'crane_plus_link4' in names: self.contact_times['fixed'] = self.now()

    def classify(self):
        if self.image is None or self.image_time <= getattr(self, 'last_vote_time', -1e6): return None
        self.last_vote_time = self.image_time
        if self.now()-self.image_time > .5: return None
        # Known inspection station projected into the overhead camera. Model color
        # and expected_color are never read by the classifier or routing logic.
        h, w = self.image.shape[:2]
        fx = w/(2*np.tan(1.3962634/2))
        u = round(w/2-fx*(-.025-.15)/(1.4-.071))
        v = round(h/2-fx*(.205-.55)/(1.4-.071))
        roi = self.image[v-4:v+5, u-4:u+5]
        if roi.shape[:2] != (9, 9): return None
        rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB).reshape(-1, 3).astype(np.float32)/255.
        labels = self.classifier.model.predict(rgb)[1].ravel().astype(int)
        counts = np.bincount(labels, minlength=len(LABELS))
        winner = int(counts.argmax())
        if counts[winner]/len(labels) < .75 or LABELS[winner] == 'background': return None
        return LABELS[winner]

    def feed(self):
        if self.feed_future is not None:
            if not self.feed_future.done(): return
            response = self.feed_future.result()
            self.feed_future = None
            if response is None or not response.success:
                self.fail('conveyor_state_service_failed'); return
            if self.feed_x <= .205+1e-6:
                self.transition('classifying'); self.votes = []; return
        if not self.setter.service_is_ready():
            if self.now()-self.phase_started > 10: self.fail('conveyor_service_timeout')
            return
        now = self.now()
        self.feed_x = max(.205, self.feed_x-.08*min(.05, max(0., now-self.last_feed)))
        self.last_feed = now
        request = SetEntityState.Request()
        request.state.name = self.target
        request.state.reference_frame = 'world'
        request.state.pose.position.x = self.feed_x
        request.state.pose.position.y = -.025
        request.state.pose.position.z = .071
        request.state.pose.orientation.w = 1.
        self.feed_future = self.setter.call_async(request)

    def step(self):
        if time.monotonic()-self.wall_started_episode > 600:
            self.fail('sorting_wall_timeout'); return
        if self.phase == 'initializing':
            if not all(j in self.joints for j in ARM_JOINTS+[HAND_JOINT]) or self.target not in self.models: return
            if not self.arm.get_subscription_count() or not self.hand.get_subscription_count(): return
            self.plan = grasp_plan(np.array([.205, -.025, .071])-BASE, self.parameters)
            self.move(self.plan['lift'], OPEN, 'returning_to_belt', 2.)
        elif self.phase == 'returning_to_belt' and self.ready():
            self.last_feed = self.now(); self.transition('feeding')
        elif self.phase == 'feeding': self.feed()
        elif self.phase == 'classifying':
            if self.now()-self.phase_started < .5: return
            color = self.classify()
            if color is not None: self.votes.append(color)
            if len(self.votes) >= 5 and len(set(self.votes[-5:])) == 1:
                self.predicted = color
                self.destination = 'delivery_tray' if color == 'blue' else 'reject_tray'
                center = np.array([.075, -.175 if color == 'blue' else .075])
                count = sum(r['destination'] == self.destination for r in self.results)
                self.tray = center + [(-.025 if count == 0 else .025), 0.]
                self.events.append(dict(state='color_decision', time=self.now(), parcel=self.target,
                                        predicted_color=color, destination=self.destination))
                self.get_logger().info(f'IMAGE COLOR: {color} -> {self.destination}')
                self.plan = grasp_plan(self.object_position()-BASE, self.parameters)
                self.move(self.plan['pick'], OPEN, 'approaching_target', max(1., self.plan['move_s']))
            elif self.now()-self.phase_started > 10: self.fail('color_not_confident')
        elif self.phase == 'releasing' and self.now() > self.motion_end+.25:
            # Contact can shift the arm while the fingers open. Once the parcel
            # is supported by the floor, retreat from the measured arm pose.
            if self.bilateral() or self.now()-self.floor_contact_at >= .2:
                if self.now() > self.motion_end+4: self.fail('release_not_supported')
                return
            if abs(self.joints[HAND_JOINT]+.15) > .05:
                if self.now() > self.motion_end+4: self.fail('release_hand_tracking_timeout')
                return
            q = np.array([self.joints[j] for j in ARM_JOINTS])
            p, _ = fk(q)
            bottom = gripper_bottom(q, self.joints[HAND_JOINT], self.fixed_vertices, self.hand_vertices)
            retract = solve(p+[0,0,max(.012,.108-bottom)], q[1]-q[2]-q[3], q)
            self.goal = q; self.hand_setpoint = self.joints[HAND_JOINT]
            self.move(retract, -.15, 'retracting', 2.)
        elif self.phase == 'verifying_delivery' and self.now()-self.phase_started > 1.5:
            p = self.object_position(); t = self.twists[self.target]; q = self.models[self.target].orientation
            # placement_confirmed uses the original tray center; translate reject
            # tray coordinates to that center while retaining physical contacts.
            translated = p.copy()
            if self.destination == 'reject_tray': translated[1] -= .25
            if not placement_confirmed(translated, [q.x,q.y,q.z,q.w],
                    np.linalg.norm([t.linear.x,t.linear.y,t.linear.z]), self.bilateral(),
                    self.now()-self.floor_contact_at < .2):
                self.fail('sorting_placement_not_confirmed'); return
            if np.linalg.norm([t.angular.x,t.angular.y,t.angular.z]) > .1:
                self.fail('sorting_placement_still_rotating'); return
            self.results.append(dict(parcel=self.target, predicted_color=self.predicted,
                expected_color=self.sequence[self.index]['expected_color'], destination=self.destination,
                position=p.tolist(), physical_placement_confirmed=True,
                quaternion_xyzw=[q.x,q.y,q.z,q.w],
                linear_speed=float(np.linalg.norm([t.linear.x,t.linear.y,t.linear.z])),
                angular_speed=float(np.linalg.norm([t.angular.x,t.angular.y,t.angular.z])),
                floor_contact_confirmed=True,
                lift_confirmed=self.lift_confirmed, transport_confirmed=self.carry_confirmed))
            self.index += 1
            if self.index == len(self.sequence):
                # A later placement must not push an earlier parcel out again.
                for result in self.results:
                    pose = self.models[result['parcel']]
                    twist = self.twists[result['parcel']]
                    position = np.array([pose.position.x,pose.position.y,pose.position.z])
                    result['final_position'] = position.tolist()
                    if result['destination'] == 'reject_tray': position[1] -= .25
                    quat = pose.orientation
                    result['retained_at_end'] = placement_confirmed(position,
                        [quat.x,quat.y,quat.z,quat.w],
                        np.linalg.norm([twist.linear.x,twist.linear.y,twist.linear.z]),
                        False, result['floor_contact_confirmed'])
                    if not result['retained_at_end']:
                        self.fail('previous_parcel_not_retained: '+result['parcel']); return
                self.transition('done'); self.finish(); return
            self.target = self.sequence[self.index]['name']
            self.feed_x = self.sequence[self.index]['start_x']
            self.contact_times = {}; self.floor_contact_at = -1e6
            self.lift_confirmed = self.carry_confirmed = False
            self.votes = []
            self.predicted = '?'; self.destination = 'inspection'
            self.parcel_started = time.monotonic()
            self.transition('initializing')
        else:
            super().step()

    def update(self):
        # Base watchdog is per parcel; the independent episode limit is above.
        self.wall_started = self.parcel_started
        if self.latest_frame is not None:
            frame = self.latest_frame.copy()
            cv2.putText(frame, f'PARCEL {self.index+1}/{len(self.sequence)} | '
                f'{getattr(self, "predicted", "?")} | {getattr(self, "destination", "inspection")}',
                (16, 130), 0, .65, (255, 255, 255), 2)
            self.latest_frame = frame
        super().update()

    def finish(self):
        if self.finished: return
        super().finish()
        report = json.loads(self.output.read_text())
        report.update(mode='conveyor_sorting', conveyor='scripted state updates during feeding only',
            parcels=self.results, expected_count=len(self.sequence),
            final_parcel_poses={name:dict(position=[p.position.x,p.position.y,p.position.z],
                quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
                for name,p in self.models.items() if name.startswith('parcel_')},
            wall_seconds=time.monotonic()-self.wall_started_episode)
        report['success'] = report['success'] and len(self.results) == len(self.sequence) and all(
            r['predicted_color'] == r['expected_color'] for r in self.results)
        self.output.write_text(json.dumps(report, indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorSorting()
    try:
        while rclpy.ok() and not node.finished: rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.transition('interrupted'); node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
