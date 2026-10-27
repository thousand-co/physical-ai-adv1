#!/usr/bin/env python3

import math

from typing import Dict, Optional, Sequence, Tuple

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PointStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    'crane_plus_joint1',
    'crane_plus_joint2',
    'crane_plus_joint3',
    'crane_plus_joint4',
]
GRIPPER_JOINT = 'crane_plus_joint_hand'
HOME_POSITION = [0.0, -1.16, -2.01, -0.73]
DEFAULT_IK_SEED = [0.322, 0.98, -0.89, 0.30]
PICK_ORIENTATION = (-0.1133, 0.6985, 0.1133, 0.6985)
PLACE_ORIENTATION = (-0.5, 0.5, 0.5, 0.5)
TARGET_COLORS = {
    'blue', 'orange', 'green', 'red', 'purple',
    'brown', 'pink', 'gray', 'olive', 'cyan',
}
JOINT_TOLERANCE = 0.15
JOINT_SETTLE_SAMPLES = 3
MOTION_FALLBACK_DELAY = 12.0
SERVICE_TIMEOUT = 8.0


class DeliveryController(Node):
    def __init__(self):
        super().__init__('delivery_controller')
        self.declare_parameter('target_color', 'blue')
        self.declare_parameter('base_x', 0.10)
        self.declare_parameter('base_y', -0.05)
        self.declare_parameter('base_z', 0.05)
        self.declare_parameter('box_x', 0.10)
        self.declare_parameter('box_y', -0.19)
        self.declare_parameter('box_drop_z', 0.15)
        self.declare_parameter('approach_height', 0.03)
        self.declare_parameter('gripper_open_position', 0.50)
        self.declare_parameter('gripper_closed_position', -0.5236)
        self.declare_parameter('box_capacity', 20)

        self.target_color = str(self.get_parameter('target_color').value).lower()
        if self.target_color not in TARGET_COLORS:
            raise ValueError(
                f'target_color must be one of: {", ".join(sorted(TARGET_COLORS))}'
            )
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.base_z = float(self.get_parameter('base_z').value)
        self.box_x = float(self.get_parameter('box_x').value)
        self.box_y = float(self.get_parameter('box_y').value)
        self.box_drop_z = float(self.get_parameter('box_drop_z').value)
        self.approach_height = float(self.get_parameter('approach_height').value)
        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        self.box_capacity = int(self.get_parameter('box_capacity').value)
        if not 1 <= self.box_capacity <= 20:
            raise ValueError('box_capacity must be between 1 and 20')

        self.state_pub = self.create_publisher(String, '/task_state', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10
        )
        self.create_subscription(PointStamped, '/target_point', self.target_callback, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.set_entity_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_k = None
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self.camera_info_callback, 10)
        self.phase = 'initializing'
        self.phase_elapsed = 0.0
        self.last_published_state = ''
        self.latest_models: Dict[str, object] = {}
        self.delivered_models = set()
        self.target_name = ''
        self.target_count = 0
        self.available_target_count = 0
        self.delivered_count = 0
        self.place_goal: Optional[Tuple[float, float, float]] = None
        self.box_slots = self.make_box_slots()
        self.latest_arm_state: Dict[str, float] = {}
        self.latest_gripper_state: Optional[float] = None
        self.target_detected = False
        self.target_pose: Optional[Tuple[float, float, float]] = None
        self.arm_goal: Optional[Sequence[float]] = None
        self.arm_settle_count = 0
        self.arm_fallback_logged = False
        self.gripper_goal: Optional[float] = None
        self.gripper_settle_count = 0
        self.gripper_fallback_logged = False
        self.ik_pending = False
        self.ik_future = None
        self.ik_next_phase = ''
        self.ik_gripper_goal = self.gripper_open
        self.entity_update_pending = False
        self.attached = False
        self.carry_offset_world: Optional[Tuple[float, float, float]] = None
        self.pick_tool_goal: Optional[Tuple[float, float, float]] = None
        self.attachment_update_elapsed = 0.0
        self.service_warning_elapsed = 0.0
        self.timer = self.create_timer(0.1, self.update_delivery)
        self.publish_state('initializing')

    def publish_state(self, state: str):
        if state == self.last_published_state:
            return
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.last_published_state = state
        self.get_logger().info(f'Task state: {state}')

    def model_states_callback(self, msg: ModelStates):
        self.latest_models = dict(zip(msg.name, msg.pose))

    def joint_state_callback(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        self.latest_arm_state.update(
            (name, float(positions[name]))
            for name in ARM_JOINTS
            if name in positions
        )
        if self.arm_goal is not None and all(name in self.latest_arm_state for name in ARM_JOINTS):
            if max(
                abs(self.latest_arm_state[name] - goal)
                for name, goal in zip(ARM_JOINTS, self.arm_goal)
            ) <= JOINT_TOLERANCE:
                self.arm_settle_count = min(
                    self.arm_settle_count + 1, JOINT_SETTLE_SAMPLES
                )
            else:
                self.arm_settle_count = 0

        if GRIPPER_JOINT in positions:
            self.latest_gripper_state = float(positions[GRIPPER_JOINT])
            if self.gripper_goal is not None:
                if abs(self.latest_gripper_state - self.gripper_goal) <= JOINT_TOLERANCE:
                    self.gripper_settle_count = min(
                        self.gripper_settle_count + 1, JOINT_SETTLE_SAMPLES
                    )
                else:
                    self.gripper_settle_count = 0

    def camera_info_callback(self, msg):
        self.camera_k = msg.k

    def target_callback(self, msg: PointStamped):
        if self.phase != 'waiting_for_object' or self.target_name:
            return
        candidates = self.remaining_target_names()
        if not candidates:
            return
        if self.camera_k is None:
            return
        # Calibrated overhead camera from the world file. Match a visual
        # detection to a model spatially; model names alone never trigger picks.
        fx, fy, cx, cy = (self.camera_k[i] for i in (0, 4, 2, 5))
        def pixel_distance(name):
            p = self.latest_models[name].position
            depth = 1.4 - p.z
            if depth <= 0:
                return float('inf')
            u = cx - fx * (p.y - .15) / depth
            v = cy - fy * (p.x - .55) / depth
            return math.hypot(u - msg.point.x, v - msg.point.y)
        candidate = min(candidates, key=pixel_distance)
        if pixel_distance(candidate) > 18.0:
            return
        self.target_name = candidate
        target = self.latest_models.get(self.target_name)
        if target is None:
            self.get_logger().warning('Color detected, but target model pose is not available yet')
            self.target_name = ''
            return
        self.target_pose = (
            float(target.position.x),
            float(target.position.y),
            float(target.position.z),
        )
        self.target_detected = True

    def update_delivery(self):
        self.phase_elapsed += 0.1
        self.service_warning_elapsed += 0.1

        if self.ik_pending and self.ik_future is not None and self.ik_future.done():
            future = self.ik_future
            next_phase = self.ik_next_phase
            gripper = self.ik_gripper_goal
            self.ik_pending = False
            self.ik_future = None
            self.ik_result(future, next_phase, gripper)
            if self.phase == 'error':
                return
        if self.ik_pending and self.phase_elapsed >= SERVICE_TIMEOUT:
            self.ik_pending = False
            self.ik_future = None
            self.fail('ik_response_timeout')
            return
        if self.entity_update_pending and self.phase_elapsed >= SERVICE_TIMEOUT:
            self.fail('gazebo_state_service_timeout')
            return

        if self.attached and self.phase in (
            'lifting',
            'planning_place',
            'placing_in_box',
        ):
            self.attachment_update_elapsed += 0.1
            if self.attachment_update_elapsed >= 0.2:
                self.attachment_update_elapsed = 0.0
                self.update_attached_target()

        if self.phase == 'initializing':
            target_names = self.remaining_target_names()
            ready = (
                self.ik_client.service_is_ready()
                and self.set_entity_client.service_is_ready()
                and self.arm_pub.get_subscription_count() > 0
                and self.gripper_pub.get_subscription_count() > 0
                and all(name in self.latest_arm_state for name in ARM_JOINTS)
            )
            if not ready:
                if self.service_warning_elapsed >= 5.0:
                    self.get_logger().info(
                        'Waiting for MoveIt IK, Gazebo, CRANE+ controllers and model states'
                    )
                    self.service_warning_elapsed = 0.0
                return
            if not target_names:
                if any(name.startswith('target_') for name in self.latest_models):
                    self.phase = 'done'
                    self.publish_state('done')
                    self.get_logger().warning(
                        f'No targets found for selected color {self.target_color}'
                    )
                return
            self.available_target_count = len(target_names)
            self.target_count = min(self.available_target_count, self.box_capacity)
            if self.target_count < self.available_target_count:
                self.get_logger().warning(
                    f'Box capacity is {self.box_capacity}; '
                    f'{self.available_target_count - self.target_count} target(s) will remain unsorted'
                )
            self.start_homing()
            return

        if self.phase == 'homing':
            if self.arm_at_goal():
                self.arm_goal = None
                self.phase = 'waiting_for_object'
                self.phase_elapsed = 0.0
                self.publish_state('waiting_for_object')
            return

        if self.phase == 'waiting_for_object':
            if self.target_detected and self.target_pose is not None:
                self.target_detected = False
                self.phase = 'planning_approach'
                self.phase_elapsed = 0.0
                x, y, z = self.target_pose
                self.publish_state('target_detected')
                self.request_ik(
                    x,
                    y,
                    z + self.approach_height,
                    'approaching_target',
                    self.gripper_open,
                    PICK_ORIENTATION,
                )
            return

        if self.phase == 'approaching_target':
            if self.arm_at_goal():
                self.arm_goal = None
                if self.target_pose is None:
                    self.fail('target_pose_lost_before_grasp')
                    return
                x, y, z = self.target_pose
                self.phase = 'planning_grasp'
                self.phase_elapsed = 0.0
                self.request_ik(
                    x,
                    y,
                    z,
                    'grasping',
                    self.gripper_open,
                    PICK_ORIENTATION,
                )
            return

        if self.phase == 'grasping':
            if self.arm_at_goal():
                self.arm_goal = None
                self.command_gripper(self.gripper_closed, 1.0)
                self.phase = 'closing_gripper'
                self.phase_elapsed = 0.0
                self.publish_state('closing_gripper')
            return

        if self.phase == 'closing_gripper':
            if self.gripper_at_goal():
                if not self.capture_target():
                    return
                self.gripper_goal = None
                if self.target_pose is None:
                    self.fail('target_pose_lost_after_grasp')
                    return
                x, y, z = self.target_pose
                self.request_ik(
                    x,
                    y,
                    z + self.approach_height,
                    'lifting',
                    self.gripper_closed,
                    PICK_ORIENTATION,
                )
            elif self.phase_elapsed >= MOTION_FALLBACK_DELAY:
                self.fail('gripper_close_timeout')
            return

        if self.phase == 'lifting':
            if self.arm_at_goal():
                self.arm_goal = None
                if self.carry_offset_world is None:
                    self.fail('target_was_not_attached')
                    return
                if self.delivered_count >= len(self.box_slots):
                    self.fail('box_capacity_exceeded')
                    return
                slot_x, slot_y = self.box_slots[self.delivered_count]
                self.place_goal = (
                    self.box_x + slot_x,
                    self.box_y + slot_y,
                    self.box_drop_z,
                )
                self.phase = 'planning_place'
                self.phase_elapsed = 0.0
                self.request_ik(
                    self.place_goal[0] - self.carry_offset_world[0],
                    self.place_goal[1] - self.carry_offset_world[1],
                    self.place_goal[2] - self.carry_offset_world[2],
                    'placing_in_box',
                    self.gripper_closed,
                    PLACE_ORIENTATION,
                )
            return

        if self.phase == 'placing_in_box':
            if self.arm_at_goal():
                self.arm_goal = None
                self.phase = 'finishing_place'
                self.phase_elapsed = 0.0
            return

        if self.phase == 'finishing_place':
            if self.place_goal is None:
                self.fail('placement_slot_missing')
                return
            if not self.entity_update_pending:
                self.set_model_pose(
                    self.target_name,
                    *self.place_goal,
                    on_success=self.release_target,
                )
            return

        if self.phase == 'releasing':
            if self.phase_elapsed >= 1.5:
                if self.target_in_box():
                    self.delivered_models.add(self.target_name)
                    self.delivered_count += 1
                    self.get_logger().info(
                        f'Delivered {self.delivered_count}/{self.target_count} '
                        f'{self.target_color} target(s)'
                    )
                    self.target_name = ''
                    self.target_pose = None
                    self.place_goal = None
                    if self.delivered_count >= self.target_count:
                        self.phase = 'done'
                        self.phase_elapsed = 0.0
                        self.publish_state('done')
                        self.get_logger().info(
                            f'Delivery complete: all {self.target_color} targets are inside the goal box'
                        )
                    else:
                        self.start_homing()
                else:
                    self.fail('placement_not_confirmed')

    def release_target(self):
        if self.phase == 'error':
            return
        self.command_gripper(self.gripper_open, 1.0)
        self.attached = False
        self.phase = 'releasing'
        self.phase_elapsed = 0.0
        self.publish_state('releasing')

    def start_homing(self):
        if self.phase == 'error':
            return
        self.command_arm(HOME_POSITION, self.gripper_open, 3.0)
        self.phase = 'homing'
        self.phase_elapsed = 0.0
        self.publish_state('homing')

    def arm_at_goal(self) -> bool:
        if self.arm_settle_count >= JOINT_SETTLE_SAMPLES:
            return True
        if self.phase_elapsed >= MOTION_FALLBACK_DELAY:
            if not self.arm_fallback_logged:
                self.get_logger().warning(
                    f'Arm did not settle within tolerance during {self.phase}; '
                    'stopping delivery'
                )
                self.arm_fallback_logged = True
            self.fail('arm_motion_timeout')
        return False

    def gripper_at_goal(self) -> bool:
        if self.gripper_settle_count >= JOINT_SETTLE_SAMPLES:
            return True
        if self.phase_elapsed >= MOTION_FALLBACK_DELAY:
            if not self.gripper_fallback_logged:
                self.get_logger().warning(
                    'Gripper did not settle within tolerance; stopping delivery'
                )
                self.gripper_fallback_logged = True
            self.fail('gripper_motion_timeout')
        return False

    def capture_target(self) -> bool:
        if self.target_pose is None or self.pick_tool_goal is None:
            self.fail('target_pose_missing_at_grasp')
            return False
        self.carry_offset_world = tuple(
            target - tool
            for target, tool in zip(self.target_pose, self.pick_tool_goal)
        )
        self.attached = True
        self.attachment_update_elapsed = 0.0
        self.get_logger().info(
            'Kinematic grasp enabled for the Gazebo demo; following the endtip until release'
        )
        return True

    def update_attached_target(self):
        if self.carry_offset_world is None or self.entity_update_pending:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'crane_plus_link_endtip',
                Time(),
            )
        except TransformException as exc:
            self.fail(f'endtip_transform_lost_while_carrying: {exc}')
            return
        position = (
            self.base_x + transform.transform.translation.x + self.carry_offset_world[0],
            self.base_y + transform.transform.translation.y + self.carry_offset_world[1],
            self.base_z + transform.transform.translation.z + self.carry_offset_world[2],
        )
        self.set_model_pose(self.target_name, *position)

    def set_model_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        on_success=None,
    ):
        if self.entity_update_pending:
            return
        request = SetEntityState.Request()
        request.state.name = name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.w = 1.0
        future = self.set_entity_client.call_async(request)
        self.entity_update_pending = True
        future.add_done_callback(
            lambda result: self.entity_state_result(result, name, on_success)
        )

    def entity_state_result(self, future, name: str, on_success=None):
        self.entity_update_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.fail(f'gazebo_pose_update_failed_for_{name}: {exc}')
            return
        if not response.success:
            self.fail(f'gazebo_rejected_pose_update_for_{name}')
            return
        if on_success is not None:
            on_success()

    def request_ik(
        self,
        world_x: float,
        world_y: float,
        world_z: float,
        next_phase: str,
        gripper: float,
        orientation: Sequence[float],
    ):
        if self.ik_pending:
            return
        if not self.ik_client.service_is_ready():
            self.fail('ik_service_unavailable')
            return

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = 'arm'
        ik.ik_link_name = 'crane_plus_link_endtip'
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.pose.position.x = world_x - self.base_x
        ik.pose_stamped.pose.position.y = world_y - self.base_y
        ik.pose_stamped.pose.position.z = world_z - self.base_z
        # Four DOF arm: use the target's radial yaw, rather than an
        # incompatible fixed orientation for all world positions.
        yaw = math.atan2(world_y - self.base_y, world_x - self.base_x)
        half = math.sqrt(0.5)
        orientation = (-math.sin(yaw / 2) * half, math.cos(yaw / 2) * half,
                       math.sin(yaw / 2) * half, math.cos(yaw / 2) * half)
        (
            ik.pose_stamped.pose.orientation.x,
            ik.pose_stamped.pose.orientation.y,
            ik.pose_stamped.pose.orientation.z,
            ik.pose_stamped.pose.orientation.w,
        ) = orientation
        ik.timeout.sec = 1
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = ARM_JOINTS
        ik.robot_state.joint_state.position = [
            self.latest_arm_state.get(name, default)
            for name, default in zip(ARM_JOINTS, DEFAULT_IK_SEED)
        ]
        if next_phase == 'grasping':
            self.pick_tool_goal = (world_x, world_y, world_z)

        self.ik_pending = True
        self.ik_next_phase = next_phase
        self.ik_gripper_goal = gripper
        self.phase_elapsed = 0.0
        self.ik_future = self.ik_client.call_async(request)

    def ik_result(self, future, next_phase: str, gripper: float):
        self.ik_pending = False
        if self.phase == 'error':
            return
        try:
            response = future.result()
        except Exception as exc:
            self.fail(f'ik_request_failed: {exc}')
            return
        if response.error_code.val != 1:
            self.fail(f'no_ik_solution_for_{next_phase}')
            return

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if any(name not in solution for name in ARM_JOINTS):
            self.fail('ik_solution_missing_arm_joint')
            return
        positions = [float(solution[name]) for name in ARM_JOINTS]
        self.command_arm(positions, gripper, 3.0)
        self.phase = next_phase
        self.phase_elapsed = 0.0
        self.publish_state(next_phase)

    def command_arm(self, positions: Sequence[float], gripper: float, duration: float):
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1_000_000_000)
        trajectory.points.append(point)
        self.arm_pub.publish(trajectory)
        self.arm_goal = tuple(float(value) for value in positions)
        self.arm_settle_count = 0
        self.arm_fallback_logged = False
        self.command_gripper(gripper, duration)

    def command_gripper(self, position: float, duration: float):
        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1_000_000_000)
        trajectory.points.append(point)
        self.gripper_pub.publish(trajectory)
        self.gripper_goal = float(position)
        self.gripper_settle_count = 0
        self.gripper_fallback_logged = False

    def target_in_box(self) -> bool:
        if not self.target_name or self.place_goal is None:
            self.get_logger().error(
                'Cannot confirm placement: target or destination slot is unset'
            )
            return False
        target = self.latest_models.get(self.target_name)
        if target is None:
            self.get_logger().error(
                'Cannot confirm placement: target model is missing from Gazebo'
            )
            return False
        pose = target.position
        return (
            abs(pose.x - self.place_goal[0]) <= 0.02
            and abs(pose.y - self.place_goal[1]) <= 0.02
            and abs(pose.z - self.place_goal[2]) <= 0.02
        )

    def remaining_target_names(self) -> list[str]:
        prefix = f'target_{self.target_color}_'
        return sorted(
            name for name in self.latest_models
            if name.startswith(prefix) and name not in self.delivered_models
        )

    @staticmethod
    def make_box_slots() -> list[Tuple[float, float]]:
        offsets = [
            (0.00, 0.00), (-0.04, 0.00), (0.04, 0.00),
            (0.00, -0.04), (0.00, 0.04), (-0.04, -0.04),
            (-0.04, 0.04), (0.04, -0.04), (0.04, 0.04),
            (-0.08, 0.00), (0.08, 0.00), (0.00, -0.08),
            (0.00, 0.08), (-0.08, -0.04), (-0.08, 0.04),
            (0.08, -0.04), (0.08, 0.04), (-0.04, -0.08),
            (0.04, -0.08), (-0.04, 0.08),
        ]
        return offsets

    def fail(self, reason: str):
        if self.phase == 'error':
            return
        self.phase = 'error'
        self.phase_elapsed = 0.0
        self.attached = False
        self.publish_state('error')
        self.get_logger().error(f'Delivery stopped: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
