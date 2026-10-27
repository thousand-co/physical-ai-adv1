import unittest
from pathlib import Path
import numpy as np
from final_project_ros.learned_color import LearnedColorClassifier, PALETTE
from final_project_ros.gazebo_delivery_demo import detect_targets
from final_project_ros.delivery_controller import DeliveryController, MOTION_FALLBACK_DELAY

ROOT = Path(__file__).resolve().parents[1]

class PerceptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.classifier = LearnedColorClassifier(ROOT / 'ros2_ws/src/final_project_ros/models/color_tree.yml')

    def test_two_blue_objects_and_red_distractor(self):
        image = np.full((180,240,3), 179, np.uint8)
        blue = np.array(PALETTE['blue'])[::-1] * 255
        red = np.array(PALETTE['red'])[::-1] * 255
        image[20:50,20:50] = blue
        image[90:120,150:180] = blue * .85
        image[20:60,120:160] = red
        detections = detect_targets(image, 'blue', 80, self.classifier)
        self.assertEqual(len(detections), 2)
        self.assertEqual([(d[0],d[1]) for d in detections], [(35,35),(165,105)])

    def test_empty_scene_does_not_trigger_a_pick(self):
        image = np.full((120,160,3),179,np.uint8)
        self.assertEqual(detect_targets(image,'blue',80,self.classifier), [])

    def test_tiny_noise_is_rejected(self):
        image = np.full((120,160,3),179,np.uint8)
        image[10:13,10:13] = np.array(PALETTE['blue'])[::-1] * 255
        self.assertEqual(detect_targets(image,'blue',80,self.classifier), [])

class MotionTests(unittest.TestCase):
    def test_unreached_arm_fails_instead_of_claiming_success(self):
        class Logger:
            def warning(self, *args): pass
        class Motion:
            arm_settle_count = 0
            phase_elapsed = MOTION_FALLBACK_DELAY
            arm_fallback_logged = False
            phase = 'approaching_target'
            failed = None
            def get_logger(self): return Logger()
            def fail(self, reason): self.failed = reason
        motion = Motion()
        self.assertFalse(DeliveryController.arm_at_goal(motion))
        self.assertEqual(motion.failed, 'arm_motion_timeout')

    def test_joint_tolerance_does_not_finish_a_running_trajectory(self):
        from types import SimpleNamespace as NS
        clock = NS(nanoseconds=9_000_000_000)
        motion = NS(latest_arm_state={}, arm_goal=[.1]*4,
                    arm_goal_tolerance=.1, arm_motion_end_ns=10_000_000_000,
                    arm_settle_count=0, gripper_goal=None,
                    get_clock=lambda: NS(now=lambda: clock))
        message = NS(name=['crane_plus_joint%d' % i for i in range(1,5)],
                     position=[.1]*4)
        for _ in range(5):
            DeliveryController.joint_state_callback(motion, message)
        self.assertEqual(motion.arm_settle_count, 0)
        clock.nanoseconds = 10_000_000_000
        for _ in range(3):
            DeliveryController.joint_state_callback(motion, message)
        self.assertTrue(DeliveryController.arm_at_goal(motion))

    def test_placement_requires_actual_model_pose(self):
        from types import SimpleNamespace as NS
        class Placement:
            target_name = 'target_blue_00'
            place_goal = (.1,-.19,.15)
            latest_models = {'target_blue_00': NS(position=NS(x=.25,y=0.,z=.12))}
        self.assertFalse(DeliveryController.target_in_box(Placement()))
        placement = Placement()
        placement.latest_models = {'target_blue_00': NS(position=NS(x=.1,y=-.19,z=.15))}
        self.assertTrue(DeliveryController.target_in_box(placement))

if __name__ == '__main__': unittest.main()
