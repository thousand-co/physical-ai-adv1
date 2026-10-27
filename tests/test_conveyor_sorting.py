"""Exercise image decisions and fail-closed evidence checks."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

from final_project_ros.conveyor_sorting import ConveyorSorting
from final_project_ros.learned_color import LearnedColorClassifier, PALETTE
from final_project_ros.physical_plan import ARM_JOINTS, HAND_JOINT, fk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from check_physical_delivery import check


class SortingTests(unittest.TestCase):
    def test_retreat_starts_at_measured_pose_after_supported_release(self):
        node = ConveyorSorting.__new__(ConveyorSorting)
        node.wall_started_episode = time.monotonic()
        node.phase = 'releasing'; node.motion_end = 9.; node.now = lambda: 10.
        node.bilateral = lambda: False
        node.floor_contact_at = 9.95
        actual = np.array([1.852, -.2464, -1.6329, -1.6363])
        node.joints = dict(zip(ARM_JOINTS, actual)); node.joints[HAND_JOINT] = -.15
        vertices = np.load(ROOT/'ros2_ws/src/final_project_ros/config/gripper_collision_vertices.npz')
        node.fixed_vertices = vertices['fixed']; node.hand_vertices = vertices['hand']
        commands = []
        node.move = lambda *args: commands.append(args)
        node.step()
        np.testing.assert_allclose(node.goal, actual)
        self.assertEqual(commands[0][2], 'retracting')
        before, _ = fk(actual); after, _ = fk(commands[0][0])
        self.assertGreater(after[2], before[2]+.01)
        np.testing.assert_allclose(after[:2], before[:2], atol=.004)
        commands.clear(); node.floor_contact_at = 0.
        node.step()
        self.assertFalse(commands, 'Do not retreat before floor support is observed')

    def test_stopped_parcel_roi_classifies_each_color_without_model_metadata(self):
        node = ConveyorSorting.__new__(ConveyorSorting)
        node.classifier = LearnedColorClassifier(ROOT/'ros2_ws/src/final_project_ros/models/color_tree.yml')
        node.now = lambda: 1.
        for color in ['blue', 'red', 'green']:
            node.image = np.zeros((480, 640, 3), dtype=np.uint8)
            # Only the station is colored; other image pixels contain background.
            fx = 640/(2*np.tan(1.3962634/2))
            u = round(320-fx*(-.025-.15)/(1.4-.071))
            v = round(240-fx*(.205-.55)/(1.4-.071))
            node.image[v-5:v+6,u-5:u+6] = np.array(PALETTE[color][::-1])*255
            node.image_time = 1.; node.last_vote_time = 0.
            self.assertEqual(node.classify(), color)
            self.assertIsNone(node.classify(), 'One camera frame must not count twice')
        node.image_time = 0.; node.last_vote_time = -1.
        self.assertIsNone(node.classify(), 'Stale images must not trigger a grasp')

    def test_success_flag_cannot_hide_wrong_routing_or_missing_parcels(self):
        report = dict(final_state='done', success=True, lift_confirmed=True,
            transport_confirmed=True, floor_contact_confirmed=True, mode='conveyor_sorting',
            expected_count=1, parcels=[dict(expected_color='red', predicted_color='red',
            destination='delivery_tray', physical_placement_confirmed=True,
            lift_confirmed=True, transport_confirmed=True,
            floor_contact_confirmed=True, retained_at_end=True)])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'report.json'
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'Sorting parcel'): check(path)
            report['parcels'] = []
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'Incomplete'): check(path)


if __name__ == '__main__': unittest.main()
