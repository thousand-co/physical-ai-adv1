"""Integration checks against the actual contact simulator and saved policy."""
import json
import unittest
import numpy as np
import mujoco
from experiment import Experiment, ROOT


class ContactGraspTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = np.array(json.loads((ROOT/'results/policy.json').read_text())['parameters'])

    def test_contact_lift_then_release_falls(self):
        env = Experiment()
        # A held-out placement with low friction and a 20 g object.
        result, _ = env.rollout(self.policy, 1000, friction=.7, mass=.02)
        self.assertTrue(result['success'], result)
        self.assertGreater(result['final_height_m'], .065)
        self.assertGreaterEqual(result['bilateral_contact_fraction'], .95)
        self.assertLessEqual(result['command_peak_velocity_rad_s'], 1.5+1e-8)
        self.assertLessEqual(result['command_peak_acceleration_rad_s2'], 4.+1e-8)
        self.assertEqual(env.model.neq, 0, 'No attachment constraints allowed')
        self.assertEqual(env.model.nmocap, 0)
        self.assertEqual(int(env.model.jnt_type[env.model.joint('box_free').id]), int(mujoco.mjtJoint.mjJNT_FREE))
        np.testing.assert_array_equal(env.data.xfrc_applied, 0)
        np.testing.assert_array_equal(env.data.qfrc_applied, 0)
        # Opening the gripper after successful retention must release the box.
        close = env.data.ctrl[4]
        for step in range(1250):
            t = min((step+1)/800,1.)
            blend = 10*t**3-15*t**4+6*t**5
            env.data.ctrl[4] = close + (-.65-close)*blend
            mujoco.mj_step(env.model,env.data)
        self.assertLess(env.data.xpos[env.box_body,2], .04)

    def test_open_gripper_does_not_lift(self):
        env = Experiment()
        result, _ = env.rollout(self.policy, 1000, friction=.7, mass=.02,disable_grip=True)
        self.assertFalse(result['success'])
        self.assertLess(result['final_height_m'], .04)

    def test_saved_evaluation_is_held_out_and_complete(self):
        report = json.loads((ROOT/'results/evaluation.json').read_text())
        cases = report['cases']
        self.assertEqual(len(cases),30)
        for case in cases:
            self.assertGreaterEqual(case['learned']['seed'],1000)
        self.assertEqual(report['summary']['open_gripper_control']['successes'],0)
        self.assertGreaterEqual(report['summary']['learned']['successes'],27)

if __name__=='__main__':
    unittest.main(verbosity=2)
