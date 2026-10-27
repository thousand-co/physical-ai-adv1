"""Physical delivery regression checks, including the observed grip-loss bug."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from final_project_ros.physical_plan import TRAY,placement_confirmed,grasp_plan,BASE
from final_project_ros.physical_delivery import PhysicalDelivery

class PhysicalDeliveryTests(unittest.TestCase):
    def test_entire_object_must_be_inside_supported_and_released(self):
        center=np.r_[TRAY,.08];quat=[0,0,0,1]
        self.assertTrue(placement_confirmed(center,quat,0.,False,True))
        self.assertFalse(placement_confirmed(center+[.06,0,0],quat,0.,False,True))
        self.assertFalse(placement_confirmed(center,quat,.1,False,True))
        self.assertFalse(placement_confirmed(center,quat,0.,True,True))
        self.assertFalse(placement_confirmed(center,quat,0.,False,False))
        self.assertFalse(placement_confirmed(center+[0,0,.045],quat,0.,False,True))

    def test_grip_pressure_survives_arm_segment_transition(self):
        sent=[]
        pub=SimpleNamespace(publish=lambda message:sent.append(message))
        node=SimpleNamespace(joints={'crane_plus_joint'+str(i):0. for i in range(1,5)},hand_setpoint=.42,goal=np.array([0.,0.,-.9,-.9]),arm=pub,hand=pub,command_peak_velocity=0.,command_peak_acceleration=0.,now=lambda:0.,transition=lambda phase:None)
        node.joints['crane_plus_joint_hand']=.23 # physically blocked by the box
        PhysicalDelivery.move(node,np.array([-.8,.1,-1.,-1.]),.42,'transporting',2.5)
        self.assertEqual(list(sent[0].points[0].positions),[0.,0.,-.9,-.9])
        hand=sent[1]
        self.assertEqual(list(hand.points[0].positions),[.42])
        self.assertEqual(list(hand.points[-1].positions),[.42])
        self.assertLessEqual(node.command_peak_velocity,1.0+1e-6)
        self.assertLessEqual(node.command_peak_acceleration,2.0+1e-6)

    def test_saved_grasp_policy_has_reachable_gazebo_waypoints(self):
        root=Path(__file__).resolve().parents[1]
        policy=json.loads((root/'physics_grasp/results/policy.json').read_text())['parameters']
        plan=grasp_plan(np.array([.205,-.025,.0698])-BASE,policy)
        self.assertEqual(len(plan['pick']),4)
        self.assertEqual(len(plan['lift']),4)
        self.assertGreater(plan['close'],0.)

if __name__=='__main__':unittest.main()
