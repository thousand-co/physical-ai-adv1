import math
import unittest
from final_project_ros.motion_profile import plan_joint_motion


class MotionProfileTests(unittest.TestCase):
    def test_limits_end_conditions_and_monotonic_joint_motion(self):
        start, goal = [0.,-.1,-2.,.5], [.3,1.,-.8,-.2]
        duration, points = plan_joint_motion(start, goal, .6, 1.2)
        self.assertEqual(points[0][1], start)
        for actual, expected in zip(points[-1][1], goal):
            self.assertAlmostEqual(actual, expected)
        for index in (0,-1):
            self.assertTrue(all(abs(v)<1e-10 for v in points[index][2]))
            self.assertTrue(all(abs(a)<1e-10 for a in points[index][3]))
        for _, position, velocity, acceleration in points:
            self.assertLessEqual(max(map(abs,velocity)),.6+1e-9)
            self.assertLessEqual(max(map(abs,acceleration)),1.2+1e-9)
            for a,b,value in zip(start,goal,position):
                self.assertGreaterEqual(value,min(a,b)-1e-9)
                self.assertLessEqual(value,max(a,b)+1e-9)
        self.assertTrue(all(b[0]>a[0] for a,b in zip(points,points[1:])))
        self.assertAlmostEqual(points[-1][0],duration)

    def test_small_descent_takes_less_time_than_large_reposition(self):
        short,_=plan_joint_motion([0.]*4,[.03]*4)
        long,_=plan_joint_motion([0.]*4,[2.]*4)
        self.assertLess(short,long)

    def test_invalid_limits_are_rejected(self):
        for value in (0.,-1.,math.inf,math.nan):
            with self.assertRaises(ValueError):
                plan_joint_motion([0.],[1.],max_velocity=value)

    def test_stationary_goal_is_valid(self):
        _,points=plan_joint_motion([.2]*4,[.2]*4)
        self.assertTrue(all(p[1]==[.2]*4 for p in points))

if __name__=='__main__': unittest.main()
