import unittest

import robot_control

class TestQuaternionMethods(unittest.TestCase):

    def test_rad_to_deg(self):
        self.assertEqual(robot_control.rad_to_deg(robot_control.math.pi), 180.0)

    def test_quat_to_euler(self):
        quaternion = [robot_control.math.cos(robot_control.math.pi/4.0), 0.0, 0.0, -robot_control.math.sin(robot_control.math.pi/4.0)]
        euler = robot_control.quat_to_euler(quaternion)
        euler_round = list(map(round, euler))
        self.assertListEqual(euler_round, [-90, 0, 0])

    def test_normalize_quat(self):
        quaternion = [0.0, 0.0, 1.7071, -0.70711]
        normalize_quaternion = [0.0, 0.0, 0.92388, -0.38268]
        normalize_quaternion = list(map(lambda x: round(x,4), normalize_quaternion))
        quaternion = robot_control.normalize_quat(quaternion)
        quaternion = list(map(lambda x: round(x, 4), quaternion))
        self.assertListEqual(quaternion, normalize_quaternion)

    def test_inv_quat(self):
        quaternion = [ robot_control.math.cos(robot_control.math.pi/4.0), 0.0, 0.0, -robot_control.math.sin(robot_control.math.pi/4.0)]
        expectation = [ -robot_control.math.cos(robot_control.math.pi/4.0), 0.0, 0.0, -robot_control.math.sin(robot_control.math.pi/4.0)]
        self.assertListEqual( robot_control.inv_quat(quaternion), expectation)

    def test_multiplication_quat(self):
        quaternion1 = [0.0, 0.0, -0.70711, 0.70711]
        quaternion2 = [0, 0, -0.38268, 0.92388]
        expectation = [0, 0, -0.92388, 0.38268]
        expectation = list(map(lambda x: round(x, 4), expectation))
        result = robot_control.multiplication_quat(quaternion1, quaternion2)
        result = list(map(lambda x: round(x, 4), result))
        self.assertListEqual(result, expectation)

    def test_get_changed_orientation(self):
        expectation = [0.0, 0.0, -0.70711, 0.70711]
        expectation = list(map(lambda x: round(x, 4), expectation))
        quaternion1 = [0, 0, -0.38268, 0.92388]
        quaternion2 = [0, 0, -0.92388, 0.38268]
        result = robot_control.get_changed_orientation(quaternion2, quaternion1)
        result = list(map(lambda x: round(x, 4), result))
        self.assertListEqual(result, expectation)


if __name__ == '__main__':
    unittest.main()
