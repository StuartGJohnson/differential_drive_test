import unittest

from scripts.robot_tools import find_robot, reset_robot

class MyTestCase(unittest.TestCase):
    def test_find(self):
        tty_string = find_robot()
        print(tty_string)
        self.assertIsNotNone(tty_string)  # add assertion here

    def test_reset(self):
        tty_string = find_robot()
        print(tty_string)
        reset_robot(tty_string)

if __name__ == '__main__':
    unittest.main()
