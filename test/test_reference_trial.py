"""No Gazebo required for parsing/preflight unit tests."""
import importlib.util
import pathlib
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'reference_stop_route_trial.py'
spec = importlib.util.spec_from_file_location('reference_stop_route_trial', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class TestReferenceTrial(unittest.TestCase):
    def test_world_selection(self):
        self.assertEqual(module.available_worlds('/world/diff_drive/state\n/world/diff_drive/state_async\n'), ['diff_drive'])
        self.assertEqual(module.available_worlds('/world/ares_world/state\n/world/diff_drive/state\n'), ['ares_world','diff_drive'])

    def test_parse_pose(self):
        data = '''Requesting state for world [diff_drive]...\nModel: [8]\n  - Name: vehicle_blue\n  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [2.432400 2.000000 0.325000]\n    [0.000000 0.000000 -0.000000]\n'''
        self.assertEqual(module.parse_pose(data), (2.4324, 2.0))

    def test_parse_pose_failure(self):
        with self.assertRaises(RuntimeError):
            module.parse_pose('No model named <vehicle_blue> was found')

if __name__ == '__main__':
    unittest.main()
