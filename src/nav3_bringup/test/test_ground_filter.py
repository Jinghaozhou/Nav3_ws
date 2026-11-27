"""Run with the Nav3 environment: python3 -m unittest discover -s src/nav3_bringup/test."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
path = Path(__file__).resolve().parents[1] / 'scripts/collision_ground_filter.py'
spec = importlib.util.spec_from_file_location('ground_filter', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

class GroundContract(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(3)
        self.xy = rng.uniform(-3, 3, (1200, 2))
        self.floor = np.c_[self.xy, -.44+.005*self.xy[:, 0]+rng.normal(0, .002, len(self.xy))]

    def test_flat_support_removed_and_low_obstacle_preserved(self):
        obstacle = np.array([[.8, 1., -.39], [.8, 1., -.34], [.8, 1., .2]])
        cloud = np.r_[self.floor, obstacle]
        before = cloud.copy()
        plane = module.fit_floor(cloud, np.zeros(3))
        np.testing.assert_allclose(plane, [.005, 0, -.44], atol=.001)
        keep = module.obstacle_mask(cloud, plane)
        self.assertLess(keep[:1200].sum(), 3)
        self.assertTrue(keep[-3:].all(), '5 cm and taller obstacles must remain')
        np.testing.assert_array_equal(cloud, before)
        # Body gait height changes must not move the frozen support plane.
        plane2 = module.fit_floor(cloud, np.array([0, 0, -.03]))
        np.testing.assert_allclose(plane2, plane, atol=.001)

    def test_missing_or_tilted_floor_is_rejected(self):
        with self.assertRaises(ValueError): module.fit_floor(self.floor[:20], np.zeros(3))
        tilted = self.floor.copy(); tilted[:, 2] += .25*tilted[:, 0]
        with self.assertRaises(ValueError): module.fit_floor(tilted, np.zeros(3))

    def test_height_prior_keeps_imu_reference_after_mount_import(self):
        # The existing 44 cm prior describes the MID360 IMU, not Go2's base.
        # Moving the robot reference downward must not move the measured floor.
        imu_position = np.zeros(3)
        go2_position = np.array([-.15, -.02329, -.17])
        with self.assertRaises(ValueError): module.fit_floor(self.floor, go2_position)
        np.testing.assert_allclose(module.fit_floor(self.floor, imu_position), [.005, 0., -.44], atol=.001)

    def test_small_patch_is_not_a_floor(self):
        patch = self.floor.copy(); patch[:, :2] *= .1
        with self.assertRaises(ValueError): module.fit_floor(patch, np.zeros(3))

    def test_local_fit_keeps_distant_obstacles_in_output(self):
        distant_obstacle = np.array([[3.5, 0., -.39], [3.6, 0., -.25], [3.7, 0., .2]])
        cloud = np.r_[self.floor, distant_obstacle]
        plane = module.fit_floor(cloud, np.zeros(3), fit_radius=3.)
        self.assertTrue(module.obstacle_mask(cloud, plane)[-3:].all())
        for radius in (0., -1., float('nan')):
            with self.assertRaises(ValueError): module.fit_floor(cloud, np.zeros(3), fit_radius=radius)

    def test_tracking_follows_small_ground_changes_but_rejects_a_step(self):
        initial = np.array([0., 0., -.44])
        candidate = np.array([.002, 0., -.445])
        updated = module.track_floor(initial, candidate, np.array([4., 2., 0.]))
        np.testing.assert_allclose(updated, .8*initial+.2*candidate)
        with self.assertRaises(ValueError):
            module.track_floor(initial, np.array([0., 0., -.34]), np.zeros(3))

if __name__ == '__main__': unittest.main()
