#!/usr/bin/env python3
import math
import unittest

import numpy as np
import pandas as pd

from analyze_static_sweep import crossing, metrics


class StaticSweepAnalysisTest(unittest.TestCase):
    def test_crossing_interpolates_descending_curve(self):
        self.assertAlmostEqual(crossing(np.array([-1.0, 0.0, 1.0]), np.array([1.0, 0.5, 0.0]), 0.75), -0.5)

    def test_linear_metrics(self):
        z = np.linspace(-1.0, 1.0, 201)
        ratio = np.clip((1.0 - z) / 2.0, 0.0, 1.0)
        frame = pd.DataFrame(
            {
                "source": "Gazebo",
                "pitch_deg": 0.0,
                "z_m": z,
                "submersion_ratio_body": ratio,
                "buoyancy_z_N": 20.0 * ratio,
                "net_force_z_N": 20.0 * ratio - 10.0,
            }
        )
        row = metrics(frame)
        self.assertAlmostEqual(row["z_at_ratio_0.99"], -0.98)
        self.assertAlmostEqual(row["z_at_ratio_0.50"], 0.0)
        self.assertAlmostEqual(row["z_at_ratio_0.01"], 0.98)
        self.assertAlmostEqual(row["transition_width_z"], 1.96)
        self.assertAlmostEqual(row["equilibrium_z"], 0.0)
        self.assertLess(row["smoothness_dFdz"], 1e-9)

    def test_missing_equilibrium_is_nan(self):
        self.assertTrue(math.isnan(crossing(np.array([0.0, 1.0]), np.array([1.0, 2.0]), 0.0)))


if __name__ == "__main__":
    unittest.main()
