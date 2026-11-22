import unittest

import control as ct
import numpy as np

from motorsim.models.current_controller import (
    current_controller_deriv, current_controller_output,
)
from motorsim.models.systems import build_current_controlled_pmsm


class TestCurrentControl(unittest.TestCase):
    def setUp(self):
        self.motor = dict(Rm=0.275, Lq=0.003, np=4, Ke=0.1, Kt=0.1,
                          J=0.01, b=0.001)
        bandwidth = 2 * np.pi * 200
        self.gains = dict(Kp=0.003 * bandwidth, Ki=0.275 * bandwidth,
                          Kaw=bandwidth)

    def test_voltage_limit_and_antiwindup(self):
        params = {**self.motor, **self.gains, 'V_bus': 48}
        # Large positive q integral with zero error must unwind when saturated.
        u = [0, 0, 0, 0, 0]
        x = [0, 100]
        voltage = current_controller_output(0, x, u, params)
        np.testing.assert_allclose(voltage, [0, 48 / np.sqrt(3)])
        self.assertLess(current_controller_deriv(0, x, u, params)[1], 0)

    def test_connected_drive_tracks_both_current_directions(self):
        drive = build_current_controlled_pmsm(self.motor, {'V_bus': 48}, self.gains)
        self.assertEqual(drive.nstates, 6)
        self.assertEqual(drive.ninputs, 3)
        # Start at a nonzero rotor angle to exercise position feedback wiring.
        initial = np.array([0, 0, 0, 0, 0, 0.31])
        for start, iq_ref in [(0, 3), (0.05, -3)]:
            t = np.linspace(start, start + 0.05, 101)
            u = np.tile([0, iq_ref, 0], (len(t), 1)).T
            response = ct.input_output_response(
                drive, t, u, X0=initial, return_x=True,
                solve_ivp_kwargs={'rtol': 1e-7, 'atol': 1e-9},
            )
            y = response.outputs
            np.testing.assert_allclose(y[5:7, -1], [0, iq_ref], atol=0.01)
            np.testing.assert_allclose(y[:3].sum(axis=0), 0, atol=1e-12)
            self.assertGreaterEqual(y[9:12].min(), -1e-12)
            self.assertLessEqual(y[9:12].max(), 1 + 1e-12)
            initial = response.states[:, -1]


if __name__ == '__main__':
    unittest.main()
