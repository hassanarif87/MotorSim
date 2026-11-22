import unittest

import numpy as np

from motorsim.models.pmsm_motor import pmsm_dq_motor_deriv, pmsm_dq_motor_output


class TestPmsmMotor(unittest.TestCase):
    def test_phase_voltage_uses_electrical_angle_and_preserves_state_order(self):
        # With two pole pairs, theta_m=pi/4 gives theta_e=pi/2.
        # [0, sqrt(3), -sqrt(3)] then corresponds to vd=2, vq=0.
        params = dict(np=2, Rm=0.5, Lq=0.25, Ke=0.2, Kt=0.2, J=0.1, b=0.01)
        x = [3.0, 4.0, 5.0, np.pi / 4]
        u = [0.0, np.sqrt(3), -np.sqrt(3), 0.1]
        result = pmsm_dq_motor_deriv(0, x, u, params)
        np.testing.assert_allclose(result, [-50.0, 30.0, 7.5, 5.0])

    def test_output_converts_dq_to_phases_using_electrical_angle(self):
        x = [3.0, 4.0, 5.0, np.pi / 4]
        # theta_e=pi/2: alpha=-iq=-3, beta=id=4.
        expected = [-3.0, 1.5 + 2 * np.sqrt(3),
                    1.5 - 2 * np.sqrt(3), 5.0, np.pi / 4]
        result = pmsm_dq_motor_output(0, x, [0, 0, 0, 0], {'np': 2})
        np.testing.assert_allclose(result, expected)
        self.assertAlmostEqual(sum(result[:3]), 0.0)

    def test_output_at_zero_angle_with_default_pole_pairs(self):
        result = pmsm_dq_motor_output(0, [0, 2, 7, 0], [0, 0, 0, 0], {})
        np.testing.assert_allclose(result, [2, -1, -1, 7, 0])

    def test_common_mode_does_not_change_motor_response(self):
        x = [0.0, 0.0, 0.0, 0.3]
        u = np.array([20.0, -10.0, -10.0, 0.0])
        expected = pmsm_dq_motor_deriv(0, x, u, {})
        u[:3] += 24.0
        np.testing.assert_allclose(pmsm_dq_motor_deriv(0, x, u, {}), expected)


if __name__ == '__main__':
    unittest.main()
