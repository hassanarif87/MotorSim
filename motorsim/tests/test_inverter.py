import unittest

import numpy as np

from motorsim.models.inverter import ideal_inverter_output
from motorsim.models.transforms import clarke_park, svpwm


class TestIdealInverter(unittest.TestCase):
    def test_phase_to_neutral_voltages_and_duty_limits(self):
        result = ideal_inverter_output(0, [], [1.2, 0.5, -0.2], {'V_bus': 48})
        np.testing.assert_allclose(result, [24, 0, -24])

    def test_equal_duties_produce_zero_motor_voltage(self):
        for duty in [0, 0.5, 1]:
            np.testing.assert_allclose(
                ideal_inverter_output(0, [], [duty] * 3, {'V_bus': 48}), 0)

    def test_svpwm_to_inverter_preserves_requested_dq_voltage(self):
        for angle in [0, 0.7, 2.3]:
            duties = svpwm(angle, ud=3, uq=10, u_supply=48) / 48
            v_abc = ideal_inverter_output(0, [], duties, {'V_bus': 48})
            np.testing.assert_allclose(clarke_park(angle, v_abc), [3, 10])

    def test_invalid_bus_voltage(self):
        for voltage in [0, -48, np.nan, np.inf]:
            with self.assertRaises(ValueError):
                ideal_inverter_output(0, [], [0.5] * 3, {'V_bus': voltage})


if __name__ == '__main__':
    unittest.main()
