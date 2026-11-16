"""Ideal, PWM-period-averaged three-phase inverter."""

import numpy as np
import numpy.typing as npt


def ideal_inverter_output(
    t: float, x: npt.ArrayLike, u: npt.ArrayLike, params: dict
) -> npt.NDArray[np.float64]:
    """Map [d_a, d_b, d_c] duties to [Va, Vb, Vc] in volts.

    V_bus is the positive DC bus voltage. Duties are limited to [0, 1].
    Voltages are relative to a floating star-connected motor neutral.
    No switching ripple, dead time, or semiconductor losses are modeled.

    The existing transforms.svpwm() returns pole-voltage references;
    divide its output by V_bus to obtain these duty-cycle inputs.
    """
    v_bus = float(params['V_bus'])
    duties = np.asarray(u, dtype=float)
    if duties.shape != (3,) or not np.all(np.isfinite(duties)):
        raise ValueError('Expected three finite duty cycles [d_a, d_b, d_c]')
    v_pole = v_bus * np.clip(duties, 0.0, 1.0)
    return v_pole - v_pole.mean()
