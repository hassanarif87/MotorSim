"""Continuous-time dq current PI control with voltage anti-windup."""

import numpy as np


def _controller_signals(x, u, params):
    # Inputs: id_ref, iq_ref, id, iq, mechanical omega.
    id_ref, iq_ref, i_d, i_q, omega = u
    error = np.array([id_ref - i_d, iq_ref - i_q])
    omega_e = params['np'] * omega
    feedforward = np.array([
        -omega_e * params['Lq'] * i_q,
        omega_e * params['Lq'] * i_d + params['Ke'] * omega,
    ])
    raw = params['Kp'] * error + np.asarray(x) + feedforward
    limit = params['V_bus'] / np.sqrt(3)
    voltage = raw * min(1.0, limit / max(np.linalg.norm(raw), 1e-12))
    return error, raw, voltage


def current_controller_deriv(t, x, u, params):
    """Integrator states are the d- and q-axis integral voltage contributions."""
    error, raw, voltage = _controller_signals(x, u, params)
    return params['Ki'] * error + params['Kaw'] * (voltage - raw)


def current_controller_output(t, x, u, params):
    """Return [vd, vq] with decoupling and a circular SVPWM voltage limit."""
    return _controller_signals(x, u, params)[2]
