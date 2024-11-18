import numpy as np


def clarke_park_inv(theta_e: float, u_qd: np.array):
    """
    Clarke-Park inverse transform
    dq0 (rotor frame) -> alpha beta -> three phase Stator frame
    Args:
        theta_e Electrical angle
        Vdq [Vd Vq] 
    Returns:
        Vector of voltages in the stator frame[Va Vb Vc] 
    """
    clark_inv = 3. / 2. * np.array([
        [2. / 3., 0.], 
        [- 1. / 3., np.sqrt(3.) / 3.], 
        [- 1. / 3., - np.sqrt(3.) / 3.]
        ])
    park_inv = np.array([
        [np.cos(theta_e), - np.sin(theta_e)],
        [ np.sin(theta_e), np.cos(theta_e)]
        ])
    return clark_inv @ park_inv @ u_qd

def clarke_park(theta_e: float, u_phase: np.array):
    """
    Clarke-Park direct transform
    Three phase Stator frame -> alpha beta ->  dq0 (rotor frame) 

    Args:
        theta_e: Electrical angle
        u_phase: Vector of voltages in the Stator frame [Va Vb Vc] 
    Returns
        Voltage in the rotor frame [Vd Vq] 
    """
    clark = 2. / 3. * np.array([
        [1., - 0.5, - 0.5], 
        [0., np.sqrt(3.) / 2., - np.sqrt(3.) / 2.]
        ])
    park = np.array([
        [np.cos(theta_e), np.sin(theta_e)],
        [ -np.sin(theta_e), np.cos(theta_e)]
        ])
    return park @ clark @ u_phase


def svpwm(el_theta: float, ud: float, uq: float, u_supply: float) -> np.array:
    """Space-vector PWM to compute phase voltage from direct/quadrature vectors.

    This code is derived from the version presented in the SimpleFOC library,
    https://docs.simplefoc.com/foc_theory

    Args:
        theta_el : Electrical angle (rad)
        Vdq : Voltage in the quadrature direction (V)
        Vdc : Supply Voltage V

    Returns:
        Three phase voltage
    """
    U_abc= clarke_park_inv(el_theta, [ud,uq])
    center = u_supply/2.
    Umin = min(U_abc[0], min(U_abc[1], U_abc[2]))
    Umax = max(U_abc[0], max(U_abc[1], U_abc[2]))
    center -= (Umax+Umin) / 2

    U_abc[0]+= center
    U_abc[1]+= center
    U_abc[2]+= center

    return U_abc