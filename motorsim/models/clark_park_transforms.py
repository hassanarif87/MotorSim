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
