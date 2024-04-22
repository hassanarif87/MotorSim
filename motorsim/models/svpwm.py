
import numpy as np

def svpwm(theta_el: float, Vd, Vq, Vdc: float):
    """_summary_
    Space-vector PWM to compute phase voltage from direct/quadrature vectors.

    This code is derived from the version presented in the SimpleFOC library,
    https://docs.simplefoc.com/foc_theory

    Args:
        theta_el : Electrical angle (rad)
        Vdq : Voltage in the quadrature direction (V)
        Vdc : Supply Voltage V

    Returns:
        Three phase voltage
    """

    # Compute amplitude and angle - clamping as needed
    Uout = np.linalg.norm([Vd, Vq]) / Vdc * np.sqrt(3)
    Uout = min(Uout, 1)

    angle = (theta_el + np.arctan2(Vd, Vq)) % (2 * np.pi)

    # Find the sector
    sector = np.floor(angle / (np.pi / 3.0)) + 1

    # Calculate duty cycles
    T1 = np.sqrt(3) * np.sin(sector * np.pi / 3 - angle) * Uout
    T2 = np.sqrt(3) * np.sin(angle - (sector - 1) * np.pi / 3) * Uout
    T0 = 1 - T1 - T2

    if sector == 1:
        Ta = T1 + T2 + T0/2
        Tb = T2 + T0/2
        Tc = T0/2
    elif sector == 2:
          Ta = T1 +  T0/2
          Tb = T1 + T2 + T0/2
          Tc = T0/2
    elif sector == 3:
          Ta = T0/2
          Tb = T1 + T2 + T0/2
          Tc = T2 + T0/2
    elif sector == 4:
          Ta = T0/2
          Tb = T1+ T0/2
          Tc = T1 + T2 + T0/2
    elif sector == 5:
          Ta = T2 + T0/2
          Tb = T0/2
          Tc = T1 + T2 + T0/2
    else:
          Ta = T1 + T2 + T0/2
          Tb = T0/2
          Tc = T1 + T0/2

    # Calculate the phase voltages, recentering them
    #average = (Ta + Tb + Tc) / 3.0
    #Ua = (Ta - average) * Vdc / np.sqrt(3)
    #Ub = (Tb - average) * Vdc / np.sqrt(3)
    #Uc = (Tc - average) * Vdc / np.sqrt(3)
    Ua = (Ta) * Vdc
    Ub = (Tb) * Vdc
    Uc = (Tc) * Vdc
    return np.array([Ua, Ub, Uc])