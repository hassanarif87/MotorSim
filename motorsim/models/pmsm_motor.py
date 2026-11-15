
import numpy as np
import numpy.typing as npt
from .transforms import clarke_park, clarke_park_inv

def pmsm_dq_motor_deriv(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a PMSM motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector [iq, id, omega, theta], mechanical rad/s and rad
        u : Input vector [Va, Vb, Vc, load torque], voltages in phase order abc
        params : Physical parameters of the motor

    Returns:
        Derivative of the state vector
    """
    
    kt= params.get('Kt', 0.1) # Torque constant 
    ke= params.get('Ke', 0.1) # Torque constant Kt = Ke 
    l_q = params.get('Lq', 0.03) # Motor dq inductance assuming Lq = Ld
    r_p = params.get('Rm', 0.03) # Motor phase resistance
    n_p = params.get('np', 1) # Pole pairs
    rotor_inertia = params.get('J', 0.03) # Rotor inertia 
    viscous_friction = params.get('b', 0) # viscous friction

    theta_e = n_p * x[3]
    # Applied inverter voltages; the transform rejects common-mode voltage.
    u_d, u_q = clarke_park(theta_e, u[:3])
    load_trq = u[3]
    # Map the states into local variable names
    i_q = x[0]
    i_d = x[1]
    omega = x[2]
    dot_id = (-r_p *i_d + n_p * omega *l_q*i_q + u_d) / l_q
    dot_iq = (-r_p *i_q -  omega * ( n_p * l_q * i_d + ke) + u_q) / l_q
    trq = 3/2  * kt * i_q

    dot_omega = (trq  - load_trq - viscous_friction *omega) / rotor_inertia

    return np.array([dot_iq, dot_id, dot_omega, omega])

def pmsm_dq_motor_output(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a PMSM motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector [iq, id, omega, theta], mechanical rad/s and rad
        u : Input vector [Va, Vb, Vc, load torque]
        params : Physical parameters of the motor

    Returns:
        [Ia, Ib, Ic, omega, theta]: phase currents (A), mechanical
        velocity (rad/s) and mechanical position (rad)
    """

    i_q, i_d, omega, theta = x
    theta_e = params.get('np', 1) * theta
    i_abc = clarke_park_inv(theta_e, [i_d, i_q])
    return np.array([*i_abc, omega, theta])


def simple_pmsm_motor_deriv(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector [iq, omega]
        u : Input vector, [u_q, trq]
        params : Physical parameters of the motor

    Returns:
        Derivative of the state vector
    """
    
    kt= params.get('Kt', 0.1) # Torque constant 
    ke= params.get('Ke', 0.1) # B-emf constant 
    v_max = params.get('V_max', 10) # Battery voltage
    l_q = params.get('Lq', 0.03) # Motor dq inductance assuming Lq = Ld
    r_p = params.get('Rm', 0.03) # Motor phase resistance
    n_p = params.get('np', 1) # Pole pairs
    rotor_inertia = params.get('J', 0.03) # Rotor inertia 
    viscous_friction = params.get('b', 0) # viscous friction
    u_q_in = u[0]
    load_trq = 0 #u[1]
    # Map the states into local variable names
    i_q = x[0]
    omega = x[1]
    u_d = -n_p * omega *l_q*i_q  # Assuming dot_id  = 0, i_d = 0
    u_q_sq = u_q_in**2
    # Assuming SVPWM
    np.min([u_q_sq, v_max**2/3 - u_d**2])
    u_q_sq = np.max([u_q_sq, 0.0])
    u_q = np.sqrt(u_q_sq)

    dot_iq = (-r_p * i_q - omega *  ke + u_q) / l_q
    trq = 3/2  * kt * i_q
    dot_omega = (trq  - load_trq - viscous_friction *x[1]) / rotor_inertia

    return np.array([dot_iq,dot_omega])


def simple_pmsm_motor_output(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector [ iq, omega]
        u : Input vector, [Voltage modulation fraction,  V_dc * u; load on the motor Nm]
        params : Physical parameters of the motor

    Returns:
        motor current and velocity
    """

    return x
