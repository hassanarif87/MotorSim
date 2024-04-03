
import numpy as np
import numpy.typing as npt

def pmsm_dq_motor_deriv(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector [ id, iq, omega]
        u : Input vector, [u_q, u_d, trq]
        params : Physical parameters of the motor

    Returns:
        Derivative of the state vector
    """
    
    kt= params.get('Kt', 0.1) # Torque constant 
    ke= params.get('Ke', 0.1) # Torque constant Kt = Ke 
    v_max = params.get('V_max', 10) # Battery voltage
    l_q = params.get('Lq', 0.03) # Motor dq inductance assuming Lq = Ld
    r_p = params.get('Rm', 0.03) # Motor phase resistance
    n_p = params.get('np', 0.03) # Pole pairs
    rotor_inertia = params.get('J', 0.03) # Rotor inertia 
    viscous_friction = params.get('b', 0) # viscous friction

    u_q = np.clip( u[0], -v_max, v_max)
    u_d = np.clip( u[1], -v_max, v_max)
    load_trq = u[2]
    # Map the states into local variable names
    i_d = x[0]
    i_q = x[1]
    omega = x[1]
    dot_id = (-r_p *i_d + n_p * omega *l_q*i_q + u_d) / l_q
    dot_iq = (-r_p *i_q + omega * ( n_p * l_q + ke) + u_q) / l_q

    dot_omega = (kt*x[0]  - load_trq - viscous_friction *x[1]) / rotor_inertia

    return np.array([dot_id, dot_iq,dot_omega])

def pmsm_dq_motor_output(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector, omega rotational speed of the motor rad/s
        u : Input vector, [Voltage modulation fraction,  V_dc * u; load on the motor Nm]
        params : Physical parameters of the motor

    Returns:
        motor velocity
    """

    return x


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
    ke= params.get('Ke', 0.1) # Torque constant Kt = Ke 
    v_max = params.get('V_max', 10) # Battery voltage
    l_q = params.get('Lq', 0.03) # Motor dq inductance assuming Lq = Ld
    r_p = params.get('Rm', 0.03) # Motor phase resistance
    n_p = params.get('np', 0.03) # Pole pairs
    rotor_inertia = params.get('J', 0.03) # Rotor inertia 
    viscous_friction = params.get('b', 0) # viscous friction
    u_q_in = u[0]
    load_trq = u[1]
    # Map the states into local variable names
    i_q = x[0]
    omega = x[1]
    u_d = n_p * omega *l_q*i_q  # Assuming dot_id  = 0, i_d = 0

    u_q =  np.sqrt( np.min(u_q_in**2, v_max**2/3 - u_d**2)) # Assuming SVPWM 

    dot_iq = (-r_p *i_q + omega * ( n_p * l_q + ke) + u_q) / l_q

    dot_omega = (kt*x[0]  - load_trq - viscous_friction *x[1]) / rotor_inertia

    return np.array([dot_iq,dot_omega])


def simple_pmsm_motor_output(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 

    Args:
        t : simulation time (s)
        x : state vector, omega rotational speed of the motor rad/s
        u : Input vector, [Voltage modulation fraction,  V_dc * u; load on the motor Nm]
        params : Physical parameters of the motor

    Returns:
        motor velocity
    """

    return x