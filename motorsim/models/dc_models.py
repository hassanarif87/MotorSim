
import numpy as np
import numpy.typing as npt

def dc_motor_deriv(
        t: float, 
        x: npt.ArrayLike, 
        u: npt.ArrayLike, 
        params: dict) -> npt.ArrayLike:
    """Eom of a simple DC motor with an attached load 
    T= Kt i
    V=Keω=Ke Θa˙
    Args:
        t : simulation time (s)
        x : state vector, 
            i_m motor armature current A
            omega rotational speed of the motor rad/s
        u : Input vector
            Voltage modulation fraction(PWM),  (V_dc * u)
            load on the motor Nm
        params : Physical parameters of the motor

    Returns:
        Derivative of the state vector
    """
    
    kt= params.get('Kt', 0.1) # Torque constant Kt = Ke 
    v_dc = params.get('v_dc', 10) # Battery voltage
    l_m = params.get('Lm', 0.03) # Motor inductance
    r_m = params.get('Rm', 0.03) # Motor winding resistance
    rotor_inertia = params.get('J', 0.03) # Rotor inertia 
    viscous_friction = params.get('b', 0) # viscous friction

    v_applied = np.clip(v_dc * u[0], -v_dc, v_dc)
    load_trq = u[1]
    # Map the states into local variable names
    
    dot_i = (v_applied - r_m*x[0] - kt*x[1] ) / l_m
    dot_omega = (kt*x[0]  - load_trq - viscous_friction *x[1]) / rotor_inertia

    return np.array([dot_i, dot_omega])

def dc_motor_output(
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