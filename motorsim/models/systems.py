import control as ct
from .dc_models import dc_motor_deriv, dc_motor_output, dc_motor_elec_deriv
from .pmsm_motor import pmsm_dq_motor_deriv,  pmsm_dq_motor_output, simple_pmsm_motor_deriv, simple_pmsm_motor_output

def build_dc_plant(params_in):
    """Build the DC motor plant model

    Args:
        params : Motor Parameters
    Returns:
        dc plant io nonlinear system
    """

    dc_plant = ct.NonlinearIOSystem(
        dc_motor_deriv, dc_motor_output, 
        inputs=('u', 'load'), outputs=('ia', 'omega'),
        states=('i_m', 'omega'), name='dc_motor',
        params=params_in
    )

    return dc_plant

def build_simple_pmsm_plant(params_in):
    """Build simple PMSM motor plant model

    Args:
        params : Motor Parameters
    Returns:
        plant io nonlinear system
    """
    plant = ct.NonlinearIOSystem(
        simple_pmsm_motor_deriv, simple_pmsm_motor_output, 
        inputs=('u_q'), outputs=('i_q', 'omega'),
        states=('i_q', 'omega'), name='simple_pmsm',
        params=params_in
    )

    return plant

def build_pmsm_plant(params_in):
    """Build a PMSM with abc voltage inputs and physical signal outputs.

    Internal states are [i_q, i_d, omega, theta]. Outputs are
    [Ia, Ib, Ic, omega, theta], with mechanical speed and position.

    Args:
        params : Motor Parameters
    Returns:
        plant io nonlinear system
    """
    plant = ct.NonlinearIOSystem(
        pmsm_dq_motor_deriv, pmsm_dq_motor_output, 
        inputs=('Va', 'Vb', 'Vc', 'trq'),
        outputs=('Ia', 'Ib', 'Ic', 'omega', 'theta'),
        states=('i_q', 'i_d', 'omega', 'theta'), name='dq_pmsm',
        params=params_in
    )

    return plant
def build_dc_electrical_plant(params_in):
    """Build simple PMSM motor plant model

    Args:
        params : Motor Parameters
    Returns:
        plant io nonlinear system
    """
    plant = ct.NonlinearIOSystem(
        dc_motor_elec_deriv, None, 
        inputs=('u'), outputs=('im'),
        states=('im'), name='electric_plant',
        params=params_in
    )

    return plant
