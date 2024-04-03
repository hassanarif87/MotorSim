import control as ct
from motorsim.models.dc_models import dc_motor_deriv, dc_motor_output
from models.pmsm_motor import simple_pmsm_motor_deriv, simple_pmsm_motor_output

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
        inputs=('u_q', 'load'), outputs=('i_q', 'omega'),
        states=('i_q', 'omega'), name='simple_pmsm',
        params=params_in
    )

    return plant
