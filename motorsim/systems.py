import control as ct
from models import dc_motor_deriv, dc_motor_output

def build_dc_plant(params_in):
    """Build the DC motor plant model

    Args:
        params : Motor Parameters
    Returns:
        dc plant io nonlinear system
    """

    dc_plant = ct.NonlinearIOSystem(
        dc_motor_deriv, dc_motor_output, 
        inputs=('u', 'load'), outputs=('omega'),
        states=('i_m', 'omega'), name='dc_motor',
        params=params_in
    )

    return dc_plant
