import control as ct
from .inverter import ideal_inverter_output
from .dc_models import dc_motor_deriv, dc_motor_output, dc_motor_elec_deriv
from .pmsm_motor import pmsm_dq_motor_deriv,  pmsm_dq_motor_output, simple_pmsm_motor_deriv, simple_pmsm_motor_output


def build_ideal_inverter(params_in):
    """Build a stateless averaged inverter with params {'V_bus': volts}.

    Inputs are duty cycles [d_a, d_b, d_c]; outputs are phase-to-neutral
    voltages [Va, Vb, Vc] for a floating star-connected motor.
    """
    ideal_inverter_output(0, [], [0, 0, 0], params_in)
    return ct.NonlinearIOSystem(
        None, ideal_inverter_output,
        inputs=('d_a', 'd_b', 'd_c'), outputs=('Va', 'Vb', 'Vc'),
        name='ideal_inverter', params=params_in,
    )

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


def build_current_controller(params_in):
    """Continuous dq PI block; gains Kp, Ki, Kaw and motor/bus parameters."""
    from .current_controller import current_controller_deriv, current_controller_output

    return ct.NonlinearIOSystem(
        current_controller_deriv, current_controller_output,
        inputs=('id_ref', 'iq_ref', 'i_d', 'i_q', 'omega'),
        outputs=('vd', 'vq'), states=('integral_d', 'integral_q'),
        name='current_controller', params=params_in,
    )


def build_svpwm(params_in):
    """Stateless vd/vq to duty-cycle block; theta is mechanical position."""
    from .transforms import svpwm

    def output(t, x, u, params):
        vd, vq, theta = u
        return svpwm(params['np'] * theta, vd, vq, params['V_bus']) / params['V_bus']

    return ct.NonlinearIOSystem(
        None, output, inputs=('vd', 'vq', 'theta'),
        outputs=('d_a', 'd_b', 'd_c'),
        name='svpwm', params=params_in,
    )


def build_current_feedback(params_in):
    """Convert measured phase currents to dq using mechanical position."""
    from .transforms import clarke_park

    def output(t, x, u, params):
        return clarke_park(params['np'] * u[3], u[:3])

    return ct.NonlinearIOSystem(
        None, output, inputs=('Ia', 'Ib', 'Ic', 'theta'),
        outputs=('i_d', 'i_q'),
        name='current_feedback', params=params_in,
    )


def build_current_controlled_pmsm(motor_params, inverter_params, controller_gains):
    """Connect the current controller, SVPWM, inverter, motor and sensing.

    External inputs: id_ref, iq_ref (A), load torque (Nm).
    Physical motor outputs come first; remaining outputs are diagnostics.
    Controller integrator states precede the four motor states.
    """
    params = {**motor_params, **inverter_params, **controller_gains}
    controller = build_current_controller(params)
    pwm = build_svpwm(params)
    inverter = build_ideal_inverter(inverter_params)
    motor = build_pmsm_plant(motor_params)
    feedback = build_current_feedback(motor_params)
    return ct.interconnect(
        [controller, pwm, inverter, motor, feedback],
        connections=[
            ['svpwm.vd', 'current_controller.vd'],
            ['svpwm.vq', 'current_controller.vq'],
            ['svpwm.theta', 'dq_pmsm.theta'],
            *[[f'ideal_inverter.d_{p}', f'svpwm.d_{p}'] for p in 'abc'],
            *[[f'dq_pmsm.V{p}', f'ideal_inverter.V{p}'] for p in 'abc'],
            *[[f'current_feedback.I{p}', f'dq_pmsm.I{p}'] for p in 'abc'],
            ['current_feedback.theta', 'dq_pmsm.theta'],
            ['current_controller.i_d', 'current_feedback.i_d'],
            ['current_controller.i_q', 'current_feedback.i_q'],
            ['current_controller.omega', 'dq_pmsm.omega'],
        ],
        inplist=['current_controller.id_ref', 'current_controller.iq_ref', 'dq_pmsm.trq'],
        inputs=('id_ref', 'iq_ref', 'trq'),
        outlist=[*[f'dq_pmsm.{s}' for s in ('Ia', 'Ib', 'Ic', 'omega', 'theta')],
                 'current_feedback.i_d', 'current_feedback.i_q',
                 'current_controller.vd', 'current_controller.vq',
                 *[f'svpwm.d_{p}' for p in 'abc'],
                 *[f'ideal_inverter.V{p}' for p in 'abc']],
        outputs=('Ia', 'Ib', 'Ic', 'omega', 'theta', 'i_d', 'i_q',
                 'vd', 'vq', 'd_a', 'd_b', 'd_c', 'Va', 'Vb', 'Vc'),
        name='current_controlled_pmsm',
    )
