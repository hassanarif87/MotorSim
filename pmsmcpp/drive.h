#pragma once

#include "blocks/current_feedback.h"
#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/plant.h"
#include "blocks/svpwm.h"

namespace pmsm {

struct DriveInput {
    DqCurrent reference;
    double load_torque;
    double bus_voltage;
};

struct DriveEvaluation {
    MotorOutput motor;
    DqCurrent measured;
    DqVoltage command;
    DutyCycles duty;
    PhaseVoltage voltage;
    Eigen::Matrix<double, 6, 1> derivative;
};

class CurrentControlledPmsm {
public:
    // [iq, id, omega, theta, integral_d, integral_q]
    using State = Eigen::Matrix<double, 6, 1>;

    CurrentControlledPmsm(PmsmPlant plant, CurrentPi controller)
        : plant_(plant), controller_(controller) {}

    DriveEvaluation evaluate(const State& state, const DriveInput& input) const {
        const MotorState motor_state{state[0], state[1], state[2], state[3]};
        const auto motor = plant_.output(motor_state);
        const double theta_e = plant_.pole_pairs() * motor.theta;
        const auto measured = current_feedback(motor.current, theta_e);
        const auto control = controller_.evaluate(
            state.tail<2>(), {input.reference, measured, motor.omega, input.bus_voltage});
        const auto duty = svpwm(control.voltage, theta_e, input.bus_voltage);
        const auto voltage = ideal_inverter(duty, input.bus_voltage);
        const auto motor_rate = plant_.derivative(motor_state, {voltage, input.load_torque});

        State rate;
        rate << motor_rate.iq, motor_rate.id, motor_rate.omega, motor_rate.theta,
            control.integral_derivative;
        return {motor, measured, control.voltage, duty, voltage, rate};
    }

private:
    PmsmPlant plant_;
    CurrentPi controller_;
};

}  // namespace pmsm
