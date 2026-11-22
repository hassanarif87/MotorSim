#pragma once

#include "blocks/continuous_pmsm.h"
#include "blocks/current_feedback.h"
#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/speed_control.h"
#include "blocks/svpwm.h"

namespace pmsm {

struct DriveSignals {
    DqCurrent reference;
    DqCurrent measured{};
    DqVoltage command{};
    DutyCycles duty{Eigen::Vector3d::Zero()};
    PhaseVoltage voltage{Eigen::Vector3d::Zero()};
};

class CurrentControlledPmsm {
public:
    CurrentControlledPmsm(ContinuousPmsm& motor, CurrentPi controller, SpeedControl speed,
                          double bus_voltage, double speed_reference,
                          double id_reference, SimulationExecutor::Duration current_period)
        : motor_(motor),
          controller_(controller, std::chrono::duration<double>(current_period).count()),
          speed_(speed),
          bus_voltage_(bus_voltage), speed_reference_(speed_reference),
          signals_{{id_reference, 0.0}} {}

    void tick_speed() {
        signals_.reference.q = speed_.output(speed_reference_, motor_.output().omega);
    }

    void tick_current() {
        const auto& motor = motor_.output();
        signals_.measured = current_feedback(motor.current, motor_.pole_pairs() * motor.theta);
        signals_.command = controller_.tick(
            {signals_.reference, signals_.measured, motor.omega, bus_voltage_});
    }

    void tick_pwm() {
        signals_.duty = svpwm(signals_.command, motor_.pole_pairs() * motor_.output().theta, bus_voltage_);
        signals_.voltage = ideal_inverter(signals_.duty, bus_voltage_);
        motor_.set_voltage(signals_.voltage);
    }

    const DriveSignals& signals() const { return signals_; }

private:
    ContinuousPmsm& motor_;
    DigitalCurrentPi controller_;
    SpeedControl speed_;
    double bus_voltage_;
    double speed_reference_;
    DriveSignals signals_;
};

}  // namespace pmsm
