#pragma once

#include <chrono>
#include <optional>
#include <stdexcept>

#include "src/blocks/transforms.h"

#include "src/graph/block_ports.h"
#include "src/configuration.h"

namespace pmsm {

inline DutyCycles svpwm(DqVoltage dq, double theta_e, double bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0) {
        throw std::invalid_argument("DC bus voltage must be positive and finite");
    }
    const auto abc = inverse_clarke_park(theta_e, dq.d, dq.q);
    const double center = (bus_voltage - abc.minCoeff() - abc.maxCoeff()) / 2.0;
    return {(abc.array() + center) / bus_voltage};
}

namespace drive_blocks {

/// dq voltage command (V), mechanical position (rad), and speed (rad/s).
struct PwmInputs {
    InputPort<Voltage> vd, vq;
    InputPort<Position> theta;
    InputPort<Speed> omega;
};

/// Normalized phase duty commands published by SVPWM.
struct DutyOutputs { Duty duty_a, duty_b, duty_c; };

class Pwm {
    struct Parameters {
        PositiveInt pole_pairs;
        PositiveReal bus_voltage_v;
        std::optional<bool> predict_midpoint_angle;
    };

public:
    explicit Pwm(const BlockSpec& spec)
        : Pwm(spec.read_parameters<Parameters>(),
              std::chrono::duration<double>(spec.period().value()).count()) {}

    void tick() {
        // Center the voltage vector over the interval for which the inverter holds it.
        const double theta_mid = inputs.theta.read() + half_period_s_ * inputs.omega.read();
        const auto duty = svpwm({inputs.vd.read(), inputs.vq.read()},
                                pole_pairs_ * theta_mid, bus_voltage_);
        outputs = {duty.abc[0], duty.abc[1], duty.abc[2]};
    }

    PwmInputs inputs;
    DutyOutputs outputs;

private:
    Pwm(const Parameters& p, double period_s)
        : pole_pairs_(p.pole_pairs.get()),
          bus_voltage_(p.bus_voltage_v.get()),
          half_period_s_(p.predict_midpoint_angle.value_or(true) ? period_s / 2.0 : 0.0) {}

    int pole_pairs_;
    double bus_voltage_;
    double half_period_s_;
};

}  // namespace drive_blocks

}  // namespace pmsm
