#pragma once

#include <stdexcept>

#include "blocks/transforms.h"

#include "block_ports.h"
#include "configuration.h"

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

/// Latest dq voltage command (V) and mechanical position (rad) sampled by SVPWM.
struct PwmInputs {
    InputPort<Voltage> vd, vq;
    InputPort<Position> theta;
};

/// Normalized phase duty commands published by SVPWM.
struct DutyOutputs { Duty duty_a, duty_b, duty_c; };

using config_detail::bounded_number;
using config_detail::positive_integer;

class Pwm {
public:
    explicit Pwm(const BlockSpec& spec)
        : pole_pairs_(positive_integer(spec.parameters, "pole_pairs")),
          bus_voltage_(bounded_number(spec.parameters, "bus_voltage_v", false)) {}

    void tick() {
        const auto duty = svpwm({inputs.vd.read(), inputs.vq.read()},
                                pole_pairs_ * inputs.theta.read(), bus_voltage_);
        outputs = {duty.abc[0], duty.abc[1], duty.abc[2]};
    }

    PwmInputs inputs;
    DutyOutputs outputs;

private:
    int pole_pairs_;
    double bus_voltage_;
};

}  // namespace drive_blocks

}  // namespace pmsm
