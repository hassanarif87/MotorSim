#pragma once

#include <stdexcept>

#include "blocks/transforms.h"

#include "graph/block_ports.h"
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

class Pwm {
    struct Parameters {
        PositiveInt pole_pairs;
        PositiveReal bus_voltage_v;
    };

public:
    explicit Pwm(const BlockSpec& spec) : Pwm(spec.read_parameters<Parameters>()) {}

    void tick() {
        const auto duty = svpwm({inputs.vd.read(), inputs.vq.read()},
                                pole_pairs_ * inputs.theta.read(), bus_voltage_);
        outputs = {duty.abc[0], duty.abc[1], duty.abc[2]};
    }

    PwmInputs inputs;
    DutyOutputs outputs;

private:
    explicit Pwm(const Parameters& p)
        : pole_pairs_(p.pole_pairs.get()),
          bus_voltage_(p.bus_voltage_v.get()) {}

    int pole_pairs_;
    double bus_voltage_;
};

}  // namespace drive_blocks

}  // namespace pmsm
