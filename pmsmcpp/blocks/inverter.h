#pragma once

#include <cmath>
#include <stdexcept>

#include "blocks/signals.h"

#include "graph/block_ports.h"
#include "configuration.h"

namespace pmsm {

inline PhaseVoltage ideal_inverter(DutyCycles duty, double bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0 || !duty.abc.allFinite()) {
        throw std::invalid_argument("Inverter requires finite duties and a positive finite DC bus");
    }
    const Eigen::Vector3d pole = bus_voltage * duty.abc.cwiseMax(0.0).cwiseMin(1.0);
    return {pole.array() - pole.mean()};
}

namespace drive_blocks {

/// Normalized phase duty commands consumed by the inverter.
struct DutyInputs { InputPort<Duty> duty_a, duty_b, duty_c; };

/// Motor-neutral phase voltages published by the averaged inverter, in volts.
struct PhaseVoltageOutputs { Voltage va, vb, vc; };

using config_detail::bounded_number;

class Inverter {
public:
    explicit Inverter(const BlockSpec& spec)
        : bus_voltage_(bounded_number(spec.parameters, "bus_voltage_v", false)) {}

    void tick() {
        const auto voltage = ideal_inverter(
            {{inputs.duty_a.read(), inputs.duty_b.read(), inputs.duty_c.read()}}, bus_voltage_);
        outputs = {voltage.abc[0], voltage.abc[1], voltage.abc[2]};
    }

    DutyInputs inputs;
    PhaseVoltageOutputs outputs;

private:
    double bus_voltage_;
};

}  // namespace drive_blocks

}  // namespace pmsm
