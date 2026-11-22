#pragma once

#include "src/graph/block_ports.h"
#include "src/configuration.h"

namespace pmsm {

namespace drive_blocks {

/// Fixed torque published by the source, in newton-metres.
struct ConstantTorqueOutputs { Torque value; };

class ConstantTorque {
    struct Parameters { FiniteReal value_nm; };

public:
    static constexpr bool is_constant = true;

    explicit ConstantTorque(const BlockSpec& spec)
        : outputs{spec.read_parameters<Parameters>().value_nm.get()} {}

    NoInputs inputs;
    ConstantTorqueOutputs outputs;
};

}  // namespace drive_blocks

}  // namespace pmsm
