#pragma once

#include "src/graph/block_ports.h"
#include "src/exec/configuration.h"

namespace pmsm {

namespace drive_blocks {

/// @brief Fixed torque published by the source, in newton-metres.
struct ConstantTorqueOutputs {
    Torque value;
};

class ConstantTorque {
    struct Parameters {
        /// @brief The constant torque value in newton-metres.
        FiniteReal value_nm;
    };

  public:
    static constexpr bool is_constant = true;

    explicit ConstantTorque(const BlockSpec& spec)
        : outputs{spec.read_parameters<Parameters>().value_nm.get()} {}

    NoInputs inputs;
    ConstantTorqueOutputs outputs;
};

} // namespace drive_blocks

} // namespace pmsm
