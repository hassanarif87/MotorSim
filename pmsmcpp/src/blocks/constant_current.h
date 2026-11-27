#pragma once

#include "src/graph/block_ports.h"
#include "src/exec/configuration.h"

namespace pmsm {

namespace drive_blocks {

/// @brief Fixed current published by the source, in amperes.
struct ConstantCurrentOutputs {
    Current value;
};

class ConstantCurrent {
    struct Parameters {
        /// @brief The constant current value in amperes.
        FiniteReal value_a;
    };

  public:
    static constexpr bool is_constant = true;

    explicit ConstantCurrent(const BlockSpec& spec)
        : outputs{spec.read_parameters<Parameters>().value_a.get()} {}

    NoInputs inputs;
    ConstantCurrentOutputs outputs;
};

} // namespace drive_blocks

} // namespace pmsm
