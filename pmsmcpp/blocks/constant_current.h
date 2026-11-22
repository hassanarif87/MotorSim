#pragma once

#include "graph/block_ports.h"
#include "configuration.h"

namespace pmsm {

namespace drive_blocks {

/// Fixed current published by the source, in amperes.
struct ConstantCurrentOutputs { Current value; };

class ConstantCurrent {
    struct Parameters { FiniteReal value_a; };

public:
    static constexpr bool is_constant = true;

    explicit ConstantCurrent(const BlockSpec& spec)
        : outputs{spec.read_parameters<Parameters>().value_a.get()} {}

    NoInputs inputs;
    ConstantCurrentOutputs outputs;
};

}  // namespace drive_blocks

}  // namespace pmsm
