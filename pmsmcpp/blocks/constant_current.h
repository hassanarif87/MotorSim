#pragma once

#include "block_ports.h"
#include "configuration.h"

namespace pmsm {

namespace drive_blocks {

/// Fixed current published by the source, in amperes.
struct ConstantCurrentOutputs { Current value; };

using config_detail::number;

struct ConstantCurrent {
    explicit ConstantCurrent(const BlockSpec& spec)
        : outputs{number(spec.parameters, "value_a")} {}

    NoInputs inputs;
    ConstantCurrentOutputs outputs;
};

}  // namespace drive_blocks

}  // namespace pmsm
