#pragma once

#include <Eigen/Core>

#include "src/graph/block_ports.h"

namespace pmsm {

struct DqCurrent {
    double d;
    double q;
};

struct DqVoltage {
    double d;
    double q;
};

struct PhaseCurrent {
    Eigen::Vector3d abc;
};

struct PhaseVoltage {
    Eigen::Vector3d abc;
};

struct DutyCycles {
    Eigen::Vector3d abc;
};

namespace drive_blocks {

/// Motor-neutral phase voltages published by inverter blocks, in volts.
struct PhaseVoltageOutputs {
    Voltage va, vb, vc;
};

} // namespace drive_blocks
} // namespace pmsm
