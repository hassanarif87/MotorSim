#pragma once

#include <Eigen/Core>

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

}  // namespace pmsm
