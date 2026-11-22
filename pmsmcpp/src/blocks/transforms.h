#pragma once

#include <cmath>

#include "src/blocks/signals.h"

namespace pmsm {

inline Eigen::Vector2d clarke_park(double theta_e, const Eigen::Vector3d& abc) {
    const double alpha = (2.0 * abc[0] - abc[1] - abc[2]) / 3.0;
    const double beta = (abc[1] - abc[2]) / std::sqrt(3.0);
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    return {c * alpha + s * beta, -s * alpha + c * beta};
}

inline Eigen::Vector3d inverse_clarke_park(double theta_e, double d, double q) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    const double alpha = c * d - s * q;
    const double beta = s * d + c * q;
    const double phase_offset = std::sqrt(3.0) / 2.0 * beta;
    return {alpha, -0.5 * alpha + phase_offset, -0.5 * alpha - phase_offset};
}

}  // namespace pmsm
