#pragma once

#include <cmath>

#include "src/blocks/signals.h"

namespace pmsm {

/// @brief Performs the Clarke-Park transformation on a three-phase signal.
/// @param theta_e The electrical angle in radians.
/// @param abc The three-phase signal as a vector of three values.
/// @return A two-dimensional vector representing the transformed signal in the d-q frame.
inline Eigen::Vector2d clarke_park(double theta_e, const Eigen::Vector3d& abc) {
    const double alpha = (2.0 * abc[0] - abc[1] - abc[2]) / 3.0;
    const double beta = (abc[1] - abc[2]) / std::sqrt(3.0);
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    return {c * alpha + s * beta, -s * alpha + c * beta};
}

/// @brief Performs the inverse Clarke-Park transformation to convert d-q frame signals back to
/// three-phase signals.
/// @param theta_e The electrical angle in radians.
/// @param d The d-axis signal.
/// @param q The q-axis signal.
/// @return A three-dimensional vector representing the converted signal in the three-phase frame.
inline Eigen::Vector3d inverse_clarke_park(double theta_e, double d, double q) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    const double alpha = c * d - s * q;
    const double beta = s * d + c * q;
    const double phase_offset = std::sqrt(3.0) / 2.0 * beta;
    return {alpha, -0.5 * alpha + phase_offset, -0.5 * alpha - phase_offset};
}

} // namespace pmsm
