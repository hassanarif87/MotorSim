#pragma once

#include <stdexcept>

#include "blocks/transforms.h"

namespace pmsm {

inline DutyCycles svpwm(DqVoltage dq, double theta_e, double bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0) {
        throw std::invalid_argument("DC bus voltage must be positive and finite");
    }
    const auto abc = inverse_clarke_park(theta_e, dq.d, dq.q);
    const double center = (bus_voltage - abc.minCoeff() - abc.maxCoeff()) / 2.0;
    return {(abc.array() + center) / bus_voltage};
}

}  // namespace pmsm
