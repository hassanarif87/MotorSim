#pragma once

#include <cmath>
#include <stdexcept>

#include "blocks/signals.h"

namespace pmsm {

inline PhaseVoltage ideal_inverter(DutyCycles duty, double bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0 || !duty.abc.allFinite()) {
        throw std::invalid_argument("Inverter requires finite duties and a positive finite DC bus");
    }
    const Eigen::Vector3d pole = bus_voltage * duty.abc.cwiseMax(0.0).cwiseMin(1.0);
    return {pole.array() - pole.mean()};
}

}  // namespace pmsm
