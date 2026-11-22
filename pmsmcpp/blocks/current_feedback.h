#pragma once

#include "blocks/transforms.h"

namespace pmsm {

inline DqCurrent current_feedback(PhaseCurrent current, double theta_e) {
    const auto dq = clarke_park(theta_e, current.abc);
    return {dq.x(), dq.y()};
}

}  // namespace pmsm
