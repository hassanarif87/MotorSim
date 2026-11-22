#pragma once

#include <algorithm>

namespace pmsm {

// Proportional speed controller used in the Python speed-control notebook.
class SpeedControl {
public:
    SpeedControl(double gain, double max_current) : gain_(gain), max_current_(max_current) {}

    double output(double reference, double measured) const {
        return std::clamp(gain_ * (reference - measured), 0.0, max_current_);
    }

private:
    double gain_;
    double max_current_;
};

}  // namespace pmsm
