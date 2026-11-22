#pragma once

#include <algorithm>
#include <cmath>

#include "graph/block_ports.h"
#include "configuration.h"

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

namespace drive_blocks {

/// Measured mechanical speed for the outer loop, in rad/s.
struct SpeedInputs { InputPort<Speed> omega; };

/// Requested q-axis current from the speed controller, in amperes.
struct SpeedOutputs { Current iq_ref; };

using config_detail::bounded_number;
using config_detail::number;

class SpeedController {
public:
    explicit SpeedController(const BlockSpec& spec)
        : controller_(bounded_number(spec.parameters, "gain_a_per_rad_s", true),
                      bounded_number(spec.parameters, "max_current_a", true)),
          reference_(spec.parameters.at("reference_rad_s").is_null()
              ? bounded_number(spec.parameters, "bus_voltage_v", false) /
                (std::sqrt(3.0) * bounded_number(spec.parameters, "back_emf_v_per_rad_s", false))
              : number(spec.parameters, "reference_rad_s")) {}

    void tick() { outputs.iq_ref = controller_.output(reference_, inputs.omega.read()); }

    SpeedInputs inputs;
    SpeedOutputs outputs;

private:
    SpeedControl controller_;
    double reference_;
};

}  // namespace drive_blocks

}  // namespace pmsm
