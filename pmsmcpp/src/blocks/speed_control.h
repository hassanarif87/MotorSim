#pragma once

#include <algorithm>
#include <cmath>

#include "src/graph/block_ports.h"
#include "src/configuration.h"

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

class SpeedController {
    struct Parameters {
        NonnegativeReal gain_a_per_rad_s;
        NonnegativeReal max_current_a;
        std::optional<FiniteReal> reference_rad_s;
        std::optional<PositiveReal> bus_voltage_v;
        std::optional<PositiveReal> back_emf_v_per_rad_s;
    };

public:
    explicit SpeedController(const BlockSpec& spec)
        : SpeedController(spec.read_parameters<Parameters>()) {}

    void tick() { outputs.iq_ref = controller_.output(reference_, inputs.omega.read()); }

    SpeedInputs inputs;
    SpeedOutputs outputs;

private:
    explicit SpeedController(const Parameters& p)
        : controller_(p.gain_a_per_rad_s.get(), p.max_current_a.get()),
          reference_(speed_reference(p)) {}

    static double speed_reference(const Parameters& p) {
        if (p.reference_rad_s) {
            return p.reference_rad_s->get();
        }
        if (!p.bus_voltage_v || !p.back_emf_v_per_rad_s) {
            throw std::invalid_argument("Automatic speed reference requires bus_voltage_v and back_emf_v_per_rad_s");
        }
        return p.bus_voltage_v->get() / (std::sqrt(3.0) * p.back_emf_v_per_rad_s->get());
    }

    SpeedControl controller_;
    double reference_;
};

}  // namespace drive_blocks

}  // namespace pmsm
