#pragma once

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "src/configuration.h"
#include "src/graph/block_ports.h"

namespace pmsm::drive_blocks {

class DeadTimeCompensation {
    struct Parameters {
        NonnegativeReal dead_time_s;
        PositiveReal zero_current_band_a;
    };

    struct Inputs {
        InputPort<Duty> duty_a, duty_b, duty_c;
        InputPort<Current> ia, ib, ic;
    };

public:
    explicit DeadTimeCompensation(const BlockSpec& spec)
        : DeadTimeCompensation(spec.read_parameters<Parameters>(),
                               std::chrono::duration<double>(spec.period().value()).count()) {}

    void tick() {
        outputs = {correct(inputs.duty_a.read(), inputs.ia.read()),
                   correct(inputs.duty_b.read(), inputs.ib.read()),
                   correct(inputs.duty_c.read(), inputs.ic.read())};
    }

    Inputs inputs;
    struct Outputs { Duty duty_a, duty_b, duty_c; } outputs;

private:
    DeadTimeCompensation(const Parameters& parameters, double period)
        : dead_fraction_(parameters.dead_time_s.get() / period),
          zero_current_band_(parameters.zero_current_band_a.get()) {
        if (dead_fraction_ >= 0.5) {
            throw std::invalid_argument("Compensated dead time must be less than half the PWM period");
        }
    }

    double correct(double duty, double current) const {
        // A fully on or fully off leg has no transitions to compensate.
        if (duty <= 0.0 || duty >= 1.0) {
            return std::clamp(duty, 0.0, 1.0);
        }
        const double polarity = std::clamp(current / zero_current_band_, -1.0, 1.0);
        return std::clamp(duty + dead_fraction_ * polarity, 0.0, 1.0);
    }

    double dead_fraction_, zero_current_band_;
};

}  // namespace pmsm::drive_blocks
