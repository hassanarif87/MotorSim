#pragma once

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "src/configuration.h"
#include "src/graph/block_ports.h"

namespace pmsm::drive_blocks {

/// @brief A block that compensates for dead time in PWM signals based on the measured phase
/// currents. This is done by adjusting the duty cycles to account for the dead time, with a
/// correction that is proportional to the measured current and limited by a specified zero-current
/// band.
class DeadTimeCompensation {
    struct Parameters {
        /// @brief The dead time in seconds. Must less than half the PWM period.
        NonnegativeReal dead_time_s;
        /// @brief The zero-current band in amperes. This defines the
        /// range of current values over which the dead time compensation is applied. Currents
        /// within this band will have dead time compensation applied.
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

    /// @brief Updates the output duty cycles based on the current input duty cycles and measured
    /// phase currents.
    void tick() {
        outputs = {correct(inputs.duty_a.read(), inputs.ia.read()),
                   correct(inputs.duty_b.read(), inputs.ib.read()),
                   correct(inputs.duty_c.read(), inputs.ic.read())};
    }

    Inputs inputs;
    struct Outputs {
        Duty duty_a, duty_b, duty_c;
    } outputs;

  private:
    DeadTimeCompensation(const Parameters& parameters, double period)
        : dead_fraction_(parameters.dead_time_s.get() / period),
          zero_current_band_(parameters.zero_current_band_a.get()) {
        if (dead_fraction_ >= 0.5) {
            throw std::invalid_argument(
                "Compensated dead time must be less than half the PWM period");
        }
    }

    /// @brief Corrects the duty cycle based on the measured current and dead time.
    /// @param duty The normalized duty cycle for the phase, in the range [0, 1].
    /// @param current The measured current for the phase in amperes.
    /// @return The corrected duty cycle.
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

} // namespace pmsm::drive_blocks
