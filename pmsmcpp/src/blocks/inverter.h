#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "src/blocks/signals.h"

#include "src/graph/block_ports.h"
#include "src/exec/configuration.h"

namespace pmsm {

namespace drive_blocks {

/// Normalized phase duty commands consumed by the inverter.
struct DutyInputs {
    InputPort<Duty> duty_a, duty_b, duty_c;
};

/// @brief A simple ideal inverter that converts normalized duty cycles to phase voltages.
class Inverter {

    /// @brief Parameters for the Inverter block.
    struct Parameters {
        /// @brief The DC bus voltage in volts. Must be positive and finite.
        PositiveReal bus_voltage_v;
    };

  public:
    /// @brief Constructs an Inverter block with the given parameters.
    /// @param spec The BlockSpec containing the parameters for the Inverter.
    explicit Inverter(const BlockSpec& spec)
        : bus_voltage_(spec.read_parameters<Parameters>().bus_voltage_v.get()) {}

    static PhaseVoltage ideal_inverter(DutyCycles duty, double bus_voltage) {
        if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0 || !duty.abc.allFinite()) {
            throw std::invalid_argument(
                "Inverter requires finite duties and a positive finite DC bus");
        }
        const Eigen::Vector3d pole = bus_voltage * duty.abc.cwiseMax(0.0).cwiseMin(1.0);
        return {pole.array() - pole.mean()};
    }

    /// @brief Updates the inverter outputs based on the current duty inputs.
    void tick() {
        const auto voltage = ideal_inverter(
            {{inputs.duty_a.read(), inputs.duty_b.read(), inputs.duty_c.read()}}, bus_voltage_);
        outputs = {voltage.abc[0], voltage.abc[1], voltage.abc[2]};
    }

    /// @brief Input ports,  representing the duty cycles for each phase.
    DutyInputs inputs;
    /// @brief Output ports, representing the phase voltages.
    PhaseVoltageOutputs outputs;

  private:
    /// @brief The DC bus voltage in volts.
    double bus_voltage_;
};

/// A PWM-cycle averaged bridge with shadow-register duty updates.
class NonIdealInverter {
    struct Parameters {
        PositiveReal bus_voltage_v;
        NonnegativeReal dead_time_s;
        NonnegativeReal switch_resistance_ohm;
    };

    struct Inputs {
        InputPort<Duty> duty_a, duty_b, duty_c;
        InputPort<Current> ia, ib, ic;
    };

  public:
    explicit NonIdealInverter(const BlockSpec& spec)
        : NonIdealInverter(spec.read_parameters<Parameters>(),
                           std::chrono::duration<double>(spec.period().value()).count()) {}

    /// @brief Updates the inverter outputs based on the current duty inputs and measured currents.
    void tick() {
        const Eigen::Vector3d command(inputs.duty_a.read(), inputs.duty_b.read(),
                                      inputs.duty_c.read());
        if (!command.allFinite()) {
            throw std::invalid_argument("Inverter requires finite duties");
        }

        // Compute the phase voltages based on the pending duty cycle and the measured currents.
        const Eigen::Vector3d pole(phase_pole_voltage(pending_duty_.x(), inputs.ia.read()),
                                   phase_pole_voltage(pending_duty_.y(), inputs.ib.read()),
                                   phase_pole_voltage(pending_duty_.z(), inputs.ic.read()));

        // Center the pole voltages to get the phase voltages.
        const Eigen::Vector3d phase = pole.array() - pole.mean();
        outputs = {phase.x(), phase.y(), phase.z()};

        // Update the pending duty cycle for the next tick, clamped to [0, 1].
        pending_duty_ = command.cwiseMax(0.0).cwiseMin(1.0);
    }

    Inputs inputs;
    PhaseVoltageOutputs outputs;

  private:
    NonIdealInverter(const Parameters& parameters, double period)
        : bus_voltage_(parameters.bus_voltage_v.get()),
          dead_fraction_(parameters.dead_time_s.get() / period),
          switch_resistance_(parameters.switch_resistance_ohm.get()) {
        if (dead_fraction_ >= 0.5) {
            throw std::invalid_argument("Inverter dead time must be less than half the PWM period");
        }
    }

    /// @brief  Calculates the phase pole voltage based on the duty cycle and measured current,
    /// accounting for dead time and switch resistance.
    /// @param duty The normalized duty cycle for the phase, in the range [0, 1].
    /// @param current The measured current for the phase in amperes.
    /// @return The calculated phase pole voltage in volts.
    double phase_pole_voltage(double duty, double current) const {
        // A leg held at either rail has no PWM edges. Positive current flows into the motor.
        if (duty > 0.0 && duty < 1.0) {
            if (current > 0.0) {
                duty = std::max(0.0, duty - dead_fraction_);
            } else if (current < 0.0) {
                duty = std::min(1.0, duty + dead_fraction_);
            }
        }
        return bus_voltage_ * duty - switch_resistance_ * current;
    }

    /// @brief The DC bus voltage in volts, dead time fraction, and switch resistance in ohms.
    double bus_voltage_, dead_fraction_, switch_resistance_;

    /// @brief The pending duty cycle to be applied on the next tick, initialized to 50%.
    Eigen::Vector3d pending_duty_ = Eigen::Vector3d::Constant(0.5);
};

} // namespace drive_blocks

} // namespace pmsm
