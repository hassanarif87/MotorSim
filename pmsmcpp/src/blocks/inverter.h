#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "src/blocks/signals.h"

#include "src/graph/block_ports.h"
#include "src/configuration.h"

namespace pmsm {

inline PhaseVoltage ideal_inverter(DutyCycles duty, double bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0 || !duty.abc.allFinite()) {
        throw std::invalid_argument("Inverter requires finite duties and a positive finite DC bus");
    }
    const Eigen::Vector3d pole = bus_voltage * duty.abc.cwiseMax(0.0).cwiseMin(1.0);
    return {pole.array() - pole.mean()};
}

namespace drive_blocks {

/// Normalized phase duty commands consumed by the inverter.
struct DutyInputs { InputPort<Duty> duty_a, duty_b, duty_c; };

/// Motor-neutral phase voltages published by the averaged inverter, in volts.
struct PhaseVoltageOutputs { Voltage va, vb, vc; };

class Inverter {
    struct Parameters { PositiveReal bus_voltage_v; };

public:
    explicit Inverter(const BlockSpec& spec)
        : bus_voltage_(spec.read_parameters<Parameters>().bus_voltage_v.get()) {}

    void tick() {
        const auto voltage = ideal_inverter(
            {{inputs.duty_a.read(), inputs.duty_b.read(), inputs.duty_c.read()}}, bus_voltage_);
        outputs = {voltage.abc[0], voltage.abc[1], voltage.abc[2]};
    }

    DutyInputs inputs;
    PhaseVoltageOutputs outputs;

private:
    double bus_voltage_;
};

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

    void tick() {
        const Eigen::Vector3d command(inputs.duty_a.read(), inputs.duty_b.read(),
                                      inputs.duty_c.read());
        if (!command.allFinite()) {
            throw std::invalid_argument("Inverter requires finite duties");
        }

        const Eigen::Vector3d pole(
            phase_pole_voltage(pending_duty_.x(), inputs.ia.read()),
            phase_pole_voltage(pending_duty_.y(), inputs.ib.read()),
            phase_pole_voltage(pending_duty_.z(), inputs.ic.read()));
        const Eigen::Vector3d phase = pole.array() - pole.mean();
        outputs = {phase.x(), phase.y(), phase.z()};
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

    double bus_voltage_, dead_fraction_, switch_resistance_;
    Eigen::Vector3d pending_duty_ = Eigen::Vector3d::Constant(0.5);
};

}  // namespace drive_blocks

}  // namespace pmsm
