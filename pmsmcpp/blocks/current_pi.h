#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "blocks/signals.h"

#include "graph/block_ports.h"
#include "configuration.h"

namespace pmsm {

struct CurrentPiParameters {
    NonnegativeReal kp;
    NonnegativeReal ki;
    NonnegativeReal kaw;
    PositiveReal inductance_h{0.03};
    PositiveInt pole_pairs{1};
    PositiveReal back_emf_v_per_rad_s{0.1};
    PositiveReal bus_voltage_v{24.0};
    FiniteReal id_reference_a{0.0};
};

struct CurrentPiInput {
    DqCurrent reference;
    DqCurrent measured;
    double omega;
    double bus_voltage;
};

struct CurrentPiEvaluation {
    DqVoltage voltage;
    Eigen::Vector2d integral_derivative;
};

class CurrentPi {
public:
    explicit CurrentPi(CurrentPiParameters parameters) : parameters_(parameters) {}

    CurrentPiEvaluation evaluate(const Eigen::Vector2d& integral,
                                 const CurrentPiInput& input) const {
        const auto& p = parameters_;
        const Eigen::Vector2d error(input.reference.d - input.measured.d,
                                    input.reference.q - input.measured.q);
        const double coupling = p.pole_pairs.get() * input.omega * p.inductance_h.get();
        const Eigen::Vector2d feedforward(-coupling * input.measured.q,
            coupling * input.measured.d + p.back_emf_v_per_rad_s.get() * input.omega);
        const Eigen::Vector2d raw = p.kp.get() * error + integral + feedforward;
        const double limit = input.bus_voltage / std::sqrt(3.0);
        const Eigen::Vector2d voltage = raw * std::min(1.0, limit / std::max(raw.norm(), 1e-12));
        return {{voltage.x(), voltage.y()}, p.ki.get() * error + p.kaw.get() * (voltage - raw)};
    }

private:
    CurrentPiParameters parameters_;
};

class DigitalCurrentPi {
public:
    DigitalCurrentPi(CurrentPi controller, double sample_period_s)
        : controller_(controller), sample_period_s_(sample_period_s) {}

    DqVoltage tick(const CurrentPiInput& input) {
        const auto evaluation = controller_.evaluate(integral_, input);
        integral_ += sample_period_s_ * evaluation.integral_derivative;
        return evaluation.voltage;
    }

private:
    CurrentPi controller_;
    double sample_period_s_;
    Eigen::Vector2d integral_ = Eigen::Vector2d::Zero();
};

namespace drive_blocks {

/// q-axis reference and measured dq currents (A), plus mechanical speed (rad/s).
struct PiInputs {
    InputPort<Current> iq_ref, id, iq;
    InputPort<Speed> omega;
};

/// Voltage-limited dq commands held between current-loop ticks, in volts.
struct PiOutputs { Voltage vd, vq; Current id_ref; };

class CurrentController {
public:
    explicit CurrentController(const BlockSpec& spec)
        : CurrentController(spec.read_parameters<CurrentPiParameters>(),
                            std::chrono::duration<double>(spec.period().value()).count()) {}

    void tick() {
        const auto command = controller_.tick({
            {outputs.id_ref, inputs.iq_ref.read()}, {inputs.id.read(), inputs.iq.read()},
            inputs.omega.read(), bus_voltage_,
        });
        outputs.vd = command.d;
        outputs.vq = command.q;
    }

    PiInputs inputs;
    PiOutputs outputs;

private:
    CurrentController(const CurrentPiParameters& parameters, double period_s)
        : controller_(CurrentPi(parameters), period_s),
          bus_voltage_(parameters.bus_voltage_v.get()) {
        outputs.id_ref = parameters.id_reference_a.get();
    }

    DigitalCurrentPi controller_;
    double bus_voltage_;
};

}  // namespace drive_blocks

}  // namespace pmsm
