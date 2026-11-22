#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Core>

#include "configuration.h"
#include "graph/block_ports.h"

namespace pmsm::drive_blocks {

class CurrentController {
    struct Parameters {
        NonnegativeReal kp;
        NonnegativeReal ki;
        NonnegativeReal kaw;
        PositiveReal inductance_h{0.03};
        PositiveInt pole_pairs{1};
        PositiveReal back_emf_v_per_rad_s{0.1};
        PositiveReal bus_voltage_v{24.0};
        FiniteReal id_reference_a{0.0};
    };

public:
    explicit CurrentController(const BlockSpec& spec)
        : parameters_(spec.read_parameters<Parameters>()),
          sample_period_s_(std::chrono::duration<double>(spec.period().value()).count()) {
        outputs.id_ref = parameters_.id_reference_a.get();
    }

    void tick() {
        const auto& p = parameters_;
        const double id = inputs.id.read();
        const double iq = inputs.iq.read();
        const double omega = inputs.omega.read();
        const Eigen::Vector2d error(outputs.id_ref - id, inputs.iq_ref.read() - iq);
        const double coupling = p.pole_pairs.get() * omega * p.inductance_h.get();
        const Eigen::Vector2d feedforward(-coupling * iq,
            coupling * id + p.back_emf_v_per_rad_s.get() * omega);
        const Eigen::Vector2d raw = p.kp.get() * error + integral_ + feedforward;
        const double limit = p.bus_voltage_v.get() / std::sqrt(3.0);
        const Eigen::Vector2d voltage = raw * std::min(1.0, limit / std::max(raw.norm(), 1e-12));
        const Eigen::Vector2d integral_derivative =
            p.ki.get() * error + p.kaw.get() * (voltage - raw);
        integral_ += sample_period_s_ * integral_derivative;
        outputs.vd = voltage.x();
        outputs.vq = voltage.y();
    }

    /// q-axis reference and measured dq currents (A), plus mechanical speed (rad/s).
    struct Inputs {
        InputPort<Current> iq_ref, id, iq;
        InputPort<Speed> omega;
    } inputs;

    /// Voltage-limited dq commands (V) and the d-axis current reference (A).
    struct Outputs {
        Voltage vd, vq;
        Current id_ref;
    } outputs;

private:
    Parameters parameters_;
    double sample_period_s_;
    Eigen::Vector2d integral_ = Eigen::Vector2d::Zero();
};

}  // namespace pmsm::drive_blocks
