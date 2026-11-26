#pragma once

#include <algorithm>
#include <chrono>
#include <optional>

#include "src/configuration.h"
#include "src/graph/block_ports.h"

namespace pmsm::drive_blocks {

// Fixed-period forms of the SimpleFOC v2.3.5 PID and low-pass filter.
struct SimpleFocPidSettings {
    FiniteReal P{0.0}, I{0.0}, D{0.0};
    NonnegativeReal output_ramp{0.0};
};

class SimpleFocPid {
  public:
    SimpleFocPid(SimpleFocPidSettings settings, double limit, double period)
        : settings_(settings), limit_(limit), period_(period) {}

    double update(double error, double feedforward = 0.0) {
        integral_ += (error + last_error_) * (settings_.I.get() * period_ / 2);
        integral_ = std::clamp(integral_, -limit_, limit_);
        const double derivative = (error - last_error_) * settings_.D.get() / period_;
        const double requested = settings_.P.get() * error + integral_ + derivative + feedforward;
        const double limited = std::clamp(requested, -limit_, limit_);
        const double max_change = settings_.output_ramp.get() * period_;
        last_output_ = max_change > 0 ? std::clamp(limited, last_output_ - max_change,
                                                   last_output_ + max_change)
                                      : limited;
        last_error_ = error;
        return last_output_;
    }

  private:
    SimpleFocPidSettings settings_;
    double limit_, period_;
    double integral_ = 0, last_error_ = 0, last_output_ = 0;
};

class SimpleFocFilter {
  public:
    SimpleFocFilter(double time_constant, double period)
        : weight_(period / (time_constant + period)) {}

    double update(double measurement) {
        value_ += weight_ * (measurement - value_);
        return value_;
    }

  private:
    double weight_;
    double value_ = 0;
};

class SimpleFocVelocity {
    struct Parameters {
        SimpleFocPidSettings PID_velocity;
        NonnegativeReal LPF_velocity_Tf;
        PositiveReal current_limit;
        FiniteReal target_rad_s;
    };

  public:
    explicit SimpleFocVelocity(const BlockSpec& spec)
        : SimpleFocVelocity(spec.read_parameters<Parameters>(),
                            std::chrono::duration<double>(spec.period().value()).count()) {}

    void tick() {
        outputs.omega_filtered = filter_.update(inputs.omega.read());
        outputs.iq_ref = pid_.update(outputs.omega_ref - outputs.omega_filtered);
    }

    struct Inputs {
        InputPort<Speed> omega;
    } inputs;
    struct Outputs {
        Speed omega_ref, omega_filtered;
        Current iq_ref;
    } outputs;

  private:
    SimpleFocVelocity(const Parameters& p, double period)
        : outputs{p.target_rad_s.get(), 0, 0}, pid_(p.PID_velocity, p.current_limit.get(), period),
          filter_(p.LPF_velocity_Tf.get(), period) {}

    SimpleFocPid pid_;
    SimpleFocFilter filter_;
};

class SimpleFocCurrent {
    struct Parameters {
        SimpleFocPidSettings PID_current_d, PID_current_q;
        NonnegativeReal LPF_current_d_Tf, LPF_current_q_Tf;
        PositiveReal voltage_limit;
        std::optional<bool> feedforward_enabled;
        PositiveInt pole_pairs;
        PositiveReal inductance_h;
        PositiveReal back_emf_v_per_rad_s;
    };

  public:
    explicit SimpleFocCurrent(const BlockSpec& spec)
        : SimpleFocCurrent(spec.read_parameters<Parameters>(),
                           std::chrono::duration<double>(spec.period().value()).count()) {}

    void tick() {
        const double id = inputs.id.read();
        const double iq = inputs.iq.read();
        const double omega = feedforward_enabled_ ? static_cast<double>(inputs.omega.read()) : 0.0;
        const double coupling = electrical_inductance_ * omega;
        outputs.id_filtered = filter_d_.update(id);
        outputs.iq_filtered = filter_q_.update(iq);
        outputs.iq_applied = inputs.iq_ref.read();
        outputs.vd = pid_d_.update(outputs.id_ref - outputs.id_filtered, -coupling * iq);
        outputs.vq = pid_q_.update(outputs.iq_applied - outputs.iq_filtered,
                                   coupling * id + back_emf_constant_ * omega);
    }

    struct Inputs {
        InputPort<Current> iq_ref, id, iq;
        InputPort<Speed> omega;
    } inputs;
    struct Outputs {
        Current id_filtered, iq_filtered, iq_applied;
        const Current id_ref{0.0};
        Voltage vd, vq;
    } outputs;

  private:
    SimpleFocCurrent(const Parameters& p, double period)
        : pid_d_(p.PID_current_d, p.voltage_limit.get(), period),
          pid_q_(p.PID_current_q, p.voltage_limit.get(), period),
          filter_d_(p.LPF_current_d_Tf.get(), period), filter_q_(p.LPF_current_q_Tf.get(), period),
          feedforward_enabled_(p.feedforward_enabled.value_or(true)),
          electrical_inductance_(p.pole_pairs.get() * p.inductance_h.get()),
          back_emf_constant_(p.back_emf_v_per_rad_s.get()) {}

    SimpleFocPid pid_d_, pid_q_;
    SimpleFocFilter filter_d_, filter_q_;
    bool feedforward_enabled_;
    double electrical_inductance_, back_emf_constant_;
};

} // namespace pmsm::drive_blocks
