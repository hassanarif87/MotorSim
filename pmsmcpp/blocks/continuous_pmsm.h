#pragma once

#include "blocks/plant.h"
#include "rk4.h"
#include "simulation_executor.h"

namespace pmsm {

class ContinuousPmsm {
public:
    explicit ContinuousPmsm(PmsmPlant plant, double load_torque, MotorState initial = {})
        : plant_(plant), state_(initial), input_{{Eigen::Vector3d::Zero()}, load_torque},
          output_(plant_.output(state_)) {}

    void set_voltage(const PhaseVoltage& voltage) { input_.voltage = voltage; }
    int pole_pairs() const { return plant_.pole_pairs(); }
    const MotorOutput& output() const { return output_; }

    void advance(SimulationExecutor::Duration interval) {
        const State initial(state_.iq, state_.id, state_.omega, state_.theta);
        const auto derivative = [this](const State& trial) {
            const auto rate = plant_.derivative({trial[0], trial[1], trial[2], trial[3]}, input_);
            return State(rate.iq, rate.id, rate.omega, rate.theta);
        };
        const auto accepted = rk4_step(initial, std::chrono::duration<double>(interval).count(),
                                       derivative);
        state_ = {accepted[0], accepted[1], accepted[2], accepted[3]};
        post_integrate();
    }

private:
    using State = Eigen::Vector4d;

    void post_integrate() { output_ = plant_.output(state_); }

    PmsmPlant plant_;
    MotorState state_;
    MotorInput input_;
    MotorOutput output_;
};

}  // namespace pmsm
