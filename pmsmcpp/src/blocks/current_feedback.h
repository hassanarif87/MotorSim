#pragma once

#include "src/blocks/transforms.h"

#include "src/graph/block_ports.h"
#include "src/exec/configuration.h"

namespace pmsm {

inline DqCurrent current_feedback(PhaseCurrent current, double theta_e) {
    const auto dq = clarke_park(theta_e, current.abc);
    return {dq.x(), dq.y()};
}

namespace drive_blocks {

/// Phase currents (A) and mechanical position (rad) sampled for the dq transform.
struct FeedbackInputs {
    InputPort<Current> ia, ib, ic;
    InputPort<Position> theta;
};

/// Sampled d- and q-axis currents, in amperes.
struct FeedbackOutputs {
    Current id, iq;
};

class Feedback {
    struct Parameters {
        PositiveInt pole_pairs;
    };

  public:
    explicit Feedback(const BlockSpec& spec)
        : pole_pairs_(spec.read_parameters<Parameters>().pole_pairs.get()) {}

    void tick() {
        const auto measured =
            current_feedback({{inputs.ia.read(), inputs.ib.read(), inputs.ic.read()}},
                             pole_pairs_ * inputs.theta.read());
        outputs = {measured.d, measured.q};
    }

    FeedbackInputs inputs;
    FeedbackOutputs outputs;

  private:
    int pole_pairs_;
};

} // namespace drive_blocks

} // namespace pmsm
