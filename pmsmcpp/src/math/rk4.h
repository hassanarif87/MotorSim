#pragma once

#include <array>

namespace pmsm {

/// Fourth-order Runge–Kutta stepper with reusable trial and rate buffers.
template <typename State>
class Rk4 {
  public:
    void resize_like(const State& state) {
        for (auto& rate : rates_) {
            rate.resize(state.size());
        }
        trial_.resize(state.size());
    }

    /// Advance the state in place. The derivative fills the supplied rate buffer.
    /// Call resize_like() before stepping a state of a new size.
    template <typename Derivative>
    void step(State& state, double dt, const Derivative& derivative) {
        auto& [k1, k2, k3, k4] = rates_;
        derivative(state, k1);
        trial_.noalias() = state + dt / 2.0 * k1;
        derivative(trial_, k2);
        trial_.noalias() = state + dt / 2.0 * k2;
        derivative(trial_, k3);
        trial_.noalias() = state + dt * k3;
        derivative(trial_, k4);
        state = state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

  private:
    State trial_;
    std::array<State, 4> rates_;
};

} // namespace pmsm
