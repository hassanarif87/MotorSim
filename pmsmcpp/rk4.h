#pragma once

namespace pmsm {

template <typename State, typename Derivative>
State rk4_step(const State& state, double dt, const Derivative& derivative) {
    const State k1 = derivative(state);
    const State k2 = derivative(state + dt / 2.0 * k1);
    const State k3 = derivative(state + dt / 2.0 * k2);
    const State k4 = derivative(state + dt * k3);
    return state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace pmsm
