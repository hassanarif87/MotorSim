#pragma once

namespace pmsm {

/// @brief  Performs a single step of the fourth-order Runge-Kutta (RK4) integration method for
/// solving ordinary differential equations (ODEs).
/// @tparam State, a type representing the state of the system.
/// @tparam Derivative, a callable type that takes a State and returns its derivative.
/// @param state The current state of the system.
/// @param dt The time step for the integration.
/// @param derivative A callable that computes the derivative of the state given the current state.
/// @return The propagated state of the system after advancing by dt.
template <typename State, typename Derivative>
State rk4_step(const State& state, double dt, const Derivative& derivative) {
    const State k1 = derivative(state);
    const State k2 = derivative(state + dt / 2.0 * k1);
    const State k3 = derivative(state + dt / 2.0 * k2);
    const State k4 = derivative(state + dt * k3);
    return state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

} // namespace pmsm
