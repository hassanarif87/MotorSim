#pragma once

#include <concepts>

#include <Eigen/Core>

namespace pmsm {

/// @brief Concept for a block that has a tick function.
/// @tparam B Model block type, must define a member function tick().
template <typename B>
concept TickBlock = requires { static_cast<void (B::*)()>(&B::tick); };

template <typename B, typename Duration>
concept AdvanceBlock = requires { static_cast<void (B::*)(Duration)>(&B::advance); };

/// @brief Const view of a block's state vector, used for reading the state without copying or
/// modifying it.
/// @tparam B Model block type, must define a static member StateSize.
template <typename B>
using StateView = Eigen::Map<const Eigen::Matrix<double, B::state_size, 1>>;

/// @brief Mutable view of a block's state vector, used for reading and modifying the state without
/// copying it.
/// @tparam B Model block type, must define a static member StateSize.
template <typename B>
using MutableStateView = Eigen::Map<Eigen::Matrix<double, B::state_size, 1>>;

/// @brief Concept for a block that has a state vector and can publish its state and compute its
/// derivative.
template <typename B>
concept ContinousBlock = requires {
    requires B::state_size > 0;
} && requires(B& block, const B& const_block, StateView<B> state, MutableStateView<B> state_dot) {
    { block.publish(state) } -> std::same_as<void>;
    { const_block.derivative(state, state_dot) } -> std::same_as<void>;
};
} // namespace pmsm
