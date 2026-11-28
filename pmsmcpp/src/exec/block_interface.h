#pragma once

#include <concepts>

#include <Eigen/Core>

namespace pmsm {

/// @brief Concept for a block that has a tick function.
/// @tparam B Model block type, must define a member function tick().
template <typename B>
concept TickBlock = requires { static_cast<void (B::*)()>(&B::tick); };

/// @brief Const view of a block's state vector, used for reading the state without copying or
/// modifying it.
using StateView = Eigen::Ref<const Eigen::VectorXd>;

/// @brief Mutable view of a block's state vector, used for reading and modifying the state without
/// copying it.
using MutableStateView = Eigen::Ref<Eigen::VectorXd>;

/// @brief Concept for a block that has a state vector and can publish its state and compute its
/// derivative.
template <typename B>
concept ContinousBlock = requires {
    requires B::state_size > 0;
} && requires(B& block, const B& const_block, StateView state, MutableStateView state_dot) {
    { block.publish(state) } -> std::same_as<void>;
    { const_block.derivative(state, state_dot) } -> std::same_as<void>;
};
} // namespace pmsm
