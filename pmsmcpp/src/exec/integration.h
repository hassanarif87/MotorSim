#pragma once

#include <chrono>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "src/exec/block_interface.h"
#include "src/exec/delegate.h"
#include "src/math/rk4.h"

namespace pmsm {
using namespace Eigen;

class IntegrationExecutor {

    using Duration = std::chrono::nanoseconds;
    using Publish = Delegate<void(const double*)>;
    using Derivative = Delegate<void(const double*, double*)>;

  public:
    bool empty() const { return integ_blocks_.empty(); }

    template <ContinousBlock B>
    void add(B& block, Vector<double, B::state_size>& initial_state) {
        if (initialized_) {
            throw std::logic_error("Expected all blocks to be registered before run");
        }

        // Current state size, on add.
        const Index offset = this->state_.size();

        const IntegrableBlock registration{
            offset,
            Publish::bind<&Registration<B>::publish>(block),
            Derivative::bind<&Registration<B>::derivative>(block),
        };

        // Allocate space for both before committing the registration.
        integ_blocks_.reserve(integ_blocks_.size() + 1);
        state_.conservativeResize(offset + B::state_size);
        state_.segment<B::state_size>(offset) = initial_state;
        integ_blocks_.push_back(registration);
    }

    /// @brief Advances the simulation by the specified time interval.
    /// @param interval The time interval to advance the simulation in nanoseconds. Must be
    /// non-negative.
    void advance(std::chrono::nanoseconds interval) {
        // Standalone callers can also advance before initialize().
        if (!initialized_) {
            rk4_.resize_like(state_);
        }
        const double dt = std::chrono::duration<double>(interval).count();

        auto full_derivative = [this](const VectorXd& trial_state, VectorXd& state_dot) {
            publish(trial_state);
            for (const auto& block : integ_blocks_) {
                block.derivative(trial_state.data() + block.offset,
                                 state_dot.data() + block.offset);
            }
        };

        rk4_.step(state_, dt, full_derivative);
        publish(state_);
    }

    void initialize() {
        rk4_.resize_like(state_);
        publish(state_);
        initialized_ = true;
    }

  private:
    /// @brief Publishes the outputs of all registered blocks at the given state.
    /// @param state
    void publish(const VectorXd& state) {
        for (const auto& block : integ_blocks_) {
            block.publish(state.data() + block.offset);
        }
    }
    /// @brief Represents a block that can be integrated, including its state offset and
    /// callbacks for publishing and computing derivatives.
    struct IntegrableBlock {
        Index offset;
        Publish publish;
        Derivative derivative;
    };

    template <ContinousBlock B>
    struct Registration {

        /// @brief Static function to publish a block outputs at the given state.
        static void publish(B& block, const double* state) {
            StateView<B> slice{state};
            block.publish(slice);
        }

        /// @brief Static function to compute the derivative of a block's state.
        static void derivative(const B& block, const double* state, double* state_dot) {
            StateView<B> slice{state};
            MutableStateView<B> state_dot_slice{state_dot};
            block.derivative(slice, state_dot_slice);
        }
    };

    /// @brief The current state vector of the simulation, containing the states of all
    /// registered
    Eigen::VectorXd state_;
    Rk4<Eigen::VectorXd> rk4_;

    /// @brief List of registered integrable blocks, each with its state offset and callbacks.
    std::vector<IntegrableBlock> integ_blocks_;

    /// @brief List of registered integrable blocks, each with its state offset and callbacks.
    bool initialized_ = false;
};
} // namespace pmsm
