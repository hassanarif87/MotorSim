#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>

#include "src/exec/block_interface.h"
#include "src/exec/delegate.h"
#include "src/exec/integration.h"

namespace pmsm {

/// @brief Executes the simulation by managing the timing and invocation of registered blocks.
class SimulationExecutor {
  public:
    /// @brief Duration type representing simulation time in nanoseconds.
    using Duration = std::chrono::nanoseconds;
    /// @brief Callback type for periodic simulation blocks.
    using TickCallback = Delegate<void()>;

    static constexpr std::size_t max_periodic_blocks = 16;

    explicit SimulationExecutor(Duration tick_interval) : tick_interval_(tick_interval) {
        if (tick_interval_ <= Duration::zero()) {
            throw std::invalid_argument("Simulation tick interval must be positive");
        }
    }

    /// Bind a block's periodic callback directly from its member signature.
    template <TickBlock Block>
        requires(!ContinousBlock<Block>)
    void register_block(Block& block, Duration period, int priority) {
        register_tick(period, priority,
                      TickCallback::bind<static_cast<void (Block::*)()>(&Block::tick)>(block));
    }

    template <ContinousBlock B>
    void register_block(B& block, Eigen::Vector<double, B::state_size> initial_state) {
        if (started_) {
            throw std::logic_error("Register continuous blocks before run");
        }
        integrator_.add(block, initial_state);
    }

    bool has_continuous_block() const { return !integrator_.empty(); }

    /// @brief Registers a periodic block with the simulation executor.
    /// @param period The period of the block, must be a multiple of the tick interval.
    /// @param priority The priority of the block, used for ordering execution.
    /// @param callback The callback function to invoke for the block.
    void register_tick(Duration period, int priority, TickCallback callback) {
        if (started_ || !callback || period <= Duration::zero()) {
            throw std::invalid_argument("Expected a callback and positive period before run");
        }

        // Ensure that the period is a multiple of the tick interval.
        if (period % tick_interval_ != Duration::zero()) {
            throw std::invalid_argument(
                "Periodic block must span a whole number of simulation ticks");
        }

        // Ensure that the number of periodic blocks does not exceed the maximum allowed.
        if (periodic_count_ == periodic_blocks_.size()) {
            throw std::length_error("Too many periodic blocks");
        }

        // Preserve registration order for equal priorities.
        auto index = periodic_count_++;
        while (index > 0 && periodic_blocks_[index - 1].priority > priority) {
            periodic_blocks_[index] = periodic_blocks_[index - 1];
            --index;
        }
        periodic_blocks_[index] = {period / tick_interval_, priority, callback};
    }

    /// @brief Get sim time in nanoseconds.
    /// @return Duration representing the current simulation time.
    Duration now() const { return now_; }

    /// @brief Runs the simulation until the specified end time, invoking registered blocks as
    /// appropriate.
    /// @param end The end time of the simulation. Must be non-negative and greater than the
    /// current time.
    void run(Duration end) {
        if (started_ || !has_continuous_block() || end < Duration::zero()) {
            throw std::logic_error(
                "Expected a single run with continuous blocks and valid end time");
        }

        // Start flag
        started_ = true;
        integrator_.initialize();

        // Compute the index of the last tick to process based on the end time and tick interval.
        const auto last_tick_index = end / tick_interval_;

        // Process each tick up to the last tick index, invoking due periodic blocks and advancing
        // the integrator.
        for (Duration::rep tick_index = 0; tick_index < last_tick_index; ++tick_index) {
            tick_due_blocks(tick_index);
            integrator_.advance(tick_interval_);
            now_ += tick_interval_;
        }

        // Run callbacks at the last tick boundary, even when it is the end time.
        tick_due_blocks(last_tick_index);

        // Handle any remaining time after the last full tick.
        const auto remainder = end % tick_interval_;
        if (remainder != Duration::zero()) {
            integrator_.advance(remainder);
            now_ = end;
        }
    }

  private:
    /// @brief Invokes all periodic blocks that are due at the specified tick index.
    /// @param tick_index Index of the current simulation tick, used to determine which periodic
    /// blocks should be invoked.
    void tick_due_blocks(Duration::rep tick_index) const {
        for (std::size_t i = 0; i < periodic_count_; ++i) {
            const auto& block = periodic_blocks_[i];
            if (tick_index % block.tick_stride == 0) {
                block.tick();
            }
        }
    }

    /// @brief Represents a periodic block in the simulation.
    struct PeriodicBlock {
        /// @brief The number of ticks between invocations of the block's tick callback.
        Duration::rep tick_stride;
        /// @brief The priority of the block, used for ordering execution.
        int priority;
        /// @brief The callback function to invoke for the block.
        TickCallback tick;
    };

    IntegrationExecutor integrator_;

    /// @brief The interval between simulation ticks in nanoseconds.
    Duration tick_interval_;
    /// @brief The current simulation time in nanoseconds.
    Duration now_{0};
    std::array<PeriodicBlock, max_periodic_blocks> periodic_blocks_{};
    std::size_t periodic_count_ = 0;
    bool started_ = false;
};

} // namespace pmsm
