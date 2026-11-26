#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace pmsm {

template <typename Block>
concept TickBlock = requires { static_cast<void (Block::*)()>(&Block::tick); };

template <typename Block, typename Duration>
concept AdvanceBlock = requires { static_cast<void (Block::*)(Duration)>(&Block::advance); };


template <typename Signature>
class Delegate;

/// @brief Delegate class for binding member functions to callbacks.
/// @tparam ...Args The argument types of the bound member function.
template <typename... Args>
class Delegate<void(Args...)> {
  public:
    /// @brief Binds a member function to the delegate.
    /// @tparam Object The type of the object whose member function is being bound.
    /// @tparam Method The member function pointer to bind.
    /// @param object The object whose member function is being bound.
    /// @return Bound delegate that can be invoked with the specified arguments.
    template <auto Method, typename Object>
    static Delegate bind(Object& object) {
        return {&object, [](void* context, Args... args) {
                    (static_cast<Object*>(context)->*Method)(std::forward<Args>(args)...);
                }};
    }

    /// @brief Checks if the delegate is valid (i.e., has a bound function).
    explicit operator bool() const { return invoke_ != nullptr; }
    /// @brief Invokes the bound member function with the specified arguments.
    /// @param ...args The arguments to pass to the bound member function.
    void operator()(Args... args) const { invoke_(context_, std::forward<Args>(args)...); }

    Delegate() = default;

  private:
    /// @brief Constructs a delegate with the given context and invocation function.
    /// @tparam ...Args The argument types of the bound member function.
    /// @param context The context (object) on which the member function will be invoked.
    /// @param invoke The invocation function.
    Delegate(void* context, void (*invoke)(void*, Args...)) : context_(context), invoke_(invoke) {}

    void* context_ = nullptr;
    void (*invoke_)(void*, Args...) = nullptr;
};

/// @brief Executes the simulation by managing the timing and invocation of registered blocks.
class SimulationExecutor {
  public:
    /// @brief Duration type representing simulation time in nanoseconds.
    using Duration = std::chrono::nanoseconds;
    /// @brief Callback type for continuous simulation blocks.
    using AdvanceCallback = Delegate<void(Duration)>;
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
        requires(!AdvanceBlock<Block, Duration>)
    void register_block(Block& block, Duration period, int priority) {
        register_tick(period, priority,
                      TickCallback::bind<static_cast<void (Block::*)()>(&Block::tick)>(block));
    }

    /// Bind a block's continuous callback directly from its member signature.
    template <typename Block>
        requires AdvanceBlock<Block, Duration> && (!TickBlock<Block>)
    void register_block(Block& block) {
        if (started_ || advance_) {
            throw std::logic_error("Expected one continuous block registered before run");
        }
        advance_ =
            AdvanceCallback::bind<static_cast<void (Block::*)(Duration)>(&Block::advance)>(block);
    }

    bool has_continuous_block() const { return static_cast<bool>(advance_); }

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
    /// @param end The end time of the simulation. Must be non-negative and greater than the current
    /// time.
    void run(Duration end) {
        if (started_ || !advance_ || end < Duration::zero()) {
            throw std::logic_error(
                "Expected a single run with a continuous callback and valid end time");
        }
        started_ = true;
        const auto last_tick_index = end / tick_interval_;
        for (Duration::rep tick_index = 0; tick_index < last_tick_index; ++tick_index) {
            tick_due_blocks(tick_index);
            advance_(tick_interval_);
            now_ += tick_interval_;
        }
        // Run callbacks at the last tick boundary, even when it is the end time.
        tick_due_blocks(last_tick_index);

        const auto remainder = end % tick_interval_;
        if (remainder != Duration::zero()) {
            advance_(remainder);
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

    /// @brief The interval between simulation ticks in nanoseconds.
    Duration tick_interval_;
    /// @brief The current simulation time in nanoseconds.
    Duration now_{0};
    AdvanceCallback advance_;
    std::array<PeriodicBlock, max_periodic_blocks> periodic_blocks_{};
    std::size_t periodic_count_ = 0;
    bool started_ = false;
};

} // namespace pmsm
