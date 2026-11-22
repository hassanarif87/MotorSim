#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace pmsm {

template <typename Block>
concept TickBlock = requires {
    static_cast<void (Block::*)()>(&Block::tick);
};

template <typename Block, typename Duration>
concept AdvanceBlock = requires {
    static_cast<void (Block::*)(Duration)>(&Block::advance);
};

template <typename Signature>
class Delegate;

template <typename... Args>
class Delegate<void(Args...)> {
public:
    template <auto Method, typename Object>
    static Delegate bind(Object& object) {
        return {&object, [](void* context, Args... args) {
            (static_cast<Object*>(context)->*Method)(std::forward<Args>(args)...);
        }};
    }

    explicit operator bool() const { return invoke_ != nullptr; }
    void operator()(Args... args) const { invoke_(context_, std::forward<Args>(args)...); }

    Delegate() = default;

private:
    Delegate(void* context, void (*invoke)(void*, Args...))
        : context_(context), invoke_(invoke) {}

    void* context_ = nullptr;
    void (*invoke_)(void*, Args...) = nullptr;
};

class SimulationExecutor {
public:
    using Duration = std::chrono::nanoseconds;
    using AdvanceCallback = Delegate<void(Duration)>;
    using TickCallback = Delegate<void()>;

    static constexpr std::size_t max_periodic_blocks = 16;

    explicit SimulationExecutor(Duration tick_interval) : tick_interval_(tick_interval) {
        if (tick_interval_ <= Duration::zero()) {
            throw std::invalid_argument("Simulation tick interval must be positive");
        }
    }

    /// Bind a block's periodic callback directly from its member signature.
    template <TickBlock Block>
        requires (!AdvanceBlock<Block, Duration>)
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
        advance_ = AdvanceCallback::bind<static_cast<void (Block::*)(Duration)>(&Block::advance)>(block);
    }

    bool has_continuous_block() const { return static_cast<bool>(advance_); }

    void register_tick(Duration period, int priority, TickCallback callback) {
        if (started_ || !callback || period <= Duration::zero()) {
            throw std::invalid_argument("Expected a callback and positive period before run");
        }
        if (period % tick_interval_ != Duration::zero()) {
            throw std::invalid_argument("Periodic block must span a whole number of simulation ticks");
        }
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

    Duration now() const { return now_; }

    void run(Duration end) {
        if (started_ || !advance_ || end < Duration::zero()) {
            throw std::logic_error("Expected a single run with a continuous callback and valid end time");
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
    void tick_due_blocks(Duration::rep tick_index) const {
        for (std::size_t i = 0; i < periodic_count_; ++i) {
            const auto& block = periodic_blocks_[i];
            if (tick_index % block.tick_stride == 0) {
                block.tick();
            }
        }
    }

    struct PeriodicBlock {
        Duration::rep tick_stride;
        int priority;
        TickCallback tick;
    };

    Duration tick_interval_;
    Duration now_{0};
    AdvanceCallback advance_;
    std::array<PeriodicBlock, max_periodic_blocks> periodic_blocks_{};
    std::size_t periodic_count_ = 0;
    bool started_ = false;
};

}  // namespace pmsm
