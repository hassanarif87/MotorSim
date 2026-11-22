#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace pmsm {

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
    using UpdateCallback = Delegate<void()>;

    static constexpr std::size_t max_updates = 16;

    explicit SimulationExecutor(Duration max_step) : max_step_(max_step) {
        if (max_step_ <= Duration::zero()) {
            throw std::invalid_argument("Maximum integration step must be positive");
        }
    }

    void register_continuous(AdvanceCallback callback) {
        if (started_ || advance_ || !callback) {
            throw std::logic_error("Expected one continuous registration before run");
        }
        advance_ = callback;
    }

    void register_update(Duration period, int priority, UpdateCallback callback) {
        if (started_ || !callback || period <= Duration::zero()) {
            throw std::invalid_argument("Expected a callback and positive period before run");
        }
        if (update_count_ == updates_.size()) {
            throw std::length_error("Too many periodic updates");
        }

        // Preserve registration order for equal priorities.
        auto index = update_count_++;
        while (index > 0 && updates_[index - 1].priority > priority) {
            updates_[index] = updates_[index - 1];
            --index;
        }
        updates_[index] = {period, priority, callback, Duration::zero(), true};
    }

    Duration now() const { return now_; }

    void run(Duration end) {
        if (started_ || !advance_ || end < Duration::zero()) {
            throw std::logic_error("Expected a single run with a continuous callback and valid end time");
        }
        started_ = true;
        for (;;) {
            auto next = now_ + std::min(max_step_, end - now_);
            for (std::size_t i = 0; i < update_count_; ++i) {
                auto& update = updates_[i];
                if (update.pending && update.next == now_) {
                    update.callback();
                    update.pending = update.period <= end - now_;
                    if (update.pending) {
                        update.next += update.period;
                    }
                }
                if (update.pending) {
                    next = std::min(next, update.next);
                }
            }
            if (now_ == end) {
                return;
            }
            advance_(next - now_);
            now_ = next;
        }
    }

private:
    struct Update {
        Duration period;
        int priority;
        UpdateCallback callback;
        Duration next;
        bool pending;
    };

    Duration max_step_;
    Duration now_{0};
    AdvanceCallback advance_;
    std::array<Update, max_updates> updates_{};
    std::size_t update_count_ = 0;
    bool started_ = false;
};

}  // namespace pmsm
