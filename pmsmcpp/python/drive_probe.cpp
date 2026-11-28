#include "python/drive_probe.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <meta>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

#include "src/blocks/constant_current.h"
#include "src/blocks/constant_torque.h"
#include "src/blocks/simplefoc.h"
#include "src/drive/drive.h"

namespace pmsm {
namespace {

using Duration = SimulationExecutor::Duration;

struct WritablePort {
    std::string_view block_type;
    std::string_view member_name;
    void (*write)(const void*, double);
};

template <typename Block, std::meta::info Member>
consteval WritablePort writable_output() {
    using Outputs = decltype(std::declval<Block&>().outputs);
    using Signal = [:std::meta::type_of(Member):];
    using Owner = [:std::meta::parent_of(Member):];
    static_assert(std::is_same_v<Outputs, Owner>);
    static_assert(!std::is_const_v<Signal>);

    return {
        std::meta::identifier_of(std::meta::dealias(^^Block)),
        std::meta::identifier_of(Member),
        [](const void* address, double value) {
            // Drive exposes this mutable output through a read-only, type-erased address.
            auto* signal = const_cast<Signal*>(static_cast<const Signal*>(address));
            signal->value = value;
        },
    };
}

using namespace drive_blocks;
constexpr std::array writable_ports{
    writable_output<ConstantCurrent, ^^ConstantCurrentOutputs::value>(),
    writable_output<ConstantTorque, ^^ConstantTorqueOutputs::value>(),
    writable_output<SimpleFocVelocity, ^^SimpleFocVelocity::Outputs::omega_ref>(),
};

struct ScheduledChange {
    Duration time;
    OutputReader source;
    void (*write)(const void*, double);
    double value;
};

class Sampler {
  public:
    Sampler(Drive& drive, std::vector<ScheduledChange> changes, std::size_t output_count,
            Duration start, Duration period, std::size_t samples)
        : drive_(drive), changes_(std::move(changes)), output_count_(output_count),
          start_(start), period_(period) {
        if (samples > trace_.time.max_size() || samples > trace_.values.max_size() / output_count) {
            throw std::invalid_argument("Requested trace is too large");
        }
        trace_.time.reserve(samples);
        trace_.values.reserve(samples * output_count);
    }

    void apply() {
        const auto now = drive_.executor().now();
        while (next_ < changes_.size() && changes_[next_].time == now) {
            const auto& change = changes_[next_++];
            change.write(change.source.address, change.value);
        }
    }

    void record() {
        const auto now = drive_.executor().now();
        if (now < start_ || (now - start_) % period_ != Duration::zero()) {
            return;
        }
        trace_.time.push_back(std::chrono::duration<double>(now).count());
        for (std::size_t i = 0; i < output_count_; ++i) {
            trace_.values.push_back(drive_.columns()[i].source());
        }
    }

    DriveProbe::Trace trace() && { return std::move(trace_); }

  private:
    Drive& drive_;
    std::vector<ScheduledChange> changes_;
    std::size_t output_count_, next_ = 0;
    Duration start_, period_;
    DriveProbe::Trace trace_;
};

} // namespace

DriveProbe::DriveProbe(const std::filesystem::path& path)
    : config_(load_configuration(path)), period_(config_.simulation.tick_interval) {
    std::set<std::string> outputs;
    for (const auto& block : config_.blocks) {
        for (const auto& writable : writable_ports) {
            if (block.type == writable.block_type) {
                auto port = (block.name + ".").append(writable.member_name);
                writers_.emplace(port, writable.write);
                outputs.insert(port);
                input_ports_.push_back(std::move(port));
            }
        }
        if (const auto period = block.period()) {
            // All configured periods are integral divisors of one second.
            period_ = Duration(std::lcm(period_.count(), period->count()));
        }
    }
    for (const auto& [label, port] : config_.log_columns) {
        outputs.insert(port);
    }
    // Drive validates the configured log columns. The wrapper exposes their
    // port names and the source ports it can write.
    const Drive validation(config_);
    output_ports_.assign(outputs.begin(), outputs.end());
}

DriveProbe::Tick DriveProbe::sample_ticks() const {
    return period_ / config_.simulation.tick_interval;
}

double DriveProbe::tick_time() const {
    return std::chrono::duration<double>(config_.simulation.tick_interval).count();
}

double DriveProbe::sample_time() const {
    return std::chrono::duration<double>(period_).count();
}

DriveProbe::Trace DriveProbe::run(const std::vector<std::string>& outputs, Tick start_tick,
                                Tick samples, Tick sample_every,
                                const std::vector<Change>& changes) const {
    if (outputs.empty() || start_tick < 0 || samples < 1 || sample_every < 1) {
        throw std::invalid_argument("Expected output ports, nonnegative start and positive sampling");
    }
    const auto tick = config_.simulation.tick_interval;
    const auto max_ticks = Duration::max() / tick;
    if (start_tick > max_ticks || sample_every > max_ticks ||
        samples - 1 > (max_ticks - start_tick) / sample_every) {
        throw std::invalid_argument("Sampling duration is out of range");
    }
    const auto end_tick = start_tick + (samples - 1) * sample_every;

    auto config = config_;
    config.log_columns.clear();
    for (const auto& output : outputs) {
        config.log_columns.push_back({"probe_" + std::to_string(config.log_columns.size()), output});
    }

    std::unordered_map<std::string, std::size_t> source_columns;
    for (const auto& [at, port, value] : changes) {
        if (at < 0 || at > end_tick || !std::isfinite(value)) {
            throw std::invalid_argument("Input changes require a finite value and a tick in the run");
        }
        if (!writers_.contains(port)) {
            throw std::invalid_argument("Unknown writable input port: " + port);
        }
        const auto index = config.log_columns.size();
        if (source_columns.try_emplace(port, index).second) {
            config.log_columns.push_back({"probe_" + std::to_string(index), port});
        }
    }

    Drive drive(config);
    std::vector<ScheduledChange> schedule;
    schedule.reserve(changes.size());
    for (const auto& [at, port, value] : changes) {
        schedule.push_back({tick * at, drive.columns()[source_columns.at(port)].source,
                            writers_.at(port), value});
    }
    std::stable_sort(schedule.begin(), schedule.end(), [](const auto& a, const auto& b) {
        return a.time < b.time;
    });

    auto& exec = drive.executor();
    Sampler sampler(drive, std::move(schedule), outputs.size(), tick * start_tick,
                    tick * sample_every, samples);
    if (!changes.empty()) {
        exec.register_tick(tick, std::numeric_limits<int>::min(),
                           SimulationExecutor::TickCallback::bind<&Sampler::apply>(sampler));
    }
    exec.register_tick(tick, std::numeric_limits<int>::max(),
                       SimulationExecutor::TickCallback::bind<&Sampler::record>(sampler));
    exec.run(tick * end_tick);
    return std::move(sampler).trace();
}

} // namespace pmsm
