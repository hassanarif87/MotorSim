#include "python/drive_probe.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <meta>
#include <numeric>
#include <set>
#include <span>
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

/// @brief A list of writable output ports for blocks that support external control. These are
/// exposed to the Python wrapper for use in drive probing.
constexpr std::array writable_ports{
    writable_output<ConstantCurrent, ^^ConstantCurrentOutputs::value>(),
    writable_output<ConstantTorque, ^^ConstantTorqueOutputs::value>(),
    writable_output<SimpleFocVelocity, ^^SimpleFocVelocity::Outputs::omega_ref>(),
};

/// @brief A scheduled change to a drive input port at a specific simulation time.
struct ScheduledChange {
    Duration time;
    const void* address;
    /// @brief Function pointer to write a new value to the source port.
    void (*write)(const void*, double);
    double value;
};

/// @brief Records a trace of the drive's outputs at a fixed period while applying scheduled changes
/// to its inputs.
struct Sampler {
    const SimulationExecutor& executor;
    /// @brief Output column
    std::span<const Drive::Column> outputs;
    /// @brief Scheduled input as changes to the simulation
    std::span<const ScheduledChange> changes;
    Duration start, period;
    /// @brief Trace recording the sample
    DriveProbe::Trace& trace;

    void apply() {
        const auto now = executor.now();
        while (!changes.empty() && changes.front().time == now) {
            const auto& change = changes.front();
            change.write(change.address, change.value);
            changes = changes.subspan(1);
        }
    }

    void record() {
        // Record the drive's outputs at the configured sampling period. The simulation executor
        // invokes this callback at the tick rate, so we check if the current time is a multiple of
        // the sampling period.
        const auto now = executor.now();
        if (now < start || (now - start) % period != Duration::zero()) {
            return;
        }
        // Time record
        trace.time.push_back(std::chrono::duration<double>(now).count());

        // Output record
        for (const auto& output : outputs) {
            trace.values.push_back(output.source());
        }
    }
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

double DriveProbe::sample_time() const { return std::chrono::duration<double>(period_).count(); }

DriveProbe::Trace DriveProbe::run(const std::vector<std::string>& outputs, Tick start_tick,
                                  Tick samples, Tick sample_every,
                                  const std::vector<Change>& changes) const {
    if (outputs.empty() || start_tick < 0 || samples < 1 || sample_every < 1) {
        throw std::invalid_argument(
            "Expected output ports, nonnegative start and positive sampling");
    }
    const auto tick = config_.simulation.tick_interval;
    const auto max_ticks = Duration::max() / tick;
    if (start_tick > max_ticks || sample_every > max_ticks ||
        samples - 1 > (max_ticks - start_tick) / sample_every) {
        throw std::invalid_argument("Sampling duration is out of range");
    }
    const auto end_tick = start_tick + (samples - 1) * sample_every;

    // Trace records the time and requested output values at the sample.
    Trace trace;

    // Check that the requested trace size does not exceed the maximum size of the vectors.
    if (std::cmp_greater(samples, trace.time.max_size()) ||
        std::cmp_greater(samples, trace.values.max_size() / outputs.size())) {
        throw std::invalid_argument("Requested trace is too large");
    }

    auto config = config_;

    // Clear pre configured log columns, and fill with requested columns
    config.log_columns.clear();
    for (const auto& output : outputs) {
        config.log_columns.push_back(
            {"probe_" + std::to_string(config.log_columns.size()), output});
    }

    std::unordered_map<std::string, std::size_t> source_columns;
    for (const auto& [at, port, value] : changes) {
        if (at < 0 || at > end_tick || !std::isfinite(value)) {
            throw std::invalid_argument(
                "Input changes require a finite value and a tick in the run");
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
        schedule.push_back({tick * at, drive.columns()[source_columns.at(port)].source.address,
                            writers_.at(port), value});
    }
    std::stable_sort(schedule.begin(), schedule.end(),
                     [](const auto& a, const auto& b) { return a.time < b.time; });

    auto& exec = drive.executor();
    const auto sample_count = static_cast<std::size_t>(samples);
    trace.time.reserve(sample_count);
    trace.values.reserve(sample_count * outputs.size());
    Sampler sampler{exec,
                    std::span{drive.columns()}.first(outputs.size()),
                    schedule,
                    tick * start_tick,
                    tick * sample_every,
                    trace};
    if (!changes.empty()) {
        exec.register_tick(tick, std::numeric_limits<int>::min(),
                           SimulationExecutor::TickCallback::bind<&Sampler::apply>(sampler));
    }
    exec.register_tick(tick, std::numeric_limits<int>::max(),
                       SimulationExecutor::TickCallback::bind<&Sampler::record>(sampler));
    exec.run(tick * end_tick);
    return trace;
}

} // namespace pmsm
