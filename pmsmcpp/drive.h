#pragma once

#include <algorithm>
#include <iterator>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "blocks/drive_blocks.h"

namespace pmsm {

/// Owns block instances and schedules them after resolving the configured model.
class Drive {
public:
    explicit Drive(const Configuration& config) : executor_(config.simulation.integration_step()) {
        const auto periodic_count = std::count_if(config.blocks.begin(), config.blocks.end(),
            [](const BlockSpec& spec) { return spec.period.has_value(); });
        if (static_cast<std::size_t>(periodic_count) >= SimulationExecutor::max_updates) {
            throw std::invalid_argument("Too many periodic blocks with one executor slot reserved for CSV");
        }

        BlockGraph graph;
        std::unordered_set<std::string> names;
        bool has_continuous = false;
        for (const auto& spec : config.blocks) {
            if (spec.name.empty() || spec.name.find('.') != std::string::npos ||
                !names.insert(spec.name).second) {
                throw std::invalid_argument("Invalid or duplicate block name: " + spec.name);
            }

            try {
                const auto factory = std::find_if(std::begin(factories_), std::end(factories_),
                    [&](const Factory& entry) { return entry.type == spec.type; });
                if (factory == std::end(factories_)) {
                    throw std::invalid_argument("Unknown block type: " + spec.type);
                }
                factory->create(*this, graph, spec, has_continuous);
            } catch (const std::exception& error) {
                throw std::invalid_argument("Block " + spec.name + ": " + error.what());
            }
        }
        if (!has_continuous) {
            throw std::invalid_argument("Exactly one continuous block is required");
        }
        graph.connect(config.connections.value());
        std::unordered_set<std::string> headers{"time_s"};
        for (const auto& [header, source] : config.log_columns.value()) {
            if (header.empty() || !headers.insert(header).second) {
                throw std::invalid_argument("Invalid or duplicate CSV column: " + header);
            }
            columns_.push_back({header, graph.numeric_port(source)});
        }
    }

    Drive(const Drive&) = delete;
    Drive& operator=(const Drive&) = delete;

    struct Column {
        std::string header;
        OutputReader source;
    };

    const std::vector<Column>& columns() const { return columns_; }
    SimulationExecutor& executor() { return executor_; }

private:
    enum class Mode { continuous, periodic, constant };

    template <typename Block, Mode mode>
    static void add(Drive& drive, BlockGraph& graph,
                    const BlockSpec& spec, bool& has_continuous) {
        if (spec.period.has_value() != (mode == Mode::periodic)) {
            throw std::invalid_argument(mode == Mode::periodic
                ? "A periodic block requires rate_hz" : "This block has no digital clock");
        }
        if (mode == Mode::continuous && has_continuous) {
            throw std::invalid_argument("Exactly one continuous block is supported");
        }

        auto block = std::make_shared<Block>(spec);
        graph.add(spec.name, block->inputs, block->outputs);
        drive.blocks_.push_back(block);
        if constexpr (mode == Mode::continuous) {
            drive.executor_.register_continuous(
                SimulationExecutor::AdvanceCallback::bind<&Block::advance>(*block));
            has_continuous = true;
        } else if constexpr (mode == Mode::periodic) {
            drive.executor_.register_update(*spec.period, spec.priority,
                SimulationExecutor::UpdateCallback::bind<&Block::tick>(*block));
        }
    }

    struct Factory {
        std::string_view type;
        void (*create)(Drive&, BlockGraph&, const BlockSpec&, bool&);
    };

#define PMSM_BLOCK_TYPES(X) \
    X("pmsm", Motor, continuous) \
    X("speed_control", SpeedController, periodic) \
    X("current_feedback", Feedback, periodic) \
    X("current_pi", CurrentController, periodic) \
    X("svpwm", Pwm, periodic) \
    X("inverter", Inverter, periodic) \
    X("constant_current", ConstantCurrent, constant)

#define PMSM_FACTORY(type, block, mode) {type, &add<drive_blocks::block, Mode::mode>},
    inline static constexpr Factory factories_[]{PMSM_BLOCK_TYPES(PMSM_FACTORY)};
#undef PMSM_FACTORY
#undef PMSM_BLOCK_TYPES

    SimulationExecutor executor_;
    std::vector<std::shared_ptr<void>> blocks_;
    std::vector<Column> columns_;
};

}  // namespace pmsm
