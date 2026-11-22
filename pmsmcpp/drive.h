#pragma once

#include <algorithm>
#include <concepts>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "block_graph.h"
#include "configuration.h"

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
                const auto factory = find_factory(spec.type);
                if (!factory) {
                    throw std::invalid_argument("Unknown block type: " + spec.type);
                }
                factory(*this, graph, spec, has_continuous);
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
    template <typename Block>
    static void add(Drive& drive, BlockGraph& graph,
                    const BlockSpec& spec, bool& has_continuous) {
        constexpr bool periodic = requires(Block& block) {
            { block.tick() } -> std::same_as<void>;
        };
        constexpr bool continuous = requires(Block& block, SimulationExecutor::Duration dt) {
            { block.advance(dt) } -> std::same_as<void>;
        };
        static_assert(!(periodic && continuous), "A block must choose tick() or advance(dt)");

        if (spec.period.has_value() != periodic) {
            throw std::invalid_argument(periodic
                ? "A periodic block requires rate_hz" : "This block has no digital clock");
        }
        if (continuous && has_continuous) {
            throw std::invalid_argument("Exactly one continuous block is supported");
        }

        auto block = std::make_shared<Block>(spec);
        graph.add(spec.name, block->inputs, block->outputs);
        drive.blocks_.push_back(block);
        if constexpr (continuous) {
            drive.executor_.register_continuous(
                SimulationExecutor::AdvanceCallback::bind<&Block::advance>(*block));
            has_continuous = true;
        } else if constexpr (periodic) {
            drive.executor_.register_update(*spec.period, spec.priority,
                SimulationExecutor::UpdateCallback::bind<&Block::tick>(*block));
        }
    }

    using Factory = void (*)(Drive&, BlockGraph&, const BlockSpec&, bool&);
    static Factory find_factory(std::string_view type);

    SimulationExecutor executor_;
    std::vector<std::shared_ptr<void>> blocks_;
    std::vector<Column> columns_;
};

}  // namespace pmsm
