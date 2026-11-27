#pragma once

#include <algorithm>
#include <memory>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "src/graph/block_graph.h"
#include "src/exec/configuration.h"

namespace pmsm {

/// Owns block instances and schedules them after resolving the configured model.
class Drive {
  public:
    explicit Drive(const Configuration& config) : executor_(config.simulation.tick_interval) {
        const auto periodic_count =
            std::count_if(config.blocks.begin(), config.blocks.end(),
                          [](const BlockSpec& spec) { return spec.rate_hz.has_value(); });
        if (static_cast<std::size_t>(periodic_count) >= SimulationExecutor::max_periodic_blocks) {
            throw std::invalid_argument(
                "Too many periodic blocks with one executor slot reserved for CSV");
        }

        BlockGraph graph;
        std::unordered_set<std::string> names;
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
                factory(*this, graph, spec);
            } catch (const std::exception& error) {
                throw std::invalid_argument("Block " + spec.name + ": " + error.what());
            }
        }
        if (!executor_.has_continuous_block()) {
            throw std::invalid_argument("Exactly one continuous block is required");
        }
        graph.connect(config.connections);
        std::unordered_set<std::string> headers{"time_s"};
        for (const auto& [header, source] : config.log_columns) {
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
    static void add(Drive& drive, BlockGraph& graph, const BlockSpec& spec) {
        constexpr bool periodic = TickBlock<Block>;
        constexpr bool continuous = AdvanceBlock<Block, SimulationExecutor::Duration>;
        constexpr bool constant = requires { requires Block::is_constant; };
        static_assert(
            periodic + continuous + constant == 1,
            "A block must implement void tick(), void advance(Duration), or declare is_constant");

        const auto period = spec.period();
        if (period.has_value() != periodic) {
            throw std::invalid_argument(periodic ? "A periodic block requires rate_hz"
                                                 : "This block has no digital clock");
        }
        if (continuous && drive.executor_.has_continuous_block()) {
            throw std::invalid_argument("Exactly one continuous block is supported");
        }

        auto block = std::make_shared<Block>(spec);
        graph.add(spec.name, block->inputs, block->outputs);
        drive.blocks_.push_back(block);
        if constexpr (continuous) {
            drive.executor_.register_block(*block);
        } else if constexpr (periodic) {
            drive.executor_.register_block(*block, *period, spec.tick_priority());
        }
    }

    using Factory = void (*)(Drive&, BlockGraph&, const BlockSpec&);
    static Factory find_factory(std::string_view type);

    SimulationExecutor executor_;
    std::vector<std::shared_ptr<void>> blocks_;
    std::vector<Column> columns_;
};

} // namespace pmsm
