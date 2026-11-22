#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <csv.hpp>

#include "configuration.h"
#include "drive.h"
#include "simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using Duration = Executor::Duration;

// Give the CSV library a round-trippable field representation.
struct CsvNumber {
    CsvNumber(double field) : value(field) {}

    double value;

    operator std::string() const {
        char buffer[64];
        const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
            std::chars_format::general, std::numeric_limits<double>::max_digits10);
        if (result.ec != std::errc{}) {
            throw std::runtime_error("Could not format CSV number");
        }
        return {buffer, result.ptr};
    }
};

class CsvLogger {
public:
    CsvLogger(std::ostream& output, const Executor& executor,
              const std::vector<pmsm::Drive::Column>& columns)
        : output_(output), writer_(output), executor_(executor), columns_(columns),
          row_(columns.size() + 1, CsvNumber(0.0)) {
        std::vector<std::string> header{"time_s"};
        for (const auto& column : columns_) {
            header.push_back(column.header);
        }
        writer_ << header;
    }

    void tick() {
        row_[0] = std::chrono::duration<double>(executor_.now()).count();
        for (std::size_t i = 0; i < columns_.size(); ++i) {
            row_[i + 1] = columns_[i].source();
        }
        writer_ << row_;
        if (!output_) {
            throw std::runtime_error("Could not write CSV row");
        }
    }

private:
    std::ostream& output_;
    csv::CSVWriter<std::ostream, false> writer_;
    const Executor& executor_;
    const std::vector<pmsm::Drive::Column>& columns_;
    std::vector<CsvNumber> row_;
};

void simulate(const std::filesystem::path& path, const pmsm::Configuration& config) {
    const auto& sim = config.simulation;
    const auto max_step = sim.integration_step();
    const Duration log_period = sim.log_period ? *sim.log_period : max_step * sim.log_every_steps;
    const auto end = max_step * sim.steps;
    pmsm::Drive drive(config);
    auto& executor = drive.executor();

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Could not open CSV file");
    }
    {
        CsvLogger logger(output, executor, drive.columns());
        executor.register_update(log_period, std::numeric_limits<int>::max(),
            Executor::UpdateCallback::bind<&CsvLogger::tick>(logger));

        executor.run(end);
        if (end % log_period != Duration::zero()) {
            logger.tick();
        }
    }
    output.close();
    if (!output) {
        throw std::runtime_error("Could not finish writing CSV");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 3) {
        std::cerr << "Usage: pmsm_sim [output.csv [config.json]]\n";
        return 2;
    }

    const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
    const std::filesystem::path base = workspace ? workspace : ".";
    const auto output_path = base / (argc >= 2 ? argv[1] : "pmsm_results.csv");
    const auto config_path = base / (argc == 3 ? argv[2] : "pmsm_config.json");
    pmsm::Configuration config;
    try {
        config = pmsm::load_configuration(config_path);
        if (!config.connections || !config.log_columns) {
            const auto defaults = pmsm::load_configuration(base / "pmsm_config.json");
            if (!config.connections) {
                config.connections = defaults.connections;
            }
            if (!config.log_columns) {
                config.log_columns = defaults.log_columns;
            }
            if (!config.connections || !config.log_columns) {
                throw std::invalid_argument("The supplied or default configuration must define connections and log_columns");
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Invalid configuration " << config_path.string() << ": " << error.what() << '\n';
        return 1;
    }

    try {
        if (std::filesystem::exists(output_path) &&
            std::filesystem::equivalent(output_path, config_path)) {
            throw std::runtime_error("Output path refers to the configuration file");
        }
        simulate(output_path, config);
    } catch (const std::exception& error) {
        std::cerr << "Could not save " << output_path.string() << ": " << error.what() << '\n';
        return 1;
    }
    std::cout << "Saved " << output_path.string() << '\n';
}
