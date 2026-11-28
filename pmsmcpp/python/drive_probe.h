#pragma once

#include <filesystem>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "src/exec/configuration.h"

namespace pmsm {

class DriveProbe {
  public:
    explicit DriveProbe(const std::filesystem::path& path);

    const std::vector<std::string>& input_ports() const { return input_ports_; }
    const std::vector<std::string>& output_ports() const { return output_ports_; }
    using Tick = SimulationExecutor::Duration::rep;
    using Change = std::tuple<Tick, std::string, double>;

    Tick sample_ticks() const;
    double tick_time() const;
    double sample_time() const;

    struct Trace {
        std::vector<double> time;
        std::vector<double> values;
    };

    Trace run(const std::vector<std::string>& outputs, Tick start_tick, Tick samples,
              Tick sample_every, const std::vector<Change>& changes) const;

  private:
    Configuration config_;
    SimulationExecutor::Duration period_;
    std::vector<std::string> input_ports_, output_ports_;
    std::unordered_map<std::string, void (*)(const void*, double)> writers_;
};

} // namespace pmsm
