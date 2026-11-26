#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <csv.hpp>

#include "src/drive/drive.h"
#include "src/simulation_executor.h"

namespace pmsm {

class CsvLogger {
  public:
    CsvLogger(std::ostream& output, const SimulationExecutor& executor,
              const std::vector<Drive::Column>& columns);

    void tick();

  private:
    std::ostream& output_;
    csv::CSVWriter<std::ostream, false> writer_;
    const SimulationExecutor& executor_;
    const std::vector<Drive::Column>& columns_;
    std::vector<char> row_;
};

} // namespace pmsm
