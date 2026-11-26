#include "src/csv_logger.h"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace pmsm {
namespace {

constexpr std::size_t number_capacity = 64;

char* append_number(char* cursor, double value, char separator) {
    const auto result = std::to_chars(cursor, cursor + number_capacity, value,
        std::chars_format::general, std::numeric_limits<double>::max_digits10);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("Could not format CSV number");
    }
    *result.ptr = separator;
    return result.ptr + 1;
}

}  // namespace

CsvLogger::CsvLogger(std::ostream& output, const SimulationExecutor& executor,
                     const std::vector<Drive::Column>& columns)
    : output_(output), writer_(output), executor_(executor), columns_(columns),
      row_((columns.size() + 1) * (number_capacity + 1)) {
    std::vector<std::string> header{"time_s"};
    for (const auto& column : columns_) {
        header.push_back(column.header);
    }
    writer_ << header;
}

void CsvLogger::tick() {
    char* cursor = append_number(row_.data(),
        std::chrono::duration<double>(executor_.now()).count(), ',');
    for (const auto& column : columns_) {
        cursor = append_number(cursor, column.source(), ',');
    }
    cursor[-1] = '\n';
    output_.write(row_.data(), cursor - row_.data());
    if (!output_) {
        throw std::runtime_error("Could not write CSV row");
    }
}

}  // namespace pmsm
