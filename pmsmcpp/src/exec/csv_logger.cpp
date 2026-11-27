#include "src/exec/csv_logger.h"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace pmsm {
namespace {

constexpr std::size_t number_capacity = 64;

/// @brief  Appends a number to a CSV row buffer, followed by a separator character.
/// @param cursor Pointer to the current position in the row buffer where the number should be
/// written.
/// @param value The number to append.
/// @param separator The separator character to follow the number.
/// @return A pointer to the position after the appended number and separator.
char* append_number(char* cursor, double value, char separator) {
    const auto result =
        std::to_chars(cursor, cursor + number_capacity, value, std::chars_format::general,
                      std::numeric_limits<double>::max_digits10);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("Could not format CSV number");
    }
    *result.ptr = separator;
    return result.ptr + 1;
}

} // namespace

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

/// @brief Writes a row of CSV data to the output stream, including the current simulation time and
/// the values from each column.
void CsvLogger::tick() {
    char* cursor =
        append_number(row_.data(), std::chrono::duration<double>(executor_.now()).count(), ',');
    for (const auto& column : columns_) {
        cursor = append_number(cursor, column.source(), ',');
    }
    cursor[-1] = '\n';
    output_.write(row_.data(), cursor - row_.data());
    if (!output_) {
        throw std::runtime_error("Could not write CSV row");
    }
}

} // namespace pmsm
