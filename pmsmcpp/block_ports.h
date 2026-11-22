#pragma once

#include <boost/describe.hpp>

namespace pmsm {

template <typename T>
class InputPort {
public:
    using Value = T;

    const T& read() const { return *source_; }
    bool connected() const { return source_ != nullptr; }
    void bind(const T& source) { source_ = &source; }

private:
    const T* source_ = nullptr;
};

namespace drive_blocks {

/// Carries a scalar with a quantity tag so graph connections can check its type.
template <typename Quantity>
struct Signal {
    Signal() = default;
    Signal(double initial) : value(initial) {}

    operator double() const { return value; }

    double value = 0.0;
};

using Voltage = Signal<struct VoltageTag>;
using Current = Signal<struct CurrentTag>;
using Speed = Signal<struct SpeedTag>;
using Position = Signal<struct PositionTag>;
using Duty = Signal<struct DutyTag>;

/// Port list for a source block with no inputs.
struct NoInputs {};
BOOST_DESCRIBE_STRUCT(NoInputs, (), ())

}  // namespace drive_blocks
}  // namespace pmsm
