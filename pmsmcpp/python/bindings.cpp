#include <algorithm>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "python/drive_probe.h"

namespace py = pybind11;

PYBIND11_MODULE(_pmsmcpp, module) {
    using pmsm::DriveProbe;
    using Tick = DriveProbe::Tick;
    py::class_<DriveProbe>(module, "DriveProbe")
        .def(py::init<const std::filesystem::path&>(), py::arg("config"))
        .def_property_readonly("input_ports", &DriveProbe::input_ports)
        .def_property_readonly("output_ports", &DriveProbe::output_ports)
        .def_property_readonly("tick_time", &DriveProbe::tick_time)
        .def_property_readonly("sample_ticks", &DriveProbe::sample_ticks)
        .def_property_readonly("sample_time", &DriveProbe::sample_time)
        .def(
            "run",
            [](const DriveProbe& drive, const std::vector<std::string>& outputs, Tick start_tick,
               Tick samples, Tick sample_every, const std::vector<DriveProbe::Change>& changes) {
                DriveProbe::Trace trace;
                {
                    py::gil_scoped_release release;
                    trace = drive.run(outputs, start_tick, samples, sample_every, changes);
                }
                const auto rows = static_cast<py::ssize_t>(trace.time.size());
                const auto columns = static_cast<py::ssize_t>(outputs.size());
                py::array_t<double> time(rows);
                py::array_t<double> values({rows, columns});
                std::copy(trace.time.begin(), trace.time.end(), time.mutable_data());
                std::copy(trace.values.begin(), trace.values.end(), values.mutable_data());
                return py::make_tuple(time, values);
            },
            py::arg("output_ports"), py::arg("start_tick"), py::arg("samples"),
            py::arg("sample_every"), py::arg("changes"));
}
