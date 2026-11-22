#include <iostream>
#include <stdexcept>
#include <string_view>

#include "src/graph/block_graph.h"

namespace {

struct Inputs {
    pmsm::InputPort<double> signal;
};

struct Outputs {
    double signal = 0.0;
    int count = 0;
};

bool rejects(const pmsm::Connections& connections, std::string_view expected) {
    Inputs inputs;
    Outputs outputs;
    pmsm::BlockGraph graph;
    graph.add("block", inputs, outputs);
    try {
        graph.connect(connections);
    } catch (const std::invalid_argument& error) {
        return std::string_view(error.what()).find(expected) != std::string_view::npos;
    }
    return false;
}

}  // namespace

int main() {
    Inputs inputs;
    Outputs outputs{1.0, 0};
    {
        pmsm::BlockGraph graph;
        graph.add("block", inputs, outputs);
        graph.connect({{"block.signal", "block.signal"}});
    }
    const bool initial_value_correct = inputs.signal.read() == 1.0;
    outputs.signal = 2.0;
    if (!initial_value_correct || inputs.signal.read() != 2.0 ||
        !rejects({}, "Missing source") ||
        !rejects({{"block.missing", "block.signal"}}, "Unknown input") ||
        !rejects({{"block.signal", "block.missing"}}, "Unknown output") ||
        !rejects({{"block.signal", "block.count"}}, "Incompatible port types") ||
        !rejects({{"block.signal", "block.signal"}, {"block.signal", "block.signal"}},
                 "Multiple sources")) {
        std::cerr << "Unexpected graph binding or validation result\n";
        return 1;
    }
}
