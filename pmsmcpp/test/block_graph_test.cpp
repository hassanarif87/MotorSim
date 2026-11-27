#include <stdexcept>
#include <string_view>

#include "gtest/gtest.h"

#include "src/graph/block_graph.h"

namespace {

struct Inputs {
    pmsm::InputPort<double> signal;
};

struct Outputs {
    double signal = 0.0;
    int count = 0;
};

void expect_rejection(const pmsm::Connections& connections, std::string_view expected) {
    Inputs inputs;
    Outputs outputs;
    pmsm::BlockGraph graph;
    graph.add("block", inputs, outputs);
    try {
        graph.connect(connections);
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string_view(error.what()).find(expected), std::string_view::npos)
            << error.what();
        return;
    }
    ADD_FAILURE() << "Expected connection to be rejected with: " << expected;
}

TEST(BlockGraphTest, BindingRemainsLiveAfterGraphDestruction) {
    Inputs inputs;
    Outputs outputs{1.0, 0};
    {
        pmsm::BlockGraph graph;
        graph.add("block", inputs, outputs);
        graph.connect({{"block.signal", "block.signal"}});
    }
    EXPECT_EQ(inputs.signal.read(), 1.0);
    outputs.signal = 2.0;
    EXPECT_EQ(inputs.signal.read(), 2.0);
}

TEST(BlockGraphTest, RejectsInvalidConnections) {
    expect_rejection({}, "Missing source");
    expect_rejection({{"block.missing", "block.signal"}}, "Unknown input");
    expect_rejection({{"block.signal", "block.missing"}}, "Unknown output");
    expect_rejection({{"block.signal", "block.count"}}, "Incompatible port types");
    expect_rejection({{"block.signal", "block.signal"}, {"block.signal", "block.signal"}},
                     "Multiple sources");
}

} // namespace
