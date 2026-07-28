#include <gtest/gtest.h>
#include "semantic_guard/emergency_cbf.hpp"

TEST(EmergencyCbf, ReversesForClosingHeadOnObstacle) {
    semantic_guard::EmergencyCbfParams params;
    std::vector<semantic_guard::EmergencyObstacle> obstacles{{0.8, 0.0, -1.0, 0.0, 0.4}};
    const auto command = semantic_guard::emergencyCbfCommand(
        0.0, 0.0, 0.0, 0.4, obstacles, params);
    EXPECT_LT(command.v, 0.0);
    EXPECT_GE(command.v, params.v_min);
}

TEST(EmergencyCbf, StopsWithNoObstacle) {
    semantic_guard::EmergencyCbfParams params;
    const auto command = semantic_guard::emergencyCbfCommand(
        0.0, 0.0, 0.0, 0.4, {}, params);
    EXPECT_DOUBLE_EQ(command.v, 0.0);
    EXPECT_DOUBLE_EQ(command.w, 0.0);
}

TEST(EmergencyCbf, ProgressesTowardPreferredHeadingWhenThreatsAreFar) {
    semantic_guard::EmergencyCbfParams params;
    params.activation_distance = 3.0;
    params.progress_v = 0.2;
    std::vector<semantic_guard::EmergencyObstacle> obstacles{{6.0, 0.0, 0.0, 0.0, 0.4}};
    const auto command = semantic_guard::emergencyCbfCommand(
        0.0, 0.0, M_PI_2, 0.4, obstacles, params, 0.0);
    EXPECT_TRUE(command.progress_mode);
    EXPECT_EQ(command.active_obstacle_count, 0);
    EXPECT_DOUBLE_EQ(command.v, 0.2);
    EXPECT_LT(command.w, 0.0);
}

TEST(EmergencyCbf, KeepsClosingObstacleActiveOutsideDistanceGate) {
    semantic_guard::EmergencyCbfParams params;
    params.activation_distance = 2.0;
    std::vector<semantic_guard::EmergencyObstacle> obstacles{{4.0, 0.0, -8.0, 0.0, 0.4}};
    const auto command = semantic_guard::emergencyCbfCommand(
        0.0, 0.0, 0.0, 0.4, obstacles, params, 0.0);
    EXPECT_FALSE(command.progress_mode);
    EXPECT_EQ(command.active_obstacle_count, 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
