#include <gtest/gtest.h>

#include "semantic_guard/guard_backtracking.hpp"

TEST(TeacherGuardBacktracking, UsesFiniteCandidateSetAndExplicitZero) {
    EXPECT_DOUBLE_EQ(0.8, semantic_guard::teacherBacktrackingCandidate(0.8, 0.5, 0, 4));
    EXPECT_DOUBLE_EQ(0.4, semantic_guard::teacherBacktrackingCandidate(0.8, 0.5, 1, 4));
    EXPECT_DOUBLE_EQ(0.1, semantic_guard::teacherBacktrackingCandidate(0.8, 0.5, 3, 4));
    EXPECT_DOUBLE_EQ(0.0, semantic_guard::teacherBacktrackingCandidate(0.8, 0.5, 4, 4));
}

TEST(TeacherGuardBacktracking, OrdersRiskDescendingAndTiesById) {
    const std::vector<unsigned int> ids{9, 4, 7};
    const std::vector<double> risk{0.2, 0.8, 0.8};
    const auto order = semantic_guard::teacherRiskOrder(ids, risk);
    ASSERT_EQ(3u, order.size());
    EXPECT_EQ(1u, order[0]);
    EXPECT_EQ(2u, order[1]);
    EXPECT_EQ(0u, order[2]);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
