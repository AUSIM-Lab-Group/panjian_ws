#include <gtest/gtest.h>

#include "semantic_guard/applied_margin_feedback.hpp"

TEST(AppliedMarginFeedback, AcceptsFixedCandidateWhenCategoryBoundDisabled) {
    EXPECT_TRUE(semantic_guard::validateAppliedMarginFeedback(
        "candidate", 0.4, 0.2, 0.4, 0.0, false));
}

TEST(AppliedMarginFeedback, RejectsFixedCandidateWhenCategoryBoundEnabled) {
    EXPECT_FALSE(semantic_guard::validateAppliedMarginFeedback(
        "candidate", 0.4, 0.2, 0.4, 0.0, true));
}

TEST(AppliedMarginFeedback, CandidateMustMatchPreGuardValue) {
    EXPECT_FALSE(semantic_guard::validateAppliedMarginFeedback(
        "candidate", 0.3, 0.8, 0.4, 0.0, false));
}

TEST(AppliedMarginFeedback, ZeroFallbackMustRemainZero) {
    EXPECT_TRUE(semantic_guard::validateAppliedMarginFeedback(
        "safe_stop", 0.0, 0.2, 0.4, 0.0, false));
    EXPECT_FALSE(semantic_guard::validateAppliedMarginFeedback(
        "safe_stop", 0.1, 0.2, 0.4, 0.0, false));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
