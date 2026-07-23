#include <gtest/gtest.h>

#include "semantic_guard/safety_recurrence.hpp"

TEST(TeacherTheorem1Telemetry, RecomputesOneStepResidual) {
    semantic_guard::SafetyRecurrenceInput input;
    input.gamma = 0.35;
    input.h_eesm_t = 1.0;
    input.beta_t = 0.2;
    input.h_eesm_pred_next = 0.75;
    input.h_eesm_next = 0.70;
    input.beta_next = 0.25;
    input.epsilon_t = 0.02;
    input.epsilon_max = 0.05;
    input.delta_bar = 0.10;
    input.delta_beta_bar = 0.30;
    input.cbf_executed = true;

    const auto audit = semantic_guard::auditSafetyRecurrence(input);
    ASSERT_TRUE(audit.finite);
    EXPECT_DOUBLE_EQ(0.80, audit.H_t);
    EXPECT_DOUBLE_EQ(0.55, audit.H_pred_next);
    EXPECT_DOUBLE_EQ(0.45, audit.H_next);
    EXPECT_NEAR(0.05, audit.delta, 1e-12);
    EXPECT_NEAR(0.05, audit.delta_beta_plus, 1e-12);
    EXPECT_NEAR(0.40, audit.recursion_rhs, 1e-12);
    EXPECT_NEAR(0.05, audit.one_step_residual, 1e-12);
    EXPECT_NEAR(-0.45 / 0.35, audit.asymptotic_bound, 1e-12);
    EXPECT_NEAR(0.80 * std::pow(0.65, 2.0) -
                    (1.0 - std::pow(0.65, 2.0)) / 0.35 * 0.45,
                semantic_guard::practicalSafetyBound(audit.H_t, input.gamma,
                                                     audit.bar_w, 2),
                1e-12);
    EXPECT_TRUE(audit.theorem1_applicable);
}

TEST(TeacherTheorem1Telemetry, ExcludesBackupAndBoundViolations) {
    semantic_guard::SafetyRecurrenceInput input;
    input.gamma = 0.35;
    input.h_eesm_t = 1.0;
    input.beta_t = 0.2;
    input.h_eesm_pred_next = 0.75;
    input.h_eesm_next = 0.75;
    input.beta_next = 0.6;  // positive increment 0.4 > declared 0.3 bound
    input.epsilon_t = 0.10;  // slack 0.10 > epsilon_max
    input.epsilon_max = 0.05;
    input.delta_bar = 0.10;
    input.delta_beta_bar = 0.30;
    input.cbf_executed = true;
    input.backup_used = true;

    const auto audit = semantic_guard::auditSafetyRecurrence(input);
    ASSERT_TRUE(audit.finite);
    EXPECT_FALSE(audit.theorem1_applicable);
    EXPECT_GT(audit.delta_beta_plus, input.delta_beta_bar);
    EXPECT_FALSE(audit.finite && audit.theorem1_applicable && !input.backup_used);
}

TEST(TeacherTheorem1Telemetry, RejectsInvalidGamma) {
    semantic_guard::SafetyRecurrenceInput input;
    input.gamma = 0.0;
    const auto audit = semantic_guard::auditSafetyRecurrence(input);
    EXPECT_FALSE(audit.finite);
    EXPECT_FALSE(audit.theorem1_applicable);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
