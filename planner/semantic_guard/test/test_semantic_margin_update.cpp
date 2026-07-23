#include <gtest/gtest.h>

#include <limits>

#include "semantic_guard/semantic_margin_update.hpp"

namespace {

using semantic_guard::SemanticContextWeights;
using semantic_guard::SemanticProjectionPolicy;
using semantic_guard::computeProvisionalTeacherPhi;
using semantic_guard::computeTeacherSemanticMargin;
using semantic_guard::projectTeacherSemanticMargin;

TEST(TeacherSemanticMargin, ProvisionalPhiMatchesFrozenWeights) {
  const auto phi = computeProvisionalTeacherPhi(
      0.75, 1.0, 0.0, 0.0, SemanticContextWeights());
  ASSERT_TRUE(phi.valid);
  EXPECT_DOUBLE_EQ(phi.mu, 0.8);
  EXPECT_DOUBLE_EQ(phi.beta_tilde, 0.60);

  const auto result = computeTeacherSemanticMargin(
      0.75, 0.75, 1.0, 0.0, 0.0, 0.60, 0.30, 2.2, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.mu, 0.8);
  EXPECT_DOUBLE_EQ(result.beta_tilde, 0.60);
  EXPECT_DOUBLE_EQ(result.beta_upper_bound, 0.75);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.60);
}

TEST(TeacherSemanticMargin, FirstAppearanceUsesZeroPreviousMargin) {
  const auto result = computeTeacherSemanticMargin(
      0.75, 0.75, 1.0, 0.0, 0.0, 0.0, 0.30, 2.2, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.positive_increment_bound, 0.30);
  EXPECT_DOUBLE_EQ(result.beta_upper_bound, 0.30);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.30);
  EXPECT_TRUE(result.positive_increment_bound_active);
}

TEST(TeacherSemanticMargin, PositiveIncrementLimitDoesNotSlowDecrease) {
  const auto result = computeTeacherSemanticMargin(
      0.10, 0.70, 0.0, 0.0, 0.0, 0.60, 0.30, 2.0, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.beta_tilde, 0.06);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.06);
}

TEST(TeacherSemanticMargin, ProjectionAcceptsExplicitAblationCandidate) {
  const auto result = projectTeacherSemanticMargin(
      0.4, 0.75, 0.0, 0.30, 2.0, 0.10);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.beta_tilde, 0.4);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.30);
}

TEST(TeacherSemanticMargin, AblationCanDisableIndividualProjectionTerms) {
  SemanticProjectionPolicy no_rate;
  no_rate.enforce_positive_increment_bound = false;
  const auto rate_result = projectTeacherSemanticMargin(
      0.60, 0.75, 0.0, 0.30, 2.0, 0.10, no_rate);
  ASSERT_TRUE(rate_result.valid);
  EXPECT_DOUBLE_EQ(rate_result.beta_pre, 0.60);

  SemanticProjectionPolicy no_available;
  no_available.enforce_available_margin_bound = false;
  const auto available_result = projectTeacherSemanticMargin(
      0.60, 0.75, 0.60, 0.30, 0.15, 0.10, no_available);
  ASSERT_TRUE(available_result.valid);
  EXPECT_DOUBLE_EQ(available_result.beta_pre, 0.60);
}

TEST(TeacherSemanticMargin, AvailableGeometryProjectsPreGuardMargin) {
  const auto result = computeTeacherSemanticMargin(
      0.75, 0.75, 1.0, 0.0, 0.0, 0.20, 0.30, 0.35, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.available_margin, 0.25);
  EXPECT_DOUBLE_EQ(result.beta_upper_bound, 0.25);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.25);
  EXPECT_TRUE(result.available_margin_bound_active);
}

TEST(TeacherSemanticMargin, NoAvailableGeometryProducesZero) {
  const auto result = computeTeacherSemanticMargin(
      0.75, 0.75, 1.0, 0.0, 0.0, 0.20, 0.30, 0.05, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.available_margin, 0.0);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.0);
}

TEST(TeacherSemanticMargin, ClassChangeImmediatelyUsesNewMaximum) {
  const auto result = computeTeacherSemanticMargin(
      0.75, 0.20, 1.0, 0.0, 0.0, 0.60, 0.30, 2.0, 0.10,
      SemanticContextWeights());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.beta_upper_bound, 0.20);
  EXPECT_DOUBLE_EQ(result.beta_pre, 0.20);
  EXPECT_TRUE(result.category_bound_active);
}

TEST(TeacherSemanticMargin, RejectsInvalidContractInputs) {
  const auto nonfinite = computeTeacherSemanticMargin(
      0.75, 0.75, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0,
      0.0, 0.30, 2.0, 0.10, SemanticContextWeights());
  EXPECT_FALSE(nonfinite.valid);
  EXPECT_EQ(nonfinite.reason, "non_finite_input");

  const auto invalid_delta = computeTeacherSemanticMargin(
      0.75, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.10,
      SemanticContextWeights());
  EXPECT_FALSE(invalid_delta.valid);
  EXPECT_EQ(invalid_delta.reason, "invalid_range");
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
