#include <gtest/gtest.h>

#include <cmath>

#include "semantic_guard/dynamic_tau.hpp"

namespace {
using semantic_guard::DynamicTauParams;
using semantic_guard::DynamicTauResult;
using semantic_guard::computeDynamicTau;

DynamicTauParams params() {
  DynamicTauParams p;
  p.ke = 0.30;
  p.t_max = 2.0;
  p.min_speed = 1e-6;
  p.min_distance = 1e-6;
  p.max_tau = 2.0;
  return p;
}

TEST(DynamicTauPolicy, StationaryAndRecedingReturnZero) {
  const auto stationary = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, params());
  const auto receding = computeDynamicTau(5.0, 0.0, 1.0, 0.0, 0.8, params());
  EXPECT_DOUBLE_EQ(stationary.tau, 0.0);
  EXPECT_DOUBLE_EQ(receding.tau, 0.0);
  EXPECT_TRUE(std::isfinite(stationary.tau));
  EXPECT_TRUE(std::isfinite(receding.tau));
}

TEST(DynamicTauPolicy, ClosingMotionProducesFiniteNonNegativeTau) {
  DynamicTauParams p = params();
  p.t_max = 5.0;
  const auto result = computeDynamicTau(5.0, 0.0, -1.0, 0.0, 0.8, p);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.closing);
  EXPECT_GE(result.tau, 0.0);
  EXPECT_LE(result.tau, p.max_tau);
  EXPECT_TRUE(std::isfinite(result.T_i));
}

TEST(DynamicTauPolicy, VelocityObstacleGateRejectsOutsideCone) {
  const auto result = computeDynamicTau(5.0, 0.0, -0.1, 2.0, 0.8, params());
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_EQ(result.reason, "velocity_gate");
}

TEST(DynamicTauPolicy, HorizonGateRejectsTiAfterTmax) {
  DynamicTauParams p = params();
  p.t_max = 0.5;
  const auto result = computeDynamicTau(5.0, 0.0, -0.1, 0.0, 0.8, p);
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_EQ(result.reason, "time_gate");
}

TEST(DynamicTauPolicy, DegenerateInputsAreFinite) {
  const auto zero_speed = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, params());
  const auto near_zero_distance = computeDynamicTau(0.0, 0.0, -1.0, 0.0, 0.8, params());
  const auto nan_position = computeDynamicTau(NAN, 0.0, -1.0, 0.0, 0.8, params());
  const auto inf_velocity = computeDynamicTau(5.0, 0.0, INFINITY, 0.0, 0.8, params());
  DynamicTauParams nan_params = params();
  nan_params.ke = NAN;
  const auto nan_parameter = computeDynamicTau(5.0, 0.0, -1.0, 0.0, 0.8, nan_params);
  for (const DynamicTauResult* result :
       {&zero_speed, &near_zero_distance, &nan_position, &inf_velocity,
        &nan_parameter}) {
    EXPECT_DOUBLE_EQ(result->tau, 0.0);
    EXPECT_TRUE(std::isfinite(result->tau));
    EXPECT_FALSE(result->valid);
    EXPECT_TRUE(std::isfinite(result->T_i));
    EXPECT_TRUE(std::isfinite(result->f_r));
    EXPECT_TRUE(std::isfinite(result->f_v));
    EXPECT_TRUE(std::isfinite(result->f_T));
    EXPECT_TRUE(std::isfinite(result->cos_delta));
  }
}

TEST(DynamicTauPolicy, ZeroRadiusKeepsConeBoundaryClosed) {
  const auto result = computeDynamicTau(2.0, 0.0, -1.0, 0.0, 0.0, params());
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_DOUBLE_EQ(result.f_v, 0.0);
  EXPECT_EQ(result.reason, "velocity_gate");
  EXPECT_TRUE(std::isfinite(result.tau));
}

TEST(DynamicTauPolicy, NonPositiveMaxTauNeverCreatesAnActiveValue) {
  for (const double max_tau : {0.0, -1.0}) {
    DynamicTauParams p = params();
    p.t_max = 4.0;
    p.max_tau = max_tau;
    const auto result = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, p);
    EXPECT_DOUBLE_EQ(result.tau, 0.0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "tau_invalid");
    EXPECT_TRUE(std::isfinite(result.tau));
    EXPECT_TRUE(std::isfinite(result.T_i));
  }
}

TEST(DynamicTauPolicy, ActiveTauIsClampedToMaxTau) {
  DynamicTauParams p = params();
  p.t_max = 4.0;
  p.max_tau = 0.5;
  const auto result = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, p);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.f_r, 1.0);
  EXPECT_DOUBLE_EQ(result.f_v, 1.0);
  EXPECT_DOUBLE_EQ(result.f_T, 1.0);
  EXPECT_NEAR(result.tau, p.max_tau, 1e-9);
  EXPECT_EQ(result.reason, "active");
}

TEST(DynamicTauPolicy, ReferenceFactorsAreAuditable) {
  DynamicTauParams p = params();
  p.t_max = 4.0;
  const auto result = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, p);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.f_r, 1.0);
  EXPECT_DOUBLE_EQ(result.f_v, 1.0);
  EXPECT_DOUBLE_EQ(result.f_T, 1.0);
  EXPECT_NEAR(result.T_i, 3.0, 1e-9);
  EXPECT_NEAR(result.tau, 0.9, 1e-9);
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
