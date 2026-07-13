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
  const auto result = computeDynamicTau(5.0, 0.0, -1.0, 0.0, 0.8, params());
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.closing);
  EXPECT_GE(result.tau, 0.0);
  EXPECT_LE(result.tau, params().max_tau);
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
  const auto non_finite = computeDynamicTau(NAN, 0.0, -1.0, 0.0, 0.8, params());
  for (const DynamicTauResult* result : {&zero_speed, &near_zero_distance, &non_finite}) {
    EXPECT_DOUBLE_EQ(result->tau, 0.0);
    EXPECT_TRUE(std::isfinite(result->tau));
    EXPECT_FALSE(result->valid);
  }
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
