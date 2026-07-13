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
  EXPECT_DOUBLE_EQ(receding.f_T, 1.0);
  EXPECT_EQ(stationary.reason, "speed_degenerate");
  EXPECT_EQ(receding.reason, "receding_or_nonclosing");
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
  EXPECT_EQ(result.reason, "active");
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
  EXPECT_EQ(zero_speed.reason, "speed_degenerate");
  EXPECT_EQ(near_zero_distance.reason, "distance_degenerate");
  EXPECT_EQ(nan_position.reason, "non_finite_input");
  EXPECT_EQ(inf_velocity.reason, "non_finite_input");
  EXPECT_EQ(nan_parameter.reason, "non_finite_input");
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
    EXPECT_EQ(result.reason, "invalid_config");
    EXPECT_TRUE(std::isfinite(result.tau));
    EXPECT_TRUE(std::isfinite(result.T_i));
  }
}

TEST(DynamicTauPolicy, InvalidConfigReasonOverridesClosingAndRecedingInputs) {
  struct InvalidConfigCase {
    const char* name;
    DynamicTauParams config;
    double radius;
  };

  DynamicTauParams negative_ke = params();
  negative_ke.ke = -1.0;
  DynamicTauParams negative_t_max = params();
  negative_t_max.t_max = -1.0;
  DynamicTauParams negative_min_speed = params();
  negative_min_speed.min_speed = -1.0;
  DynamicTauParams negative_min_distance = params();
  negative_min_distance.min_distance = -1.0;
  DynamicTauParams zero_max_tau = params();
  zero_max_tau.max_tau = 0.0;
  DynamicTauParams negative_max_tau = params();
  negative_max_tau.max_tau = -1.0;

  const InvalidConfigCase cases[] = {
      {"negative_ke", negative_ke, 0.5},
      {"negative_t_max", negative_t_max, 0.5},
      {"negative_min_speed", negative_min_speed, 0.5},
      {"negative_min_distance", negative_min_distance, 0.5},
      {"negative_radius", params(), -0.5},
      {"zero_max_tau", zero_max_tau, 0.5},
      {"negative_max_tau", negative_max_tau, 0.5},
  };

  for (const InvalidConfigCase& test_case : cases) {
    SCOPED_TRACE(test_case.name);
    const auto closing =
        computeDynamicTau(2.0, 0.0, -0.5, 0.0, test_case.radius,
                          test_case.config);
    const auto receding =
        computeDynamicTau(2.0, 0.0, 0.5, 0.0, test_case.radius,
                          test_case.config);
    for (const DynamicTauResult* result : {&closing, &receding}) {
      EXPECT_DOUBLE_EQ(result->tau, 0.0);
      EXPECT_FALSE(result->valid);
      EXPECT_EQ(result->reason, "invalid_config");
    }
  }
}

TEST(DynamicTauPolicy, NonFiniteInputTakesPriorityOverInvalidConfig) {
  DynamicTauParams invalid_config = params();
  invalid_config.max_tau = -1.0;
  const auto result =
      computeDynamicTau(NAN, 0.0, -0.5, 0.0, 0.5, invalid_config);
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "non_finite_input");
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
