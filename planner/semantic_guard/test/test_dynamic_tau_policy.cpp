#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "semantic_guard/dynamic_tau.hpp"
#include "semantic_guard/planar_velocity.hpp"

namespace {
using semantic_guard::DynamicTauMode;
using semantic_guard::DynamicTauParams;
using semantic_guard::DynamicTauResult;
using semantic_guard::computeDynamicTau;
using semantic_guard::bodyPlanarVelocityToWorld;
using semantic_guard::dynamicTauModeName;
using semantic_guard::parseDynamicTauMode;

DynamicTauParams teacherParams() {
  DynamicTauParams p;
  p.mode = DynamicTauMode::kTeacherTca;
  p.ke = 0.30;
  p.min_speed = 1e-6;
  p.max_tau = 5.0;
  p.delta_tau = 1.0;
  return p;
}

DynamicTauParams legacyParams() {
  DynamicTauParams p;
  p.mode = DynamicTauMode::kLegacyGate;
  p.ke = 0.30;
  p.t_max = 2.0;
  p.min_speed = 1e-6;
  p.min_distance = 1e-6;
  p.max_tau = 2.0;
  return p;
}

TEST(PlanarVelocityContract, RotatesBothBodyComponentsAtNonzeroYaw) {
  const double half_sqrt_two = std::sqrt(0.5);
  double world_vx = 0.0;
  double world_vy = 0.0;
  ASSERT_TRUE(bodyPlanarVelocityToWorld(
      1.0, 0.25, 0.0, 0.0, half_sqrt_two, half_sqrt_two,
      &world_vx, &world_vy));
  EXPECT_NEAR(world_vx, -0.25, 1e-12);
  EXPECT_NEAR(world_vy, 1.0, 1e-12);
}

TEST(PlanarVelocityContract, RejectsDegenerateQuaternion) {
  double world_vx = 0.0;
  double world_vy = 0.0;
  EXPECT_FALSE(bodyPlanarVelocityToWorld(
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, &world_vx, &world_vy));
}

TEST(DynamicTauModeContract, TeacherTcaIsDefault) {
  const DynamicTauParams p;
  EXPECT_EQ(p.mode, DynamicTauMode::kTeacherTca);
  EXPECT_STREQ(dynamicTauModeName(p.mode), "teacher_tca");
}

TEST(DynamicTauModeContract, StrictParserRoundTripsCanonicalNames) {
  const struct {
    const char* name;
    DynamicTauMode mode;
  } cases[] = {
      {"legacy_gate", DynamicTauMode::kLegacyGate},
      {"teacher_tca", DynamicTauMode::kTeacherTca},
      {"teacher_ke_tca", DynamicTauMode::kTeacherKeTca},
  };

  for (const auto& test_case : cases) {
    DynamicTauMode parsed = DynamicTauMode::kLegacyGate;
    ASSERT_TRUE(parseDynamicTauMode(test_case.name, &parsed));
    EXPECT_EQ(parsed, test_case.mode);
    EXPECT_STREQ(dynamicTauModeName(parsed), test_case.name);
  }

  for (const std::string bad_name :
       {"", "teacher", "ke_tca", "Teacher_Tca", " teacher_tca"}) {
    DynamicTauMode parsed = DynamicTauMode::kTeacherKeTca;
    EXPECT_FALSE(parseDynamicTauMode(bad_name, &parsed));
    EXPECT_EQ(parsed, DynamicTauMode::kTeacherKeTca);
  }
  EXPECT_FALSE(parseDynamicTauMode("teacher_tca", nullptr));
  EXPECT_STREQ(dynamicTauModeName(static_cast<DynamicTauMode>(99)), "invalid");
}

TEST(TeacherTca, HeadOnUsesTeacherFormulaExactly) {
  const DynamicTauParams p = teacherParams();
  // -(l dot v)/(||v||^2+delta) = -(-10)/(4+1) = 2.
  const auto result = computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, p);
  EXPECT_TRUE(result.inputs_finite);
  EXPECT_TRUE(result.config_valid);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.closing);
  EXPECT_FALSE(result.ke_scaled);
  EXPECT_DOUBLE_EQ(result.relative_dot, -10.0);
  EXPECT_DOUBLE_EQ(result.speed_squared, 4.0);
  EXPECT_DOUBLE_EQ(result.denominator, 5.0);
  EXPECT_DOUBLE_EQ(result.t_ca_raw, 2.0);
  EXPECT_DOUBLE_EQ(result.t_ca_clipped, 2.0);
  EXPECT_DOUBLE_EQ(result.T_i, 2.0);
  EXPECT_DOUBLE_EQ(result.tau_unclipped, 2.0);
  EXPECT_DOUBLE_EQ(result.tau, 2.0);
  EXPECT_EQ(result.reason, "teacher_tca_active");
}

TEST(TeacherTca, RecedingAndTangentClipAtZero) {
  const DynamicTauParams p = teacherParams();
  const auto receding = computeDynamicTau(5.0, 0.0, 2.0, 0.0, 0.8, p);
  const auto tangent = computeDynamicTau(5.0, 0.0, 0.0, 2.0, 0.8, p);

  EXPECT_LT(receding.t_ca_raw, 0.0);
  EXPECT_TRUE(receding.lower_clipped);
  EXPECT_DOUBLE_EQ(receding.tau, 0.0);
  EXPECT_TRUE(receding.computed);
  EXPECT_FALSE(receding.valid);
  EXPECT_EQ(receding.reason, "teacher_receding");

  EXPECT_DOUBLE_EQ(tangent.t_ca_raw, 0.0);
  EXPECT_TRUE(tangent.lower_clipped);
  EXPECT_DOUBLE_EQ(tangent.tau, 0.0);
  EXPECT_TRUE(tangent.computed);
  EXPECT_FALSE(tangent.valid);
  EXPECT_EQ(tangent.reason, "teacher_tangent");
}

TEST(TeacherTca, StaticRelativeStateUsesRegularizedFormula) {
  const DynamicTauParams p = teacherParams();
  const auto result = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, p);
  EXPECT_TRUE(result.inputs_finite);
  EXPECT_TRUE(result.config_valid);
  EXPECT_DOUBLE_EQ(result.denominator, p.delta_tau);
  EXPECT_DOUBLE_EQ(result.t_ca_raw, 0.0);
  EXPECT_TRUE(result.lower_clipped);
  EXPECT_DOUBLE_EQ(result.tau, 0.0);
  EXPECT_TRUE(result.computed);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "teacher_tangent");
}

TEST(TeacherTca, NearZeroSpeedStillUsesExactFormula) {
  DynamicTauParams p = teacherParams();
  p.min_speed = 1.0;
  const auto result =
      computeDynamicTau(1.0, 0.0, -1.0e-5, 0.0, 0.8, p);
  const double expected_raw = 1.0e-5 / (1.0e-10 + p.delta_tau);
  EXPECT_NEAR(result.t_ca_raw, expected_raw, 1.0e-12);
  EXPECT_NEAR(result.tau, expected_raw, 1.0e-12);
  EXPECT_TRUE(result.valid);
}

TEST(TeacherTca, UpperClipUsesMaxTau) {
  DynamicTauParams p = teacherParams();
  p.delta_tau = 1e-6;
  p.max_tau = 1.25;
  const auto result = computeDynamicTau(10.0, 0.0, -1.0, 0.0, 0.8, p);
  EXPECT_GT(result.t_ca_raw, p.max_tau);
  EXPECT_TRUE(result.upper_clipped);
  EXPECT_DOUBLE_EQ(result.t_ca_clipped, p.max_tau);
  EXPECT_DOUBLE_EQ(result.tau, p.max_tau);
  EXPECT_EQ(result.reason, "teacher_tca_clipped");
}

TEST(TeacherTca, SimultaneousRelativeConventionSignFlipIsInvariant) {
  const DynamicTauParams p = teacherParams();
  const auto obstacle_minus_robot =
      computeDynamicTau(5.0, 2.0, -2.0, -1.0, 0.8, p);
  const auto robot_minus_obstacle =
      computeDynamicTau(-5.0, -2.0, 2.0, 1.0, 0.8, p);
  EXPECT_DOUBLE_EQ(obstacle_minus_robot.relative_dot,
                   robot_minus_obstacle.relative_dot);
  EXPECT_DOUBLE_EQ(obstacle_minus_robot.speed_squared,
                   robot_minus_obstacle.speed_squared);
  EXPECT_DOUBLE_EQ(obstacle_minus_robot.t_ca_raw,
                   robot_minus_obstacle.t_ca_raw);
  EXPECT_DOUBLE_EQ(obstacle_minus_robot.tau, robot_minus_obstacle.tau);
  EXPECT_EQ(obstacle_minus_robot.reason, robot_minus_obstacle.reason);
}

TEST(TeacherTca, InflatedRadiusDoesNotEnterTeacherFormula) {
  const DynamicTauParams p = teacherParams();
  const auto small_radius =
      computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.0, p);
  const auto large_radius =
      computeDynamicTau(5.0, 0.0, -2.0, 0.0, 100.0, p);
  EXPECT_DOUBLE_EQ(small_radius.t_ca_raw, large_radius.t_ca_raw);
  EXPECT_DOUBLE_EQ(small_radius.t_ca_clipped, large_radius.t_ca_clipped);
  EXPECT_DOUBLE_EQ(small_radius.tau, large_radius.tau);
  EXPECT_EQ(small_radius.reason, large_radius.reason);
}

TEST(TeacherKeTca, IsExplicitlyDiagnosedAndNeverImplicit) {
  DynamicTauParams p = teacherParams();
  p.mode = DynamicTauMode::kTeacherKeTca;
  p.ke = 0.30;
  const auto result = computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, p);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.ke_scaled);
  EXPECT_DOUBLE_EQ(result.t_ca_clipped, 2.0);
  EXPECT_DOUBLE_EQ(result.tau_unclipped, 0.6);
  EXPECT_DOUBLE_EQ(result.tau, 0.6);
  EXPECT_EQ(result.reason, "teacher_ke_tca_active");
}

TEST(TeacherTca, InvalidConfigurationAndArithmeticFailClosed) {
  DynamicTauParams zero_delta = teacherParams();
  zero_delta.delta_tau = 0.0;
  DynamicTauParams negative_delta = teacherParams();
  negative_delta.delta_tau = -1.0;
  DynamicTauParams zero_max_tau = teacherParams();
  zero_max_tau.max_tau = 0.0;
  DynamicTauParams non_finite_delta = teacherParams();
  non_finite_delta.delta_tau =
      std::numeric_limits<double>::quiet_NaN();

  for (const DynamicTauParams* invalid :
       {&zero_delta, &negative_delta, &zero_max_tau}) {
    const auto result =
        computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, *invalid);
    EXPECT_DOUBLE_EQ(result.tau, 0.0);
    EXPECT_FALSE(result.computed);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "invalid_config");
  }
  const auto nan_config =
      computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, non_finite_delta);
  EXPECT_EQ(nan_config.reason, "non_finite_input");

  DynamicTauParams legacy_only_values = teacherParams();
  legacy_only_values.min_speed =
      std::numeric_limits<double>::quiet_NaN();
  legacy_only_values.min_distance = -1.0;
  legacy_only_values.t_max = -1.0;
  const auto strict_teacher = computeDynamicTau(
      5.0, 0.0, -2.0, 0.0, 0.8, legacy_only_values);
  EXPECT_TRUE(strict_teacher.valid);
  EXPECT_EQ(strict_teacher.reason, "teacher_tca_active");

  DynamicTauParams invalid_mode = teacherParams();
  invalid_mode.mode = static_cast<DynamicTauMode>(99);
  const auto mode_result =
      computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, invalid_mode);
  EXPECT_DOUBLE_EQ(mode_result.tau, 0.0);
  EXPECT_FALSE(mode_result.valid);
  EXPECT_EQ(mode_result.reason, "invalid_mode");
}

TEST(TeacherKeTca, RequiresFiniteStrictlyPositiveKe) {
  DynamicTauParams zero_ke = teacherParams();
  zero_ke.mode = DynamicTauMode::kTeacherKeTca;
  zero_ke.ke = 0.0;
  DynamicTauParams nan_ke = zero_ke;
  nan_ke.ke = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, zero_ke).reason,
            "invalid_config");
  EXPECT_EQ(computeDynamicTau(5.0, 0.0, -2.0, 0.0, 0.8, nan_ke).reason,
            "non_finite_input");
}

TEST(LegacyGateGolden, ReferenceFactorsAndTauRemainUnchanged) {
  DynamicTauParams p = legacyParams();
  p.t_max = 4.0;
  const auto result = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, p);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.f_r, 1.0);
  EXPECT_DOUBLE_EQ(result.f_v, 1.0);
  EXPECT_DOUBLE_EQ(result.f_T, 1.0);
  EXPECT_NEAR(result.T_i, 3.0, 1e-12);
  EXPECT_NEAR(result.tau, 0.9, 1e-12);
  EXPECT_EQ(result.reason, "active");
}

TEST(LegacyGateGolden, StaticRecedingConeAndHorizonReasonsAreUnchanged) {
  const DynamicTauParams p = legacyParams();
  const auto stationary = computeDynamicTau(5.0, 0.0, 0.0, 0.0, 0.8, p);
  const auto receding = computeDynamicTau(5.0, 0.0, 1.0, 0.0, 0.8, p);
  const auto outside_cone =
      computeDynamicTau(5.0, 0.0, -0.1, 2.0, 0.8, p);
  DynamicTauParams short_horizon = p;
  short_horizon.t_max = 0.5;
  const auto after_horizon =
      computeDynamicTau(5.0, 0.0, -0.1, 0.0, 0.8, short_horizon);

  EXPECT_EQ(stationary.reason, "speed_degenerate");
  EXPECT_EQ(receding.reason, "receding_or_nonclosing");
  EXPECT_EQ(outside_cone.reason, "velocity_gate");
  EXPECT_EQ(after_horizon.reason, "time_gate");
  for (const DynamicTauResult* result :
       {&stationary, &receding, &outside_cone, &after_horizon}) {
    EXPECT_DOUBLE_EQ(result->tau, 0.0);
    EXPECT_FALSE(result->valid);
  }
}

TEST(LegacyGateGolden, ClampAndBoundaryBehaviorRemainUnchanged) {
  DynamicTauParams p = legacyParams();
  p.t_max = 4.0;
  p.max_tau = 0.5;
  const auto clamped = computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, p);
  EXPECT_TRUE(clamped.valid);
  EXPECT_NEAR(clamped.tau, 0.5, 1e-12);

  const auto zero_radius =
      computeDynamicTau(2.0, 0.0, -1.0, 0.0, 0.0, legacyParams());
  EXPECT_DOUBLE_EQ(zero_radius.f_v, 0.0);
  EXPECT_EQ(zero_radius.reason, "velocity_gate");
}

TEST(LegacyGateGolden, LegacyValidationPriorityRemainsUnchanged) {
  DynamicTauParams invalid = legacyParams();
  invalid.max_tau = -1.0;
  const auto invalid_config =
      computeDynamicTau(2.0, 0.0, -0.5, 0.0, 0.5, invalid);
  const auto non_finite =
      computeDynamicTau(NAN, 0.0, -0.5, 0.0, 0.5, invalid);
  EXPECT_EQ(invalid_config.reason, "invalid_config");
  EXPECT_EQ(non_finite.reason, "non_finite_input");
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
