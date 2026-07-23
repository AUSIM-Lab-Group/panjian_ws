#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "semantic_guard/typed_margin_contract.hpp"

namespace {

TEST(TypedMarginContract, ReordersByObstacleId) {
  std::vector<double> reordered;
  std::string reason;
  ASSERT_TRUE(semantic_guard::reorderValuesByObstacleId(
      {42, 7}, {0.42, 0.07}, {7, 42}, &reordered, &reason));
  EXPECT_EQ(reason, "ok");
  ASSERT_EQ(reordered.size(), 2U);
  EXPECT_DOUBLE_EQ(reordered[0], 0.07);
  EXPECT_DOUBLE_EQ(reordered[1], 0.42);
}

TEST(TypedMarginContract, RejectsMissingExtraAndDuplicateIds) {
  std::vector<double> reordered;
  std::string reason;
  EXPECT_FALSE(semantic_guard::reorderValuesByObstacleId(
      {1, 2}, {0.1, 0.2}, {1, 3}, &reordered, &reason));
  EXPECT_EQ(reason, "missing_or_extra_id");

  EXPECT_FALSE(semantic_guard::reorderValuesByObstacleId(
      {1, 1}, {0.1, 0.2}, {1, 2}, &reordered, &reason));
  EXPECT_EQ(reason, "duplicate_id");
}

TEST(TypedMarginContract, HistorySurvivesReorderAndClearsDisappearance) {
  std::map<uint32_t, double> history;
  ASSERT_TRUE(semantic_guard::storeAcceptedMargins(
      {10, 20}, {0.10, 0.20}, &history));

  std::vector<double> values;
  ASSERT_TRUE(semantic_guard::acceptedMarginsForIds(
      {20, 10}, history, &values));
  ASSERT_EQ(values.size(), 2U);
  EXPECT_DOUBLE_EQ(values[0], 0.20);
  EXPECT_DOUBLE_EQ(values[1], 0.10);

  semantic_guard::retainAcceptedMarginsForActiveIds({20}, &history);
  EXPECT_EQ(history.count(10), 0U);
  EXPECT_EQ(history.count(20), 1U);
  EXPECT_FALSE(semantic_guard::acceptedMarginsForIds(
      {10}, history, &values));
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
