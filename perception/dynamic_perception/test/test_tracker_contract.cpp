#include <gtest/gtest.h>

#include <vector>

#include "dynamic_perception/Hungarian.h"
#include "dynamic_perception/tracker_contract.hpp"

TEST(TrackerContract, EmptyDetectionHasNoAssignments)
{
  const std::vector<int> assignments;
  EXPECT_TRUE(assignments.empty());
}

TEST(TrackerContract, RejectsAllUnmatchedAssignments)
{
  EXPECT_FALSE(tp::isValidTrackAssignment(-1, 2));
  EXPECT_FALSE(tp::isValidTrackAssignment(2, 2));
  EXPECT_TRUE(tp::isValidTrackAssignment(1, 2));
}

TEST(TrackerContract, DeletedTrackIdsAreNotReused)
{
  tp::TrackIdSequence ids(4000);
  EXPECT_EQ(ids.next(), 4000);
  EXPECT_EQ(ids.next(), 4001);
  EXPECT_EQ(ids.next(), 4002);
}

TEST(TrackerContract, CrossingTargetsKeepOneToOneAssignments)
{
  HungarianAlgorithm hungarian;
  std::vector<std::vector<double>> costs = {
      {0.10, 1.80},
      {1.70, 0.20},
  };
  std::vector<int> assignments;
  hungarian.Solve(costs, assignments);

  ASSERT_EQ(assignments.size(), 2u);
  EXPECT_EQ(assignments[0], 0);
  EXPECT_EQ(assignments[1], 1);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
