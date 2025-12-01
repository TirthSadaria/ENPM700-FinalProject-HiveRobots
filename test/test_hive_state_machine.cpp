#include <gtest/gtest.h>
#include "hive_control/hive_controller.hpp"
#include "hive_control/hive_state.hpp"

using namespace hive_control;

TEST(HiveControllerTest, InitialStateIsExploring) {
  HiveController controller;
  EXPECT_EQ(controller.getCurrentStateName(), "EXPLORING");
}

TEST(HiveControllerTest, StateTransitionToReturn) {
  HiveController controller;
  auto return_state = std::make_shared<ReturnState>(true);
  controller.setState(return_state);
  EXPECT_EQ(controller.getCurrentStateName(), "RETURN_CLOCKWISE");
}

TEST(HiveControllerTest, StateTransitionToIdle) {
  HiveController controller;
  auto idle_state = std::make_shared<IdleState>();
  controller.setState(idle_state);
  EXPECT_EQ(controller.getCurrentStateName(), "IDLE");
}

// Add more tests for processLaserScan, velocity command, etc.

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
