#include <ros/ros.h>
#include <kortex_driver/non-generated/kortex_arm_driver.h>
#include <gtest/gtest.h>
#include <urdf/model.h>

class KortexDriverTest : public ::testing::Test {
 protected:
  
  void SetUp() override 
  {
  }

  void TearDown() override 
  {
  }

  ros::NodeHandle n;

};

// Wait for is_initialized to become true or until we reach the 10 seconds timeout
TEST_F(KortexDriverTest, initializeDriver)
{
    bool is_initialized = false;
    const int max_attempts = 20;
    for (int i = 0; i < max_attempts && is_initialized == false; i++)
    {
        ros::param::get("is_initialized", is_initialized);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }    
    ASSERT_TRUE(is_initialized);
}

// Check for all MoveIt services
// If they're advertised, then MoveIt started correctly
TEST_F(KortexDriverTest, initializeMoveIt)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    ASSERT_TRUE(ros::service::waitForService("apply_planning_scene", 30));
    ASSERT_TRUE(ros::service::waitForService("clear_octomap", 30));
    ASSERT_TRUE(ros::service::waitForService("compute_cartesian_path", 30));
    ASSERT_TRUE(ros::service::waitForService("compute_fk", 30));
    ASSERT_TRUE(ros::service::waitForService("compute_ik", 30));
    ASSERT_TRUE(ros::service::waitForService("get_planning_scene", 30));
    ASSERT_TRUE(ros::service::waitForService("plan_kinematic_path", 30));
    ASSERT_TRUE(ros::service::waitForService("query_planner_interface", 30));
    ASSERT_TRUE(ros::service::waitForService("check_state_validity", 30));
}

// Make sure the robot_description is parsable
TEST_F(KortexDriverTest, parseURDF)
{
    urdf::Model model;
    ASSERT_TRUE(model.initParam("robot_description"));
}
