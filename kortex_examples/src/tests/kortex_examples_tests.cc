#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>

class KortexDriverTest : public ::testing::Test {
 protected:
  
  void SetUp() override 
  {
      // Make sure we erase all test results
      for (auto ex : examples)
      {
          ros::param::del(example_prefix + ex.first);
      }
      
      // Look for the /<robot_name>/has_vision_module parameter to know if the vision examples should fail
      ros::param::get("~robot_name", robot_name);
      ros::param::get("/" + robot_name + "/has_vision_module", has_vision_module);

      // Add the vision configuration examples to the list of tests to run if needed
      if (has_vision_module)
      {
        examples.push_back(std::make_pair("vision_configuration_cpp", true));
        examples.push_back(std::make_pair("vision_configuration_python", true));        
      } 
  }

  void TearDown() override 
  {
  }

  ros::NodeHandle n;
  std::string robot_name;
  bool has_vision_module;
  const int timeout_seconds = 90;
  const std::string example_prefix = "/kortex_examples_test_results/";
  // The first element is the name of the example
  // The second element is a boolean value to indicate what value we expect when running this test
  std::vector<std::pair<std::string, bool>> examples = 
  {
      {"actuator_configuration_python", true},
      {"cartesian_poses_with_notifications_cpp", true},
      {"cartesian_poses_with_notifications_python", true},
      {"full_arm_movement_cpp", true},
      {"full_arm_movement_python", true},
      {"actuator_configuration_cpp", true},
      {"waypoint_action_cpp", true},
      {"waypoint_action_python", true},
      {"moveit_general_python", true}
  };

  bool waitForExample(std::string example_name)
  {
      bool success = false;
      for (int i = 0; i < timeout_seconds && !success; i++)
      {
          // If the parameter is not yet on the server, wait for it to come up
          if (!ros::param::has(example_prefix + example_name))
          {
              std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
          // If the parameter is set on the server, break
          if (ros::param::get(example_prefix + example_name, success))
          {
              break;
          }
      }
      return success;
  }

};

// Check if all examples ran correctly
TEST_F(KortexDriverTest, checkAllExamplesSuccess)
{
    bool success = true;
    std::ostringstream failed_tests_message;
    failed_tests_message << "The following examples failed :" << std::endl;
    for (auto ex : examples)
    {
        bool example_success = waitForExample(ex.first);
        if (!example_success)
        {
            failed_tests_message << "\t- " << ex.first << std::endl;
        }
        success &= (example_success == ex.second);
    }
    ASSERT_TRUE(success) << failed_tests_message.str();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "KortexArmDriverInitTestsNode");
  testing::InitGoogleTest(&argc, argv);

  std::thread t([]{while(ros::ok()) ros::spin();});

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}