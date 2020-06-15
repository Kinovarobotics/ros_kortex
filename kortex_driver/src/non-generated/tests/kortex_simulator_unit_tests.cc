#include <ros/ros.h>
#include <kortex_driver/non-generated/kortex_arm_driver.h>
#include <kortex_driver/non-generated/kortex_arm_simulation.h>
#include <gtest/gtest.h>
#include <urdf/model.h>

class KortexSimulatorTest : public ::testing::Test {
 protected:
  
  void SetUp() override 
    {
        // Create Simulator
        m_simulator.reset(new KortexArmSimulation(n));
    }

  void TearDown() override 
    {
    }

  ros::NodeHandle n;
  std::unique_ptr<KortexArmSimulation> m_simulator;

};

// Make sure after initialisation the default actions are
// Tests ReadAllActions at the same time
TEST_F(KortexSimulatorTest, DefaultActions)
{
    auto actions = m_simulator->GetActionsMap();
    bool retract, home, zero, other = false;
    for (auto a : actions)
    {
        if (a.second.name == "Retract") retract = true;
        else if (a.second.name == "Home") home = true;
        else if (a.second.name == "Zero") zero = true;
        else other = true;
    }
    ASSERT_TRUE(retract);
    ASSERT_TRUE(home);
    ASSERT_TRUE(zero);
    ASSERT_FALSE(other);
}

// Tests DeleteAction so default actions are not deleted
TEST_F(KortexSimulatorTest, DeleteDefaultActions)
{
    static const std::vector<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
    // Make sure the action can be deleted properly
    kortex_driver::DeleteAction::Request req;
    kortex_driver::ActionHandle handle;
    for (unsigned int i : DEFAULT_ACTIONS_IDENTIFIERS)
    {
        handle.identifier = i;
        req.input = handle;
        m_simulator->DeleteAction(req);
        auto actions_map = m_simulator->GetActionsMap();
        ASSERT_EQ(actions_map.count(i), 1);
    }
}

// Tests ReadAllActions
TEST_F(KortexSimulatorTest, ReadAllActions)
{
    // Test for 3 known actions (default) of this type in the map
    kortex_driver::ReadAllActions::Request req;
    kortex_driver::RequestedActionType type;
    type.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    req.input = type;
    auto res = m_simulator->ReadAllActions(req);
    auto action_list = res.output;
    ASSERT_EQ(action_list.action_list.size(), 3); // Number of default actions

    // Test for 0 known actions of this type in the map
    type.action_type = kortex_driver::ActionType::REACH_POSE;
    req.input = type;
    res = m_simulator->ReadAllActions(req);
    action_list = res.output;
    ASSERT_TRUE(action_list.action_list.empty());
}

// Tests CreateAction handler for a supported Action, and DeleteAction
TEST_F(KortexSimulatorTest, CreateSupportedAction)
{
    static const std::string name = "MyNewAction";

    kortex_driver::CreateAction::Request req;
    kortex_driver::Action action;
    action.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    action.name = name;
    kortex_driver::ConstrainedJointAngles angles;
    for (int i = 0; i < 7; i++)
    {
        kortex_driver::JointAngle angle;
        angle.joint_identifier = i;
        angle.value = 10.0f*i;
        angles.joint_angles.joint_angles.push_back(angle);
    }
    action.oneof_action_parameters.reach_joint_angles.push_back(angles);
    req.input = action;
    auto res = m_simulator->CreateAction(req);
    auto handle = res.output;
    auto actions_map = m_simulator->GetActionsMap();

    // Make sure the action was added to the map
    ASSERT_EQ(actions_map.count(handle.identifier), 1);
    ASSERT_EQ(actions_map[handle.identifier].name, name);
    ASSERT_EQ(actions_map[handle.identifier].handle.action_type, kortex_driver::ActionType::REACH_JOINT_ANGLES);

    // Make sure the action can be deleted properly
    kortex_driver::DeleteAction::Request del_req;
    del_req.input = handle;
    m_simulator->DeleteAction(del_req);
    actions_map = m_simulator->GetActionsMap();
    ASSERT_EQ(actions_map.count(handle.identifier), 0);
}

// Tests CreateAction handler for an unsupported Action
TEST_F(KortexSimulatorTest, CreateUnsupportedAction)
{
    static const std::string name = "MyNewAction";

    kortex_driver::CreateAction::Request req;
    kortex_driver::Action action;
    action.handle.action_type = kortex_driver::ActionType::SEND_JOINT_SPEEDS;
    action.name = name;
    kortex_driver::Base_JointSpeeds speeds;
    for (int i = 0; i < 7; i++)
    {
        kortex_driver::JointSpeed speed;
        speed.joint_identifier = i;
        speed.value = 10.0f*i;
        speeds.joint_speeds.push_back(speed);
    }
    action.oneof_action_parameters.send_joint_speeds.push_back(speeds);
    req.input = action;
    auto res = m_simulator->CreateAction(req);
    auto handle = res.output;
    auto actions_map = m_simulator->GetActionsMap();

    // Make sure the action was added to the map
    ASSERT_EQ(actions_map.count(handle.identifier), 0);
}
