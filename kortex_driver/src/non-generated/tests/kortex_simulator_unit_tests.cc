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

        // Create dummy action
        dummy_action.name = "MyDummyAction";
        dummy_action.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
        dummy_action.handle.permission = 7;
        kortex_driver::ConstrainedJointAngles angles;
        for (int i = 0; i < m_simulator->GetDOF(); i++)
        {
            kortex_driver::JointAngle angle;
            angle.joint_identifier = i;
            angle.value = 10.0f*i;
            angles.joint_angles.joint_angles.push_back(angle);
        }
        dummy_action.oneof_action_parameters.reach_joint_angles.push_back(angles);
        }

    void TearDown() override 
    {
    }

    void CompareReachJointAnglesActions(const kortex_driver::Action& a1, const kortex_driver::Action& a2, bool same)
    {
        ASSERT_EQ(a1.oneof_action_parameters.reach_joint_angles.size(), a2.oneof_action_parameters.reach_joint_angles.size());
        auto angles1 = a1.oneof_action_parameters.reach_joint_angles[0];
        auto angles2 = a2.oneof_action_parameters.reach_joint_angles[0];
        ASSERT_EQ(angles1.joint_angles.joint_angles.size(), angles2.joint_angles.joint_angles.size());
        for (int i = 0; i < angles1.joint_angles.joint_angles.size() && same; i++)
        {
            ASSERT_EQ(same, angles1.joint_angles.joint_angles.at(i) == angles2.joint_angles.joint_angles.at(i));
        }
    }

    kortex_driver::Action dummy_action;
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

// Tests UpdateAction so default actions are not updated
TEST_F(KortexSimulatorTest, UpdateDefaultActions)
{
    static const std::vector<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
    // Make sure the action cannot be updated
    kortex_driver::UpdateAction::Request req;
    for (unsigned int i : DEFAULT_ACTIONS_IDENTIFIERS)
    {
        dummy_action.handle.identifier = i;
        req.input = dummy_action;
        m_simulator->UpdateAction(req);
        auto actions_map = m_simulator->GetActionsMap();
        ASSERT_EQ(actions_map.count(i), 1);
        CompareReachJointAnglesActions(actions_map[i], dummy_action, false);
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
    dummy_action.name = name;
    req.input = dummy_action;
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
    kortex_driver::CreateAction::Request req;
    dummy_action.oneof_action_parameters.reach_joint_angles.clear();
    dummy_action.handle.action_type = kortex_driver::ActionType::SEND_JOINT_SPEEDS;
    kortex_driver::Base_JointSpeeds speeds;
    for (int i = 0; i < m_simulator->GetDOF(); i++)
    {
        kortex_driver::JointSpeed speed;
        speed.joint_identifier = i;
        speed.value = 10.0f*i;
        speeds.joint_speeds.push_back(speed);
    }
    dummy_action.oneof_action_parameters.send_joint_speeds.push_back(speeds);
    req.input = dummy_action;
    auto res = m_simulator->CreateAction(req);
    auto handle = res.output;
    auto actions_map = m_simulator->GetActionsMap();

    // Make sure the action was added to the map
    ASSERT_EQ(actions_map.count(handle.identifier), 0);
}

// Tests UpdateAction on existing and non-existing actions
TEST_F(KortexSimulatorTest, UpdateAction)
{
    // Create Action at first
    kortex_driver::CreateAction::Request req;
    req.input = dummy_action;
    auto res = m_simulator->CreateAction(req);
    auto handle = res.output;
    auto actions_map = m_simulator->GetActionsMap();

    // Make sure the action was added to the map
    ASSERT_EQ(actions_map.count(handle.identifier), 1);
    ASSERT_EQ(actions_map[handle.identifier].handle.action_type, kortex_driver::ActionType::REACH_JOINT_ANGLES);

    // Modify and update the Action
    dummy_action.name = "MyUpdatedName";
    dummy_action.handle.identifier = handle.identifier;
    dummy_action.oneof_action_parameters.reach_joint_angles[0].joint_angles.joint_angles[3].value = 0.0f;
    kortex_driver::UpdateAction::Request update_req;
    update_req.input = dummy_action;
    m_simulator->UpdateAction(update_req);
    actions_map = m_simulator->GetActionsMap();
    ASSERT_EQ(actions_map.count(handle.identifier), 1);
    ASSERT_EQ(dummy_action.name, actions_map[handle.identifier].name);
    CompareReachJointAnglesActions(dummy_action, actions_map[handle.identifier], true);

    // Modify and update the Action with a different type
    kortex_driver::Action wrong_type_action;
    wrong_type_action.name = "WrongType";
    wrong_type_action.handle.identifier = handle.identifier;
    wrong_type_action.handle.action_type = kortex_driver::ActionType::SEND_JOINT_SPEEDS;
    wrong_type_action.oneof_action_parameters.reach_joint_angles.clear();
    wrong_type_action.oneof_action_parameters.send_joint_speeds.push_back(kortex_driver::Base_JointSpeeds());
    update_req.input = wrong_type_action;
    m_simulator->UpdateAction(update_req);
    actions_map = m_simulator->GetActionsMap();
    ASSERT_EQ(actions_map.count(handle.identifier), 1);
    ASSERT_EQ(dummy_action.name, actions_map[handle.identifier].name);
    ASSERT_EQ(kortex_driver::ActionType::REACH_JOINT_ANGLES, actions_map[handle.identifier].handle.action_type);
}
