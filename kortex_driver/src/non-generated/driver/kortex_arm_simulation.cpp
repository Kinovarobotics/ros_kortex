/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2020 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "kortex_driver/non-generated/kortex_arm_simulation.h"
#include "kortex_driver/ErrorCodes.h"
#include "kortex_driver/SubErrorCodes.h"
#include "kortex_driver/ActionNotification.h"
#include "kortex_driver/ActionEvent.h"

#include <set>

namespace 
{
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    static constexpr unsigned int FIRST_CREATED_ACTION_ID = 10000;
    static const std::set<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
}

KortexArmSimulation::KortexArmSimulation(ros::NodeHandle& node_handle): m_node_handle(node_handle),
                                                                        m_map_actions{},
                                                                        m_is_action_being_executed{false},
                                                                        m_action_preempted{false}
{
    // Namespacing and prefixing information
    ros::param::get("~robot_name", m_robot_name);
    ros::param::get("~prefix", m_prefix);

    // Arm and gripper information
    ros::param::get("~dof", m_degrees_of_freedom);
    ros::param::get("~arm", m_arm_name);
    ros::param::get("~joint_names", m_arm_joint_names);
    for (auto s : m_arm_joint_names)
    {
        s.insert(0, m_prefix);
    }
    ros::param::get("~gripper", m_gripper_name);
    if (IsGripperPresent())
    {
        ros::param::get("~gripper_joint_names", m_gripper_joint_names);
        for (auto s : m_gripper_joint_names)
        {
            s.insert(0, m_prefix);
        }
        ros::param::get("~gripper_joint_limits_max", m_gripper_joint_limits_max);
        ros::param::get("~gripper_joint_limits_min", m_gripper_joint_limits_min);
    }

    // Print out simulation configuration
    ROS_INFO("Simulating arm with following characteristics:");
    ROS_INFO("Arm type : %s", m_arm_name.c_str());
    ROS_INFO("Gripper type : %s", m_gripper_name.empty() ? "None" : m_gripper_name.c_str());
    ROS_INFO("Arm namespace : %s", m_robot_name.c_str());
    ROS_INFO("URDF prefix : %s", m_prefix.c_str());

    // Start MoveIt client
    m_moveit_arm_interface.reset(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
    if (IsGripperPresent())
    {
        m_moveit_gripper_interface.reset(new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP));   
    }

    // Create default actions
    CreateDefaultActions();

    // Create publishers
    m_pub_action_topic = m_node_handle.advertise<kortex_driver::ActionNotification>("action_topic", 1000);
}

KortexArmSimulation::~KortexArmSimulation()
{
}

std::unordered_map<uint32_t, kortex_driver::Action> KortexArmSimulation::GetActionsMap() const
{
    return m_map_actions;
}

kortex_driver::CreateAction::Response KortexArmSimulation::CreateAction(const kortex_driver::CreateAction::Request& req)
{
    auto new_action = req.input;
    unsigned int identifier = FIRST_CREATED_ACTION_ID;
    bool identifier_taken = true;
    // Find unique identifier for new action
    while (identifier_taken)
    {
        identifier_taken = m_map_actions.count(identifier) == 1;
        if (identifier_taken)
        {
            ++identifier;
        }
    }
    // Add Action to map if type is supported
    switch (new_action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::ActionType::REACH_POSE:
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::ActionType::TIME_DELAY:
            new_action.handle.identifier = identifier;
            new_action.handle.permission = 7;
            m_map_actions.emplace(std::make_pair(identifier, new_action));
            break;
        default:
            ROS_ERROR("Unsupported action type %d : could not create simulated action.", new_action.handle.action_type);
            break;
    }
    // Return ActionHandle for added action
    kortex_driver::CreateAction::Response response;
    response.output = new_action.handle;
    return response;
}

kortex_driver::ReadAction::Response KortexArmSimulation::ReadAction(const kortex_driver::ReadAction::Request& req)
{
    auto input = req.input;
    kortex_driver::ReadAction::Response response;
    auto it = m_map_actions.find(input.identifier);
    if (it != m_map_actions.end())
    {
        response.output = it->second;
    }
    return response;
}

kortex_driver::ReadAllActions::Response KortexArmSimulation::ReadAllActions(const kortex_driver::ReadAllActions::Request& req)
{
    auto input = req.input;
    kortex_driver::ReadAllActions::Response response;
    kortex_driver::ActionList action_list;
    for (auto a : m_map_actions)
    {
        // If requested action type is specified and matches iterated action's type, add it to the list
        if (input.action_type == 0 || input.action_type == a.second.handle.action_type)
        {
            action_list.action_list.push_back(a.second);
        }
        
    }
    response.output = action_list;
    return response;
}

kortex_driver::DeleteAction::Response KortexArmSimulation::DeleteAction(const kortex_driver::DeleteAction::Request& req)
{
    auto handle = req.input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(handle.identifier);
        if (it != m_map_actions.end())
        {
            m_map_actions.erase(it);
            ROS_INFO("Simulated action #%u properly deleted.", handle.identifier);
        }
        else
        {
            ROS_WARN("Could not find simulated action #%u to delete in actions map.", handle.identifier);
        }
    }
    else
    {
        ROS_ERROR("Cannot delete default simulated actions.");
    }
    kortex_driver::DeleteAction::Response response;
    return response;
}

kortex_driver::UpdateAction::Response KortexArmSimulation::UpdateAction(const kortex_driver::UpdateAction::Request& req)
{
    auto action = req.input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(action.handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(action.handle.identifier);
        if (it != m_map_actions.end())
        {
            if (it->second.handle.action_type == action.handle.action_type)
            {
                it->second = action;
                ROS_INFO("Simulated action #%u properly updated.", action.handle.identifier);
            }
            else
            {
                ROS_ERROR("Cannot update action with different type.");
            }
        }
        else
        {
            ROS_ERROR("Could not find simulated action #%u to update in actions map.", action.handle.identifier);
        }
    }
    else
    {
       ROS_ERROR("Cannot update default simulated actions."); 
    }
    kortex_driver::UpdateAction::Response response;
    return response;
}

kortex_driver::ExecuteActionFromReference::Response KortexArmSimulation::ExecuteActionFromReference(const kortex_driver::ExecuteActionFromReference::Request& req)
{
    auto input = req.input;
    kortex_driver::ExecuteActionFromReference::Response response;
    return response;
}

kortex_driver::ExecuteAction::Response KortexArmSimulation::ExecuteAction(const kortex_driver::ExecuteAction::Request& req)
{
    auto input = req.input;
    kortex_driver::ExecuteAction::Response response;
    return response;
}

kortex_driver::StopAction::Response KortexArmSimulation::StopAction(const kortex_driver::StopAction::Request& req)
{
    auto input = req.input;
    kortex_driver::StopAction::Response response;
    return response;
}

void KortexArmSimulation::CreateDefaultActions()
{
    kortex_driver::Action retract, home, zero;
    kortex_driver::ConstrainedJointAngles retract_angles, home_angles, zero_angles;
    // Retract
    retract.handle.identifier = 1;
    retract.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    retract.handle.permission = 7;
    retract.name = "Retract";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("retract");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        retract_angles.joint_angles.joint_angles.push_back(a);
    }
    retract.oneof_action_parameters.reach_joint_angles.push_back(retract_angles);
    // Home
    home.handle.identifier = 2;
    home.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    home.handle.permission = 7;
    home.name = "Home";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("home");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        home_angles.joint_angles.joint_angles.push_back(a);
    }
    home.oneof_action_parameters.reach_joint_angles.push_back(home_angles);
    // Zero
    zero.handle.identifier = 3;
    zero.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    zero.handle.permission = 7;
    zero.name = "Zero";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("vertical");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        zero_angles.joint_angles.joint_angles.push_back(a);
    }
    zero.oneof_action_parameters.reach_joint_angles.push_back(zero_angles);
    // Add actions
    m_map_actions.emplace(std::make_pair(retract.handle.identifier, retract));
    m_map_actions.emplace(std::make_pair(home.handle.identifier, home));
    m_map_actions.emplace(std::make_pair(zero.handle.identifier, zero));
}

void KortexArmSimulation::CancelAction()
{
    m_action_preempted = true;
    if (m_action_executor_thread.joinable())
    {
        m_action_executor_thread.join();
    }
    m_action_preempted = false;
}

void KortexArmSimulation::PlayAction(const kortex_driver::Action& action)
{
    kortex_driver::KortexError action_result;

    // Notify action started
    kortex_driver::ActionNotification start_notif;
    start_notif.handle = action.handle;
    start_notif.action_event = kortex_driver::ActionEvent::ACTION_START;
    m_pub_action_topic.publish(start_notif);
    m_is_action_being_executed = true;
    
    // Switch executor on the action type
    switch (action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
            action_result = ExecuteReachJointAngles(action);
            break;
        case kortex_driver::ActionType::REACH_POSE:
            action_result = ExecuteReachPose(action);
            break;
        case kortex_driver::ActionType::SEND_JOINT_SPEEDS:
            action_result = ExecuteSendJointSpeeds(action);
            break;
        case kortex_driver::ActionType::SEND_TWIST_COMMAND:
            action_result = ExecuteSendTwist(action);
            break;
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
            action_result = ExecuteSendGripperCommand(action);
            break;
        case kortex_driver::ActionType::TIME_DELAY:
            action_result = ExecuteTimeDelay(action);
            break;
        default:
            action_result.code = kortex_driver::ErrorCodes::ERROR_DEVICE;
            action_result.subCode = kortex_driver::SubErrorCodes::UNSUPPORTED_ACTION;
            break;
    }
    
    kortex_driver::ActionNotification end_notif;
    end_notif.handle = action.handle;
    // Action was cancelled by user
    if (m_action_preempted.load())
    {
        // Notify ACTION_ABORT
        end_notif.action_event = kortex_driver::ActionEvent::ACTION_ABORT;
        ROS_WARN("Action was aborted by user.");
    }
    // Action ended on its own
    else
    {
        if (action_result.code != kortex_driver::ErrorCodes::ERROR_NONE)
        {
            // Notify ACTION_ABORT
            end_notif.action_event = kortex_driver::ActionEvent::ACTION_ABORT;
            end_notif.abort_details = action_result.subCode;
            ROS_WARN("Action was failed : \nError code is %d\nSub-error code is %d\nError description is : %s", 
                        action_result.code,
                        action_result.subCode,
                        action_result.description.c_str());
        }
        else
        {
            // Notify ACTION_END
            end_notif.action_event = kortex_driver::ActionEvent::ACTION_END;
            ROS_WARN("Action ended.");
        }
    }
    m_pub_action_topic.publish(end_notif);
    
    m_is_action_being_executed = false;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteReachJointAngles(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteReachPose(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendJointSpeeds(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendTwist(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendGripperCommand(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteTimeDelay(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    return result;
}
