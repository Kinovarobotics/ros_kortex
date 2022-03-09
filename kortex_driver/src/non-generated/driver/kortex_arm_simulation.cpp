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
#include "kortex_driver/JointTrajectoryConstraintType.h"
#include "kortex_driver/GripperMode.h"
#include "kortex_driver/CartesianReferenceFrame.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "controller_manager_msgs/SwitchController.h"
#include "std_msgs/Float64.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>

#include <urdf/model.h>

#include <set>
#include <chrono>
#include <unordered_set>

namespace 
{
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    static constexpr unsigned int FIRST_CREATED_ACTION_ID = 10000;
    static const std::set<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
    static constexpr double JOINT_TRAJECTORY_TIMESTEP_SECONDS = 0.01;
    static constexpr double MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS = 0.001;
}

KortexArmSimulation::KortexArmSimulation(ros::NodeHandle& node_handle): m_node_handle(node_handle),
                                                                        m_map_actions{},
                                                                        m_is_action_being_executed{false},
                                                                        m_action_preempted{false},
                                                                        m_current_action_type{0},
                                                                        m_first_state_received{false},
                                                                        m_active_controller_type{ControllerType::kTrajectory}
{
    // Namespacing and prefixing information
    ros::param::get("~robot_name", m_robot_name);
    ros::param::get("~prefix", m_prefix);

    // Arm information
    urdf::Model model;
    model.initParam("/" + m_robot_name + "/robot_description");
    ros::param::get("~dof", m_degrees_of_freedom);
    ros::param::get("~arm", m_arm_name);
    ros::param::get("~joint_names", m_arm_joint_names);
    ros::param::get("~maximum_velocities", m_arm_velocity_max_limits);
    ros::param::get("~maximum_accelerations", m_arm_acceleration_max_limits);
    m_arm_joint_limits_min.resize(GetDOF());
    m_arm_joint_limits_max.resize(GetDOF());
    for (int i = 0; i < GetDOF(); i++)
    {
        auto joint = model.getJoint(m_arm_joint_names[i]);
        m_arm_joint_limits_min[i] = joint->limits->lower;
        m_arm_joint_limits_max[i] = joint->limits->upper;
    }

    // Cartesian Twist limits
    ros::param::get("~maximum_linear_velocity", m_max_cartesian_twist_linear);
    ros::param::get("~maximum_angular_velocity", m_max_cartesian_twist_angular);
    ros::param::get("~maximum_linear_acceleration", m_max_cartesian_acceleration_linear);
    ros::param::get("~maximum_angular_acceleration", m_max_cartesian_acceleration_angular);
    
    // Gripper information
    ros::param::get("~gripper", m_gripper_name);
    if (IsGripperPresent())
    {
        ros::param::get("~gripper_joint_names", m_gripper_joint_names);

        // The joint_states feedback uses alphabetical order for the indexes
        // If the gripper joint is before
        if (m_gripper_joint_names[0] < m_arm_joint_names[0])
        {
            m_gripper_joint_index = 0;
            m_first_arm_joint_index = 1;
        }
        // If the gripper joint is after the arm joints
        else
        {
            m_gripper_joint_index = GetDOF();
            m_first_arm_joint_index = 0;
        }

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

    // Building the KDL chain from the robot description
    // The chain goes from 'base_link' to 'tool_frame'
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        const std::string error_string("Failed to parse robot_description parameter to build the kinematic tree!"); 
        ROS_ERROR("%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    if (!tree.getChain(m_prefix + "base_link", m_prefix + "tool_frame", m_chain))
    {
        const std::string error_string("Failed to extract kinematic chain from parsed tree!"); 
        ROS_ERROR("%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
    m_ik_pos_solver.reset(new KDL::ChainIkSolverPos_NR(m_chain, *m_fk_solver, *m_ik_vel_solver));

    // Build the velocity profile for each joint using the max velocities and max accelerations
    for (int i = 0; i < GetDOF(); i++)
    {
        m_velocity_trap_profiles.push_back(KDL::VelocityProfile_Trap(m_arm_velocity_max_limits[i], m_arm_acceleration_max_limits[i]));
    }

    // Start MoveIt client
    m_moveit_arm_interface.reset(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
    if (IsGripperPresent())
    {
        m_moveit_gripper_interface.reset(new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP));   
    }

    // Create default actions
    CreateDefaultActions();

    // Create publishers and subscribers
    for (int i = 0; i < GetDOF(); i++)
    {
        m_pub_position_controllers.push_back(m_node_handle.advertise<std_msgs::Float64>(
            "/" + m_robot_name + "/" + m_prefix + "joint_" + std::to_string(i+1) + "_position_controller/command", 1000));
    }
    m_pub_action_topic = m_node_handle.advertise<kortex_driver::ActionNotification>("action_topic", 1000);
    m_sub_joint_state = m_node_handle.subscribe("/" + m_robot_name + "/" + "joint_states", 1, &KortexArmSimulation::cb_joint_states, this);
    m_feedback.actuators.resize(GetDOF());
    m_feedback.interconnect.oneof_tool_feedback.gripper_feedback.resize(1);
    m_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.resize(1);

    m_sub_joint_speeds = m_node_handle.subscribe("in/joint_velocity", 1, &KortexArmSimulation::new_joint_speeds_cb, this);
    m_sub_twist = m_node_handle.subscribe("in/cartesian_velocity", 1, &KortexArmSimulation::new_twist_cb, this);
    m_sub_clear_faults = m_node_handle.subscribe("in/clear_faults", 1, &KortexArmSimulation::clear_faults_cb, this);
    m_sub_stop = m_node_handle.subscribe("in/stop", 1, &KortexArmSimulation::stop_cb, this);
    m_sub_emergency_stop = m_node_handle.subscribe("in/emergency_stop", 1, &KortexArmSimulation::emergency_stop_cb, this);

    // Create service clients
    m_client_switch_controllers = m_node_handle.serviceClient<controller_manager_msgs::SwitchController>
        ("/" + m_robot_name + "/controller_manager/switch_controller");
    
    // Fill controllers'names
    m_trajectory_controllers_list.push_back(m_prefix + m_arm_name + "_joint_trajectory_controller"); // only one trajectory controller
    m_position_controllers_list.resize(GetDOF()); // one position controller per 
    std::generate(m_position_controllers_list.begin(), 
                    m_position_controllers_list.end(), 
                    [this]() -> std::string
                    {
                        static int i = 1;
                        return m_prefix + "joint_" + std::to_string(i++) + "_position_controller";
                    });

    // Initialize the velocity commands to null
    m_velocity_commands.resize(GetDOF());
    std::fill(m_velocity_commands.begin(), m_velocity_commands.end(), 0.0);

    // Create and connect action clients
    m_follow_joint_trajectory_action_client.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
        "/" + m_robot_name + "/" + m_prefix + m_arm_name + "_joint_trajectory_controller" + "/follow_joint_trajectory", true));
    m_follow_joint_trajectory_action_client->waitForServer();
    if (IsGripperPresent())
    {
        m_gripper_action_client.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(
            "/" + m_robot_name + "/" + m_prefix + m_gripper_name + "_gripper_controller" + "/gripper_cmd", true));
        m_gripper_action_client->waitForServer();
    }

    // Create usual ROS parameters
    m_node_handle.setParam("degrees_of_freedom", m_degrees_of_freedom);
    m_node_handle.setParam("is_gripper_present", IsGripperPresent());
    m_node_handle.setParam("gripper_joint_names", m_gripper_joint_names);
	m_node_handle.setParam("has_vision_module", false);
    m_node_handle.setParam("has_interconnect_module", false);
}

KortexArmSimulation::~KortexArmSimulation()
{
    JoinThreadAndCancelAction();
}

kortex_driver::BaseCyclic_Feedback KortexArmSimulation::GetFeedback()
{
    // If the feedback is not yet received, return now
    if (!m_first_state_received)
    {
        return m_feedback;
    }

    // Make a copy of current state
    sensor_msgs::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }

    // Fill joint angles feedback
    for (int i = 0; i < GetDOF(); i++)
    {
        m_feedback.actuators[i].position = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(current.position[m_first_arm_joint_index + i]));
        m_feedback.actuators[i].velocity = m_math_util.toDeg(current.velocity[m_first_arm_joint_index + i]);
    }

    // Calculate FK to get end effector pose
    auto frame = KDL::Frame();
    Eigen::VectorXd positions_eigen(GetDOF());
    for (int i = 0; i < GetDOF(); i++)
    {
        positions_eigen[i] = current.position[m_first_arm_joint_index + i];
    }
    KDL::JntArray current_kdl(GetDOF());
    current_kdl.data = positions_eigen;
    m_fk_solver->JntToCart(current_kdl, frame);
    m_feedback.base.tool_pose_x = frame.p.x();
    m_feedback.base.commanded_tool_pose_x = frame.p.x();
    m_feedback.base.tool_pose_y = frame.p.y();
    m_feedback.base.commanded_tool_pose_y = frame.p.y();
    m_feedback.base.tool_pose_z = frame.p.z();
    m_feedback.base.commanded_tool_pose_z = frame.p.z();
    double alpha, beta, gamma;
    frame.M.GetEulerZYX(alpha, beta, gamma);
    m_feedback.base.tool_pose_theta_x = m_math_util.toDeg(gamma);
    m_feedback.base.commanded_tool_pose_theta_x = m_math_util.toDeg(gamma);
    m_feedback.base.tool_pose_theta_y = m_math_util.toDeg(beta);
    m_feedback.base.commanded_tool_pose_theta_y = m_math_util.toDeg(beta);
    m_feedback.base.tool_pose_theta_z = m_math_util.toDeg(alpha);
    m_feedback.base.commanded_tool_pose_theta_z = m_math_util.toDeg(alpha);

    // Fill gripper information
    if (IsGripperPresent())
    {
        // Gripper index is right after last actuator and is expressed in % in the base feedback (not in absolute position like in joint_states)
        m_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position = 100.0 * m_math_util.relative_position_from_absolute(current.position[m_gripper_joint_index], m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]);
    }

    return m_feedback;
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
    
    return kortex_driver::DeleteAction::Response();
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

    return kortex_driver::UpdateAction::Response();
}

kortex_driver::ExecuteActionFromReference::Response KortexArmSimulation::ExecuteActionFromReference(const kortex_driver::ExecuteActionFromReference::Request& req)
{
    auto handle = req.input;
    auto it = m_map_actions.find(handle.identifier);
    if (it != m_map_actions.end())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, it->second);
    }
    else
    {
        ROS_ERROR("Could not find action with given identifier %d", handle.identifier);
    }
    
    return kortex_driver::ExecuteActionFromReference::Response();
}

kortex_driver::ExecuteAction::Response KortexArmSimulation::ExecuteAction(const kortex_driver::ExecuteAction::Request& req)
{
    auto action = req.input;
    // Add Action to map if type is supported
    switch (action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::ActionType::REACH_POSE:
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::ActionType::TIME_DELAY:
            JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
            m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
            break;
        default:
            ROS_ERROR("Unsupported action type %d : could not execute simulated action.", action.handle.action_type);
            break;
    }

    return kortex_driver::ExecuteAction::Response();
}

kortex_driver::StopAction::Response KortexArmSimulation::StopAction(const kortex_driver::StopAction::Request& req)
{
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }

    m_follow_joint_trajectory_action_client->cancelAllGoals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->cancelAllGoals();
    }
    
    return kortex_driver::StopAction::Response();
}

kortex_driver::PlayCartesianTrajectory::Response KortexArmSimulation::PlayCartesianTrajectory(const kortex_driver::PlayCartesianTrajectory::Request& req)
{
    auto constrained_pose = req.input;
    kortex_driver::Action action;
    action.name = "PlayCartesianTrajectory";
    action.handle.action_type = kortex_driver::ActionType::REACH_POSE;
    action.oneof_action_parameters.reach_pose.push_back(constrained_pose);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::PlayCartesianTrajectory::Response();
}

kortex_driver::SendTwistCommand::Response KortexArmSimulation::SendTwistCommand(const kortex_driver::SendTwistCommand::Request& req)
{
    auto twist_command = req.input;
    kortex_driver::Action action;
    action.name = "SendTwistCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_TWIST_COMMAND;
    action.oneof_action_parameters.send_twist_command.push_back(twist_command);

    // Convert orientations to rad
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_x);
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_y);
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_z);
    
    // Fill the twist command
    m_twist_command = twist_command.twist;

    // If we are already executing twist control, don't cancel the thread
    if (m_current_action_type != kortex_driver::ActionType::SEND_TWIST_COMMAND)
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    }

    return kortex_driver::SendTwistCommand::Response();
}

kortex_driver::PlayJointTrajectory::Response KortexArmSimulation::PlayJointTrajectory(const kortex_driver::PlayJointTrajectory::Request& req)
{
    auto constrained_joint_angles = req.input;
    kortex_driver::Action action;
    action.name = "PlayJointTrajectory";
    action.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    action.oneof_action_parameters.reach_joint_angles.push_back(constrained_joint_angles);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::PlayJointTrajectory::Response();
}

kortex_driver::SendJointSpeedsCommand::Response KortexArmSimulation::SendJointSpeedsCommand(const kortex_driver::SendJointSpeedsCommand::Request& req)
{
    auto joint_speeds = req.input;
    kortex_driver::Action action;
    action.name = "SendJointSpeedsCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_JOINT_SPEEDS;

    // Convert radians in degrees
    for (unsigned int i = 0; i < joint_speeds.joint_speeds.size(); i++)
    {
        joint_speeds.joint_speeds[i].value = m_math_util.toDeg(joint_speeds.joint_speeds[i].value);
    }
    action.oneof_action_parameters.send_joint_speeds.push_back(joint_speeds);

    // Fill the velocity commands vector
    int n = 0;
    std::generate(m_velocity_commands.begin(),
        m_velocity_commands.end(), 
        [this, &action, &n]() -> double
        {
            return m_math_util.toRad(action.oneof_action_parameters.send_joint_speeds[0].joint_speeds[n++].value);
        });

    // If we are already executing joint speed control, don't cancel the thread
    if (m_current_action_type != kortex_driver::ActionType::SEND_JOINT_SPEEDS)
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    }
    
    return kortex_driver::SendJointSpeedsCommand::Response();
}

kortex_driver::SendGripperCommand::Response KortexArmSimulation::SendGripperCommand(const kortex_driver::SendGripperCommand::Request& req)
{
    auto gripper_command = req.input;
    kortex_driver::Action action;
    action.name = "GripperCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_GRIPPER_COMMAND;
    action.oneof_action_parameters.send_gripper_command.push_back(gripper_command);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::SendGripperCommand::Response();
}

kortex_driver::Stop::Response KortexArmSimulation::Stop(const kortex_driver::Stop::Request& req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->cancelAllGoals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->cancelAllGoals();
    }
    return kortex_driver::Stop::Response();
}

kortex_driver::ApplyEmergencyStop::Response KortexArmSimulation::ApplyEmergencyStop(const kortex_driver::ApplyEmergencyStop::Request& req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->cancelAllGoals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->cancelAllGoals();
    }
    return kortex_driver::ApplyEmergencyStop::Response();
}

void KortexArmSimulation::cb_joint_states(const sensor_msgs::JointState& state)
{
    const std::lock_guard<std::mutex> lock(m_state_mutex);
    m_first_state_received = true;
    m_current_state = state;
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
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
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
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
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
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        zero_angles.joint_angles.joint_angles.push_back(a);
    }
    zero.oneof_action_parameters.reach_joint_angles.push_back(zero_angles);
    // Add actions
    m_map_actions.emplace(std::make_pair(retract.handle.identifier, retract));
    m_map_actions.emplace(std::make_pair(home.handle.identifier, home));
    m_map_actions.emplace(std::make_pair(zero.handle.identifier, zero));
}

bool KortexArmSimulation::SwitchControllerType(ControllerType new_type)
{
    bool success = true;
    controller_manager_msgs::SwitchController service;
    service.request.strictness = service.request.STRICT;
    if (m_active_controller_type != new_type)
    {
        // Set the controllers we want to switch to
        switch (new_type)
        {
            case ControllerType::kTrajectory:
                service.request.start_controllers = m_trajectory_controllers_list;
                service.request.stop_controllers = m_position_controllers_list;
                break;
            case ControllerType::kIndividual:
                service.request.start_controllers = m_position_controllers_list;
                service.request.stop_controllers = m_trajectory_controllers_list;
                break;
            default:
                ROS_ERROR("Kortex arm simulator : Unsupported controller type %d", int(new_type));
                return false;
        }

        // Call the service
        if (!m_client_switch_controllers.call(service))
        {
            ROS_ERROR("Failed to call the service for switching controllers");
            success = false;
        }
        else
        {
            success = service.response.ok;
        }

        // Update active type if the switch was successful
        if (success)
        {
            m_active_controller_type = new_type;
        }
    }

    return success;
}

kortex_driver::KortexError KortexArmSimulation::FillKortexError(uint32_t code, uint32_t subCode, const std::string& description) const
{
    kortex_driver::KortexError error;
    error.code = code;
    error.subCode = subCode;
    error.description = description;
    return error;
}

void KortexArmSimulation::JoinThreadAndCancelAction()
{
    // Tell the thread to stop and join it
    m_action_preempted = true;
    if (m_action_executor_thread.joinable())
    {
        m_action_executor_thread.join();
    }
    m_current_action_type = 0;
    m_action_preempted = false;
}

void KortexArmSimulation::PlayAction(const kortex_driver::Action& action)
{
    auto action_result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE, kortex_driver::SubErrorCodes::SUB_ERROR_NONE);

    // Notify action started
    kortex_driver::ActionNotification start_notif;
    start_notif.handle = action.handle;
    start_notif.action_event = kortex_driver::ActionEvent::ACTION_START;
    m_pub_action_topic.publish(start_notif);
    m_is_action_being_executed = true;
    m_current_action_type = action.handle.action_type;
    
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
            action_result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE, kortex_driver::SubErrorCodes::UNSUPPORTED_ACTION);
            break;
    }
    
    // Oddly enough, gripper actions don't send notifications through Kortex API when they end
    if (action.handle.action_type != kortex_driver::ActionType::SEND_GRIPPER_COMMAND)
    {
        kortex_driver::ActionNotification end_notif;
        end_notif.handle = action.handle;
        // Action was cancelled by user and is not a velocity command
        if (m_action_preempted.load() && action.handle.action_type != kortex_driver::ActionType::SEND_JOINT_SPEEDS)
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
            }
        }
        m_pub_action_topic.publish(end_notif);
    }
    
    m_is_action_being_executed = false;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteReachJointAngles(const kortex_driver::Action& action)
{
    auto result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE, kortex_driver::SubErrorCodes::SUB_ERROR_NONE);
    if (action.oneof_action_parameters.reach_joint_angles.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                    kortex_driver::SubErrorCodes::INVALID_PARAM,
                                    "Error playing joint angles action : action is malformed.");
    }
    auto constrained_joint_angles = action.oneof_action_parameters.reach_joint_angles[0];
    if (constrained_joint_angles.joint_angles.joint_angles.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint angles action : action contains " + std::to_string(constrained_joint_angles.joint_angles.joint_angles.size()) + " joint angles but arm has " + std::to_string(GetDOF()));
    }

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kTrajectory))
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint angles action : simulated trajectory controller could not be switched to.");
    }

    // Initialize trajectory object
    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = m_prefix + "base_link";
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        const std::string joint_name = m_prefix + "joint_" + std::to_string(i+1); //joint names are 1-based
        traj.joint_names.push_back(joint_name);
    }

    // Get current position
    sensor_msgs::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }

    // Transform kortex structure to trajectory_msgs to fill endpoint structure
    trajectory_msgs::JointTrajectoryPoint endpoint;
    std::unordered_set<int> limited_joints; // joints limited in range
    int degrees_of_freedom = constrained_joint_angles.joint_angles.joint_angles.size();
    if (degrees_of_freedom == 6)
    {
        limited_joints = {1,2,4};
    }
    else if (degrees_of_freedom == 7)
    {
        limited_joints = {1,3,5};
    }
    else 
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Unsupported number of joints, expected 6 or 7");
    }
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        // If the current actuator has turned on itself many times, we need the endpoint to follow that trend too
        int n_turns = 0;
        double rad_wrapped_goal;
        if (limited_joints.count(i))
        {
            rad_wrapped_goal = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(constrained_joint_angles.joint_angles.joint_angles[i].value));
        }
        else
        {
            rad_wrapped_goal = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(constrained_joint_angles.joint_angles.joint_angles[i].value), n_turns);
        }
        endpoint.positions.push_back(rad_wrapped_goal + double(n_turns) * 2.0 * M_PI);
        endpoint.velocities.push_back(0.0);
        endpoint.accelerations.push_back(0.0);
    }

    // Calculate velocity profiles to know how much time this trajectory must last
    switch (constrained_joint_angles.constraint.type)
    {
        // If the duration is supplied, set the duration of the velocity profiles with that value
        case kortex_driver::JointTrajectoryConstraintType::JOINT_CONSTRAINT_DURATION:
        {
            // Error check on the given duration
            if (constrained_joint_angles.constraint.value <= 0.0f)
            {
                return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Invalid duration constraint : it has to be higher than 0.0!");
            }
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], constrained_joint_angles.constraint.value);
            }
            endpoint.time_from_start = ros::Duration(constrained_joint_angles.constraint.value);
            ROS_DEBUG("Using supplied duration : %2.2f", constrained_joint_angles.constraint.value);
            break;
        }
        // If a max velocity is supplied for each joint, we need to find the limiting duration with this velocity constraint
        case kortex_driver::JointTrajectoryConstraintType::JOINT_CONSTRAINT_SPEED:
        {
            // Error check on the given velocity
            float max_velocity = m_math_util.toRad(constrained_joint_angles.constraint.value);
            if (max_velocity <= 0.0f)
            {
                return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Invalid velocity constraint : it has to be higher than 0.0!");
            }
            // Find the limiting duration with given velocity
            double max_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                double velocity_ratio = std::min(1.0, double(max_velocity)/m_arm_velocity_max_limits[i]);
                m_velocity_trap_profiles[i].SetProfileVelocity(current.position[m_first_arm_joint_index + i], endpoint.positions[i], velocity_ratio);
                max_duration = std::max(max_duration, m_velocity_trap_profiles[i].Duration());
                ROS_DEBUG("Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.position[m_first_arm_joint_index + i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            ROS_DEBUG("max_duration is : %2.2f", max_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], max_duration);
            }
            endpoint.time_from_start = ros::Duration(max_duration);
            break;
        }
        default:
        {
            // Find the optimal duration based on actual velocity and acceleration limits
            double optimal_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfile(current.position[m_first_arm_joint_index + i], endpoint.positions[i]);
                optimal_duration = std::max(optimal_duration, m_velocity_trap_profiles[i].Duration());
                ROS_DEBUG("Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.position[m_first_arm_joint_index + i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            ROS_DEBUG("optimal_duration is : %2.2f", optimal_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], optimal_duration);
            }
            endpoint.time_from_start = ros::Duration(optimal_duration);
            break;
        }
    }

    // Copy velocity profile data into trajectory using JOINT_TRAJECTORY_TIMESTEP_SECONDS timesteps
    // For each timestep
    for (double t = JOINT_TRAJECTORY_TIMESTEP_SECONDS; t < m_velocity_trap_profiles[0].Duration(); t += JOINT_TRAJECTORY_TIMESTEP_SECONDS)
    {
        // Create trajectory point
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(t);
        // Add position, velocity, acceleration from each velocity profile
        for (int i = 0; i < GetDOF(); i++)
        {
            p.positions.push_back(m_velocity_trap_profiles[i].Pos(t));
            p.velocities.push_back(m_velocity_trap_profiles[i].Vel(t));
            p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(t));
        }
        // Add trajectory point to goal
        traj.points.push_back(p);
    }
    // Finally, add endpoint to trajectory
    // Add position, velocity, acceleration from each velocity profile
    trajectory_msgs::JointTrajectoryPoint p;
    p.time_from_start = ros::Duration(m_velocity_trap_profiles[0].Duration());
    for (int i = 0; i < GetDOF(); i++)
    {
        p.positions.push_back(m_velocity_trap_profiles[i].Pos(m_velocity_trap_profiles[i].Duration()));
        p.velocities.push_back(m_velocity_trap_profiles[i].Vel(m_velocity_trap_profiles[i].Duration()));
        p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(m_velocity_trap_profiles[i].Duration()));
    }
    // Add trajectory point to goal
    traj.points.push_back(p);

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }
    
    // Send goal
    control_msgs::FollowJointTrajectoryActionGoal goal;
    traj.header.stamp = ros::Time::now();
    goal.goal.trajectory = traj;
    m_follow_joint_trajectory_action_client->sendGoal(goal.goal);

    // Wait for goal to be done, or for preempt to be called (check every 100ms)
    while(!m_action_preempted.load())
    {
        if (m_follow_joint_trajectory_action_client->waitForResult(ros::Duration(0.1f)))
        {
            // Sometimes an error is thrown related to a bad cast in a ros::time structure inside the SimpleActionClient
            // See https://answers.ros.org/question/209452/exception-thrown-while-processing-service-call-time-is-out-of-dual-32-bit-range/
            // If this error happens here we just send the goal again with an updated timestamp
            auto status = m_follow_joint_trajectory_action_client->getResult();
            if (status->error_string == "Time is out of dual 32-bit range")
            {
                traj.header.stamp = ros::Time::now();
                goal.goal.trajectory = traj;
                m_follow_joint_trajectory_action_client->sendGoal(goal.goal);
            }
            else
            {
                break;
            }
        }
    }

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_follow_joint_trajectory_action_client->cancelAllGoals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_follow_joint_trajectory_action_client->getResult();
        if (status->error_code != status->SUCCESSFUL)
        {
            result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::SubErrorCodes::INVALID_PARAM,
                                        status->error_string);
        }
    }
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteReachPose(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.reach_pose.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing pose action : action is malformed.");
    }
    auto constrained_pose = action.oneof_action_parameters.reach_pose[0];

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kTrajectory))
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::METHOD_FAILED,
                                "Error playing pose action : simulated trajectory controller could not be switched to.");
    }

    // Get current position
    sensor_msgs::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Get Start frame
    // For the Rotation part : ThetaX = gamma, ThetaY = beta, ThetaZ = alpha 
    auto start = KDL::Frame();
    Eigen::VectorXd positions_eigen(m_degrees_of_freedom);
    for (int i = 0; i < GetDOF(); i++)
    {
        positions_eigen[i] = current.position[m_first_arm_joint_index + i];
    }
    KDL::JntArray current_kdl(GetDOF());
    current_kdl.data = positions_eigen;
    m_fk_solver->JntToCart(current_kdl, start);

    {
    ROS_DEBUG("START FRAME :");
    ROS_DEBUG("X=%2.4f Y=%2.4f Z=%2.4f", start.p[0], start.p[1], start.p[2]);
    double sa, sb, sg; start.M.GetEulerZYX(sa, sb, sg);
    ROS_DEBUG("ALPHA=%2.4f BETA=%2.4f GAMMA=%2.4f", m_math_util.toDeg(sa), m_math_util.toDeg(sb), m_math_util.toDeg(sg));
    KDL::Vector axis;
    ROS_DEBUG("start rot = %2.4f", start.M.GetRotAngle(axis));
    }

    // Get End frame
    auto end_pos = KDL::Vector(constrained_pose.target_pose.x, constrained_pose.target_pose.y, constrained_pose.target_pose.z);
    auto end_rot = KDL::Rotation::EulerZYX(m_math_util.toRad(constrained_pose.target_pose.theta_z), m_math_util.toRad(constrained_pose.target_pose.theta_y), m_math_util.toRad(constrained_pose.target_pose.theta_x));
    KDL::Frame end(end_rot, end_pos);

    {
    ROS_DEBUG("END FRAME :");
    ROS_DEBUG("X=%2.4f Y=%2.4f Z=%2.4f", end_pos[0], end_pos[1], end_pos[2]);
    double ea, eb, eg; end_rot.GetEulerZYX(ea, eb, eg);
    ROS_DEBUG("ALPHA=%2.4f BETA=%2.4f GAMMA=%2.4f", m_math_util.toDeg(ea), m_math_util.toDeg(eb), m_math_util.toDeg(eg));
    KDL::Vector axis;
    ROS_DEBUG("end rot = %2.4f", end_rot.GetRotAngle(axis));
    }

    // If different speed limits than the default ones are provided, use them instead
    float translation_speed_limit = m_max_cartesian_twist_linear;
    float rotation_speed_limit = m_max_cartesian_twist_angular;
    if (!constrained_pose.constraint.oneof_type.speed.empty())
    {
        // If a max velocity is supplied for each joint, we need to find the limiting duration with this velocity constraint
        translation_speed_limit = std::min(constrained_pose.constraint.oneof_type.speed[0].translation, translation_speed_limit);
        rotation_speed_limit = std::min(constrained_pose.constraint.oneof_type.speed[0].orientation, rotation_speed_limit);
    }

    // Calculate norm of translation movement and minimum duration to move this amount given max translation speed
    double delta_pos = (end_pos - start.p).Norm();
    double minimum_translation_duration = delta_pos / m_max_cartesian_twist_linear; // in seconds

    // Calculate angle variation of rotation movement and minimum duration to move this amount given max rotation speed
    KDL::Vector axis; // we need to create this variable to access the RotAngle for start and end frames'rotation components
    KDL::Rotation dR = end_rot * start.M.Inverse();
    double delta_rot = dR.GetRotAngle(axis);
    double minimum_rotation_duration = delta_rot / m_max_cartesian_twist_angular; // in seconds

    ROS_INFO("trans : %2.4f rot : %2.4f", minimum_translation_duration, minimum_rotation_duration);

    // The default value for the duration will be the longer duration of the two
    double duration = std::max(minimum_translation_duration, minimum_rotation_duration);

    // eq_radius is chosen here to make it so translations and rotations are normalised
    // Here is a good explanation for it : https://github.com/zakharov/BRICS_RN/blob/master/navigation_trajectory_common/include/navigation_trajectory_common/Conversions.h#L358-L382
    float eq_radius = translation_speed_limit / rotation_speed_limit;

    // Create Path_Line object
    // I know this is ugly but the RotationalInterpolation object is mandatory and needs to be created as such
    KDL::Path_Line line(start, end, new KDL::RotationalInterpolation_SingleAxis(), eq_radius);

    // Create a trapezoidal velocity profile for the Cartesian trajectory to parametrize it in time
    KDL::VelocityProfile_Trap velocity_profile(std::min(translation_speed_limit, rotation_speed_limit), m_max_cartesian_acceleration_linear);
    velocity_profile.SetProfile(0.0, line.PathLength());
    duration = std::max(duration, velocity_profile.Duration());

    // If duration is not supplied, use the one we just calculated
    // If the duration is supplied, simply use it
    if (!constrained_pose.constraint.oneof_type.duration.empty())
    {
        double supplied_duration = constrained_pose.constraint.oneof_type.duration[0];
        if (duration > supplied_duration)
        {
            ROS_WARN("Cannot use supplied duration %2.4f because the minimum duration based on velocity limits is %2.4f",
                        supplied_duration,
                        duration);
        }
        duration = std::max(duration, supplied_duration);
    }

    // Set the velocity profile duration
    velocity_profile.SetProfileDuration(0.0, line.PathLength(), duration);
    KDL::Trajectory_Segment segment(&line, &velocity_profile, false);
    ROS_DEBUG("Duration of trajectory will be %2.4f seconds", duration);

    // Initialize trajectory object
    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = m_prefix + "base_link";
    traj.header.stamp = ros::Time::now();
    for (int i = 0; i < GetDOF(); i++)
    {
        const std::string joint_name = m_prefix + "joint_" + std::to_string(i+1); //joint names are 1-based
        traj.joint_names.push_back(joint_name);
    }

    // Fill trajectory object
    KDL::JntArray previous = current_kdl; // Position i - 1, initialise to starting angles
    KDL::JntArray current_joints(GetDOF()); // Position i
    for (float t = JOINT_TRAJECTORY_TIMESTEP_SECONDS; t < duration; t += JOINT_TRAJECTORY_TIMESTEP_SECONDS)
    {
        // First get the next Cartesian position and twists
        auto pos = segment.Pos(t);
        
        // Position is enough to have a smooth trajectory it seems but if eventually vel and acc are useful somehow this is how to get them:
        // auto vel = segment.Vel(t);
        // auto acc = segment.Acc(t);

        // Create trajectory point
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(t);

        // Use inverse IK solver
        int code = m_ik_pos_solver->CartToJnt(previous, pos, current_joints);
        if (code != m_ik_pos_solver->E_NOERROR)
        {
            ROS_ERROR("IK ERROR CODE = %d", code);
        }

        for (int i = 0; i < GetDOF(); i++)
        {
            p.positions.push_back(current_joints(i));
        }
        
        // Add trajectory point to goal
        traj.points.push_back(p);
        previous = current_joints;
    }

    // Add last point
    trajectory_msgs::JointTrajectoryPoint p;
    p.time_from_start = ros::Duration(segment.Duration());
    auto pos = segment.Pos(segment.Duration());
    int code = m_ik_pos_solver->CartToJnt(previous, pos, current_joints);
    for (int i = 0; i < GetDOF(); i++)
    {
        p.positions.push_back(current_joints(i));
    }
    
    // Add trajectory point to goal
    traj.points.push_back(p);

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }
    
    // Send goal
    control_msgs::FollowJointTrajectoryActionGoal goal;
    goal.goal.trajectory = traj;
    m_follow_joint_trajectory_action_client->sendGoal(goal.goal);

    // Wait for goal to be done, or for preempt to be called (check every 10ms)
    while(!m_action_preempted.load() && !m_follow_joint_trajectory_action_client->waitForResult(ros::Duration(0.01f))) {}

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_follow_joint_trajectory_action_client->cancelAllGoals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_follow_joint_trajectory_action_client->getResult();
        if (status->error_code != status->SUCCESSFUL)
        {
            result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::SubErrorCodes::INVALID_PARAM,
                                        status->error_string);
        }
    }
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendJointSpeeds(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_joint_speeds.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joints speeds : action is malformed.");
    }
    auto joint_speeds = action.oneof_action_parameters.send_joint_speeds[0];
    if (joint_speeds.joint_speeds.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint speeds action : action contains " + std::to_string(joint_speeds.joint_speeds.size()) + " joint speeds but arm has " + std::to_string(GetDOF()));
    }

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kIndividual))
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint speeds action : simulated positions controllers could not be switched to.");
    }

    // Get current position
    sensor_msgs::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Initialise commands
    std::vector<double> commands(GetDOF(), 0.0); // in rad
    for (int i = 0; i < GetDOF(); i++)
    {
        commands[i] = current.position[m_first_arm_joint_index + i];
    }
    std::vector<double> previous_commands = commands; // in rad
    std::vector<double> velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<double> previous_velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<bool> stopped(GetDOF(), false);
    
    // While we're not done
    while (true)
    {
        // If action is preempted, set the velocities to 0
        if (m_action_preempted.load())
        {
            std::fill(m_velocity_commands.begin(), m_velocity_commands.end(), 0.0);
        }
        // For each joint
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate real position increment
            // This helps to know if we hit joint limits or if we stopped
            {
            const std::lock_guard<std::mutex> lock(m_state_mutex);
            current = m_current_state;
            }

            // Calculate permitted velocity command because we don't have infinite acceleration
            double vel_delta = m_velocity_commands[i] - previous_velocity_commands[i];
            double max_delta = std::copysign(m_arm_acceleration_max_limits[i] * JOINT_TRAJECTORY_TIMESTEP_SECONDS, vel_delta);
            
            // If the velocity change is within acceleration limits for this timestep
            double velocity_command;
            if (fabs(vel_delta) < fabs(max_delta))
            {
                velocity_command = m_velocity_commands[i];
            }
            // If we cannot instantly accelerate to this velocity
            else
            {
                velocity_command = previous_velocity_commands[i] + max_delta;
            }

            // Cap to the velocity limit for the joint
            velocity_command = std::copysign(std::min(fabs(velocity_command), double(fabs(m_arm_velocity_max_limits[i]))), velocity_command);

            // Check if velocity command is in fact too small
            if (fabs(velocity_command) < MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS)
            {
                velocity_command = 0.0;
                commands[i] = current.position[m_first_arm_joint_index + i];
                stopped[i] = true;
            }
            // Else calculate the position increment and send it
            else
            {
                commands[i] = previous_commands[i] + velocity_command * JOINT_TRAJECTORY_TIMESTEP_SECONDS;
                stopped[i] = false;

                // Cap the command to the joint limit
                if (m_arm_joint_limits_min[i] != 0.0 && commands[i] < m_arm_joint_limits_min[i])
                {
                    commands[i] = m_arm_joint_limits_min[i];
                    velocity_command = std::max(velocity_command, 0.0);
                }
                else if (m_arm_joint_limits_max[i] != 0.0 && commands[i] > m_arm_joint_limits_max[i])
                {
                    commands[i] = m_arm_joint_limits_max[i];
                    velocity_command = std::min(velocity_command, 0.0);
                }

                // Send the position increments to the controllers
                std_msgs::Float64 message;
                message.data = commands[i];
                m_pub_position_controllers[i].publish(message);
            }

            // Remember actual command as previous and actual velocity command as previous
            previous_commands[i] = commands[i];
            previous_velocity_commands[i] = velocity_command;
        }

        // Sleep for TIMESTEP
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000*JOINT_TRAJECTORY_TIMESTEP_SECONDS)));

        // If the action is preempted and we're back to zero velocity, we're done here
        if (m_action_preempted.load())
        {
            // Check if all joints are stopped and break if yes
            if (std::all_of(stopped.begin(), stopped.end(), [](bool b){return b;}))
            {
                break;
            }
        }
    }

    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendTwist(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_twist_command.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing twist action : action is malformed.");
    }
    auto twist = action.oneof_action_parameters.send_twist_command[0];

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kIndividual))
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint speeds action : simulated positions controllers could not be switched to.");
    }

    // Only mixed frame is supported in simulation
    if (twist.reference_frame != kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing twist action : only mixed frame is supported in simulation.");
    }

    // Get current position
    sensor_msgs::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Initialise commands
    std::vector<double> commands(GetDOF(), 0.0); // in rad
    for (int i = 0; i < GetDOF(); i++)
    {
        commands[i] = current.position[m_first_arm_joint_index + i];
    }
    std::vector<double> previous_commands = commands; // in rad
    std::vector<double> previous_velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<bool> stopped(GetDOF(), false);
    kortex_driver::Twist twist_command = m_twist_command;
    kortex_driver::Twist previous_twist_command;

    // While we're not done
    while (ros::ok())
    {
        // If action is preempted, set the velocities to 0
        if (m_action_preempted.load())
        {
            m_twist_command = kortex_driver::Twist();
        }

        // Calculate actual twist command considering max linear and angular accelerations
        double max_linear_twist_delta = JOINT_TRAJECTORY_TIMESTEP_SECONDS * m_max_cartesian_acceleration_linear;
        double max_angular_twist_delta = JOINT_TRAJECTORY_TIMESTEP_SECONDS * m_max_cartesian_acceleration_angular;
        kortex_driver::Twist delta_twist = m_math_util.substractTwists(m_twist_command, previous_twist_command);

        // If the velocity change is within acceleration limits for this timestep
        if (fabs(delta_twist.linear_x) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_x = m_twist_command.linear_x;
        }
        // If we cannot instantly accelerate to this velocity
        else
        {
            twist_command.linear_x = previous_twist_command.linear_x + std::copysign(max_linear_twist_delta, delta_twist.linear_x);
        }
        // same for linear_y
        if (fabs(delta_twist.linear_y) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_y = m_twist_command.linear_y;
        }
        else
        {
            twist_command.linear_y = previous_twist_command.linear_y + std::copysign(max_linear_twist_delta, delta_twist.linear_y);
        }
        // same for linear_z
        if (fabs(delta_twist.linear_z) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_z = m_twist_command.linear_z;
        }
        else
        {
            twist_command.linear_z = previous_twist_command.linear_z + std::copysign(max_linear_twist_delta, delta_twist.linear_z);
        }
        // same for angular_x
        if (fabs(delta_twist.angular_x) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_x = m_twist_command.angular_x;
        }
        else
        {
            twist_command.angular_x = previous_twist_command.angular_x + std::copysign(max_angular_twist_delta, delta_twist.angular_x);
        }
        // same for angular_y
        if (fabs(delta_twist.angular_y) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_y = m_twist_command.angular_y;
        }
        else
        {
            twist_command.angular_y = previous_twist_command.angular_y + std::copysign(max_angular_twist_delta, delta_twist.angular_y);
        }
        // same for angular_z
        if (fabs(delta_twist.angular_z) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_z = m_twist_command.angular_z;
        }
        else
        {
            twist_command.angular_z = previous_twist_command.angular_z + std::copysign(max_angular_twist_delta, delta_twist.angular_z);
        }

        // Cap to the velocity limit
        twist_command.linear_x = std::copysign(std::min(fabs(twist_command.linear_x), fabs(m_max_cartesian_twist_linear)), twist_command.linear_x);
        twist_command.linear_y = std::copysign(std::min(fabs(twist_command.linear_y), fabs(m_max_cartesian_twist_linear)), twist_command.linear_y);
        twist_command.linear_z = std::copysign(std::min(fabs(twist_command.linear_z), fabs(m_max_cartesian_twist_linear)), twist_command.linear_z);
        twist_command.angular_x = std::copysign(std::min(fabs(twist_command.angular_x), fabs(m_max_cartesian_twist_angular)), twist_command.angular_x);
        twist_command.angular_y = std::copysign(std::min(fabs(twist_command.angular_y), fabs(m_max_cartesian_twist_angular)), twist_command.angular_y);
        twist_command.angular_z = std::copysign(std::min(fabs(twist_command.angular_z), fabs(m_max_cartesian_twist_angular)), twist_command.angular_z);

        // Fill current joint position commands KDL structure
        KDL::JntArray commands_kdl(GetDOF());
        Eigen::VectorXd commands_eigen(GetDOF());
        for (int i = 0; i < GetDOF(); i++)
        {
            commands_eigen[i] = commands[i];
        }
        commands_kdl.data = commands_eigen;
        
        // Fill KDL Twist structure with Kortex twist
        KDL::Twist twist_kdl = KDL::Twist(KDL::Vector(twist_command.linear_x, twist_command.linear_y, twist_command.linear_z),
                                            KDL::Vector(twist_command.angular_x, twist_command.angular_y, twist_command.angular_z));
        
        // Call IK and fill joint velocity commands
        KDL::JntArray joint_velocities(GetDOF());
        int ik_result = m_ik_vel_solver->CartToJnt(commands_kdl, twist_kdl, joint_velocities);
        if (ik_result != m_ik_vel_solver->E_NOERROR)
        {
            ROS_WARN("IK ERROR = %d", ik_result);
        }
        
        // We need to know if the joint velocities have to be adjusted, and by what ratio
        double ratio = 1.0;
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate permitted velocity command because we don't have infinite acceleration
            double vel_delta = joint_velocities(i) - previous_velocity_commands[i];
            double max_delta = std::copysign(m_arm_acceleration_max_limits[i] * JOINT_TRAJECTORY_TIMESTEP_SECONDS, vel_delta);
            
            // If we cannot instantly accelerate to this velocity
            if (fabs(vel_delta) > fabs(max_delta))
            {
                ratio = std::max(ratio, fabs(vel_delta / max_delta));
            }
        }

        // Command the velocities
        // For each joint
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate position increment
            commands[i] = previous_commands[i] + joint_velocities(i) / ratio * JOINT_TRAJECTORY_TIMESTEP_SECONDS;

            // Cap the command to the joint limit
            if (m_arm_joint_limits_min[i] != 0.0 && commands[i] < 0.0)
            {
                commands[i] = std::max(m_arm_joint_limits_min[i], commands[i]);
            }
            else if (m_arm_joint_limits_max[i] != 0.0 && commands[i] > 0.0)
            {
                commands[i] = std::min(m_arm_joint_limits_max[i], commands[i]);
            }
            
            // Send the position increments to the controllers
            std_msgs::Float64 message;
            message.data = commands[i];
            m_pub_position_controllers[i].publish(message);

            // Check if joint is stopped or not
            stopped[i] = fabs(joint_velocities(i)) < MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS;

            // Remember actual command as previous
            previous_commands[i] = commands[i];
            previous_velocity_commands[i] = joint_velocities(i);
        }

        // Put current values to previous
        previous_twist_command = twist_command;

        // Sleep for TIMESTEP
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000*JOINT_TRAJECTORY_TIMESTEP_SECONDS)));

        // If the action is preempted and we're back to zero velocity, we're done here
        if (m_action_preempted.load())
        {
            // Check if all joints are stopped and break if yes
            if (std::all_of(stopped.begin(), stopped.end(), [](bool b){return b;}))
            {
                break;
            }
        }
    }

    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendGripperCommand(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_gripper_command.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing gripper command action : action is malformed.");
    }
    auto gripper_command = action.oneof_action_parameters.send_gripper_command[0];

    if (gripper_command.gripper.finger.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing gripper command action : there must be exactly one finger");
    }

    if (gripper_command.mode != kortex_driver::GripperMode::GRIPPER_POSITION)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                            kortex_driver::SubErrorCodes::UNSUPPORTED_ACTION,
                            "Error playing gripper command action : gripper mode " + std::to_string(gripper_command.mode) + " is not supported; only position is.");
    }

    // The incoming command is relative [0,1] and we need to put it in absolute unit [m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]]:
    double absolute_gripper_command = m_math_util.absolute_position_from_relative(gripper_command.gripper.finger[0].value, m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]);

    // Create the goal
    control_msgs::GripperCommandGoal goal;
    goal.command.position = absolute_gripper_command;

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }

    // Send goal
    m_gripper_action_client->sendGoal(goal);

    // Wait for goal to be done, or for preempt to be called (check every 10ms)
    while(!m_action_preempted.load() && !m_gripper_action_client->waitForResult(ros::Duration(0.01f))) {}

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_gripper_action_client->cancelAllGoals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_gripper_action_client->getResult();
        
        if (!status->reached_goal)
        {
            result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::SubErrorCodes::METHOD_FAILED,
                                        "The gripper command failed during execution.");
        }
    }
    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteTimeDelay(const kortex_driver::Action& action)
{
    auto result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE,
                                kortex_driver::SubErrorCodes::SUB_ERROR_NONE);
    if (!action.oneof_action_parameters.delay.empty())
    {
        auto start = std::chrono::system_clock::now();
        uint32_t delay_seconds = action.oneof_action_parameters.delay[0].duration;
        // While not preempted and duration not elapsed
        while (!m_action_preempted.load() && (std::chrono::system_clock::now() - start) < std::chrono::seconds(delay_seconds))
        {
            // sleep a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else
    {
        result.code = kortex_driver::ErrorCodes::ERROR_DEVICE;
        result.subCode = kortex_driver::SubErrorCodes::INVALID_PARAM;
        result.description = "Error playing time delay action : action is malformed.";
    }
    return result;
}

void KortexArmSimulation::new_joint_speeds_cb(const kortex_driver::Base_JointSpeeds& joint_speeds)
{
    kortex_driver::SendJointSpeedsCommandRequest req;
    req.input = joint_speeds;
    SendJointSpeedsCommand(req);
}

void KortexArmSimulation::new_twist_cb(const kortex_driver::TwistCommand& twist)
{
    // TODO Implement
}

void KortexArmSimulation::clear_faults_cb(const std_msgs::Empty& empty)
{
    // does nothing
}

void KortexArmSimulation::stop_cb(const std_msgs::Empty& empty)
{
    Stop(kortex_driver::StopRequest());
}

void KortexArmSimulation::emergency_stop_cb(const std_msgs::Empty& empty)
{
    ApplyEmergencyStop(kortex_driver::ApplyEmergencyStopRequest());
}
