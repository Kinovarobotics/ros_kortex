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

#include "kortex_driver/non-generated/kortex_subscribers.h"

KortexSubscribers::KortexSubscribers(ros::NodeHandle& node_handle, Kinova::Api::Base::BaseClient* base):
m_node_handle(node_handle), m_base(base)
{
    std::string robot_name;
    ros::param::get("~robot_name", robot_name);
    m_joint_speeds_sub = m_node_handle.subscribe("in/joint_velocity", 1, &KortexSubscribers::new_joint_speeds_cb, this);
    m_twist_sub = m_node_handle.subscribe("in/cartesian_velocity", 1, &KortexSubscribers::new_twist_cb, this);
    m_clear_faults_sub = m_node_handle.subscribe("in/clear_faults", 1, &KortexSubscribers::clear_faults_cb, this);
    m_stop_sub = m_node_handle.subscribe("in/stop", 1, &KortexSubscribers::stop_cb, this);
    m_emergency_stop_sub = m_node_handle.subscribe("in/emergency_stop", 1, &KortexSubscribers::emergency_stop_cb, this);
}

KortexSubscribers::~KortexSubscribers()
{
}

void KortexSubscribers::new_joint_speeds_cb(const kortex_driver::Base_JointSpeeds& joint_speeds)
{
    Kinova::Api::Base::JointSpeeds speeds;
    kortex_driver::Base_JointSpeeds joint_speeds_in_rad(joint_speeds); // Since joint_speeds is const we need this copy

    // Convert radians in degrees
    for (unsigned int i = 0; i < joint_speeds.joint_speeds.size(); i++)
    {
        joint_speeds_in_rad.joint_speeds[i].value = KortexMathUtil::toDeg(joint_speeds.joint_speeds[i].value);
    }
    ToProtoData(joint_speeds_in_rad, &speeds);

    try
    {
        m_base->SendJointSpeedsCommand(speeds);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while sending joint speeds");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_DEBUG("Runtime exception detected while sending joint speeds!");
        ROS_DEBUG("%s", ex_runtime.what());
    }
}

void KortexSubscribers::new_twist_cb(const kortex_driver::TwistCommand& twist)
{
    Kinova::Api::Base::TwistCommand twist_command;
    ToProtoData(twist, &twist_command);

    // Convert radians to degrees
    twist_command.mutable_twist()->set_angular_x(KortexMathUtil::toDeg(twist_command.twist().angular_x()));
    twist_command.mutable_twist()->set_angular_y(KortexMathUtil::toDeg(twist_command.twist().angular_y()));
    twist_command.mutable_twist()->set_angular_z(KortexMathUtil::toDeg(twist_command.twist().angular_z()));

    try
    {
        m_base->SendTwistCommand(twist_command);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while sending twist command");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_DEBUG("Runtime exception detected while sending twist command!");
        ROS_DEBUG("%s", ex_runtime.what());
    }
}

void KortexSubscribers::clear_faults_cb(const std_msgs::Empty& dummy)
{
    try
    {
        m_base->ClearFaults();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while clearing the faults");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_DEBUG("Runtime exception detected while clearing the faults!");
        ROS_DEBUG("%s", ex_runtime.what());
    }
}

void KortexSubscribers::stop_cb(const std_msgs::Empty& dummy)
{
    try
    {
        m_base->Stop();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while clearing the faults");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_DEBUG("Runtime exception detected while clearing the faults!");
        ROS_DEBUG("%s", ex_runtime.what());
    }
}

void KortexSubscribers::emergency_stop_cb(const std_msgs::Empty& dummy)
{
    try
    {
        m_base->ApplyEmergencyStop();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while clearing the faults");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_DEBUG("Runtime exception detected while clearing the faults!");
        ROS_DEBUG("%s", ex_runtime.what());
    }
}

