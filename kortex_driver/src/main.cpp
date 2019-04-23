/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
 * This file has been auto-generated and should not be modified.
 */
 
#include "node.h"
#include "math_util.h"

#include <sensor_msgs/JointState.h>
#include <sstream>

#define JOINT_COUNT 7

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BaseServices");
    
    uint32_t cyclic_data_rate = 100;

    ros::NodeHandle n;
    bool valid_ip = false;

    if(argc > 2)
    {
        ROS_INFO("Connecting to IP = %s - node refresh rate = %s", argv[1], argv[2]);
        
        //Converting the second parameter(the cyclic rate) to an unsigned int variable.
        stringstream tempRate;
        tempRate << argv[2];
        tempRate >> cyclic_data_rate;
        if(tempRate.fail() || tempRate.bad())
        {
            ROS_INFO("ERROR - Bad error rate, shutting down the node...");
            ros::shutdown();
            return 0;
        }
    }
    else
    {
        ROS_INFO("You need to provide an IP adresse as the first parameter and a cycle rate(Hertz) as the second parameter. ex: rosrun package node 192.168.1.1 100");
        ros::shutdown();
        return 0;
    }
    
    BaseServices services_object(argv[1], n);

    ros::ServiceServer serviceSetDeviceID = n.advertiseService("SetDeviceID", &BaseServices::SetDeviceID, &services_object);

    ros::ServiceServer serviceRefresh = n.advertiseService("Refresh", &BaseServices::Refresh, &services_object);
    ros::ServiceServer serviceRefreshCommand = n.advertiseService("RefreshCommand", &BaseServices::RefreshCommand, &services_object);
    ros::ServiceServer serviceRefreshFeedback = n.advertiseService("RefreshFeedback", &BaseServices::RefreshFeedback, &services_object);
    ros::ServiceServer serviceRefreshCustomData = n.advertiseService("RefreshCustomData", &BaseServices::RefreshCustomData, &services_object);
    ros::ServiceServer serviceCreateUserProfile = n.advertiseService("CreateUserProfile", &BaseServices::CreateUserProfile, &services_object);
    ros::ServiceServer serviceUpdateUserProfile = n.advertiseService("UpdateUserProfile", &BaseServices::UpdateUserProfile, &services_object);
    ros::ServiceServer serviceReadUserProfile = n.advertiseService("ReadUserProfile", &BaseServices::ReadUserProfile, &services_object);
    ros::ServiceServer serviceDeleteUserProfile = n.advertiseService("DeleteUserProfile", &BaseServices::DeleteUserProfile, &services_object);
    ros::ServiceServer serviceReadAllUserProfiles = n.advertiseService("ReadAllUserProfiles", &BaseServices::ReadAllUserProfiles, &services_object);
    ros::ServiceServer serviceReadAllUsers = n.advertiseService("ReadAllUsers", &BaseServices::ReadAllUsers, &services_object);
    ros::ServiceServer serviceChangePassword = n.advertiseService("ChangePassword", &BaseServices::ChangePassword, &services_object);
    ros::ServiceServer serviceCreateSequence = n.advertiseService("CreateSequence", &BaseServices::CreateSequence, &services_object);
    ros::ServiceServer serviceUpdateSequence = n.advertiseService("UpdateSequence", &BaseServices::UpdateSequence, &services_object);
    ros::ServiceServer serviceReadSequence = n.advertiseService("ReadSequence", &BaseServices::ReadSequence, &services_object);
    ros::ServiceServer serviceDeleteSequence = n.advertiseService("DeleteSequence", &BaseServices::DeleteSequence, &services_object);
    ros::ServiceServer serviceReadAllSequences = n.advertiseService("ReadAllSequences", &BaseServices::ReadAllSequences, &services_object);
    ros::ServiceServer serviceDeleteSequenceTask = n.advertiseService("DeleteSequenceTask", &BaseServices::DeleteSequenceTask, &services_object);
    ros::ServiceServer serviceDeleteAllSequenceTasks = n.advertiseService("DeleteAllSequenceTasks", &BaseServices::DeleteAllSequenceTasks, &services_object);
    ros::ServiceServer servicePlaySequence = n.advertiseService("PlaySequence", &BaseServices::PlaySequence, &services_object);
    ros::ServiceServer servicePlayAdvancedSequence = n.advertiseService("PlayAdvancedSequence", &BaseServices::PlayAdvancedSequence, &services_object);
    ros::ServiceServer serviceStopSequence = n.advertiseService("StopSequence", &BaseServices::StopSequence, &services_object);
    ros::ServiceServer servicePauseSequence = n.advertiseService("PauseSequence", &BaseServices::PauseSequence, &services_object);
    ros::ServiceServer serviceResumeSequence = n.advertiseService("ResumeSequence", &BaseServices::ResumeSequence, &services_object);
    ros::ServiceServer serviceCreateProtectionZone = n.advertiseService("CreateProtectionZone", &BaseServices::CreateProtectionZone, &services_object);
    ros::ServiceServer serviceUpdateProtectionZone = n.advertiseService("UpdateProtectionZone", &BaseServices::UpdateProtectionZone, &services_object);
    ros::ServiceServer serviceReadProtectionZone = n.advertiseService("ReadProtectionZone", &BaseServices::ReadProtectionZone, &services_object);
    ros::ServiceServer serviceDeleteProtectionZone = n.advertiseService("DeleteProtectionZone", &BaseServices::DeleteProtectionZone, &services_object);
    ros::ServiceServer serviceReadAllProtectionZones = n.advertiseService("ReadAllProtectionZones", &BaseServices::ReadAllProtectionZones, &services_object);
    ros::ServiceServer serviceCreateMapping = n.advertiseService("CreateMapping", &BaseServices::CreateMapping, &services_object);
    ros::ServiceServer serviceReadMapping = n.advertiseService("ReadMapping", &BaseServices::ReadMapping, &services_object);
    ros::ServiceServer serviceReadAllMappings = n.advertiseService("ReadAllMappings", &BaseServices::ReadAllMappings, &services_object);
    ros::ServiceServer serviceCreateMap = n.advertiseService("CreateMap", &BaseServices::CreateMap, &services_object);
    ros::ServiceServer serviceReadAllMaps = n.advertiseService("ReadAllMaps", &BaseServices::ReadAllMaps, &services_object);
    ros::ServiceServer serviceActivateMap = n.advertiseService("ActivateMap", &BaseServices::ActivateMap, &services_object);
    ros::ServiceServer serviceCreateAction = n.advertiseService("CreateAction", &BaseServices::CreateAction, &services_object);
    ros::ServiceServer serviceReadAction = n.advertiseService("ReadAction", &BaseServices::ReadAction, &services_object);
    ros::ServiceServer serviceReadAllActions = n.advertiseService("ReadAllActions", &BaseServices::ReadAllActions, &services_object);
    ros::ServiceServer serviceDeleteAction = n.advertiseService("DeleteAction", &BaseServices::DeleteAction, &services_object);
    ros::ServiceServer serviceUpdateAction = n.advertiseService("UpdateAction", &BaseServices::UpdateAction, &services_object);
    ros::ServiceServer serviceExecuteActionFromReference = n.advertiseService("ExecuteActionFromReference", &BaseServices::ExecuteActionFromReference, &services_object);
    ros::ServiceServer serviceExecuteAction = n.advertiseService("ExecuteAction", &BaseServices::ExecuteAction, &services_object);
    ros::ServiceServer servicePauseAction = n.advertiseService("PauseAction", &BaseServices::PauseAction, &services_object);
    ros::ServiceServer serviceStopAction = n.advertiseService("StopAction", &BaseServices::StopAction, &services_object);
    ros::ServiceServer serviceResumeAction = n.advertiseService("ResumeAction", &BaseServices::ResumeAction, &services_object);
    ros::ServiceServer serviceGetIPv4Configuration = n.advertiseService("GetIPv4Configuration", &BaseServices::GetIPv4Configuration, &services_object);
    ros::ServiceServer serviceSetIPv4Configuration = n.advertiseService("SetIPv4Configuration", &BaseServices::SetIPv4Configuration, &services_object);
    ros::ServiceServer serviceSetCommunicationInterfaceEnable = n.advertiseService("SetCommunicationInterfaceEnable", &BaseServices::SetCommunicationInterfaceEnable, &services_object);
    ros::ServiceServer serviceIsCommunicationInterfaceEnable = n.advertiseService("IsCommunicationInterfaceEnable", &BaseServices::IsCommunicationInterfaceEnable, &services_object);
    ros::ServiceServer serviceGetAvailableWifi = n.advertiseService("GetAvailableWifi", &BaseServices::GetAvailableWifi, &services_object);
    ros::ServiceServer serviceGetWifiInformation = n.advertiseService("GetWifiInformation", &BaseServices::GetWifiInformation, &services_object);
    ros::ServiceServer serviceAddWifiConfiguration = n.advertiseService("AddWifiConfiguration", &BaseServices::AddWifiConfiguration, &services_object);
    ros::ServiceServer serviceDeleteWifiConfiguration = n.advertiseService("DeleteWifiConfiguration", &BaseServices::DeleteWifiConfiguration, &services_object);
    ros::ServiceServer serviceGetAllConfiguredWifis = n.advertiseService("GetAllConfiguredWifis", &BaseServices::GetAllConfiguredWifis, &services_object);
    ros::ServiceServer serviceConnectWifi = n.advertiseService("ConnectWifi", &BaseServices::ConnectWifi, &services_object);
    ros::ServiceServer serviceDisconnectWifi = n.advertiseService("DisconnectWifi", &BaseServices::DisconnectWifi, &services_object);
    ros::ServiceServer serviceGetConnectedWifiInformation = n.advertiseService("GetConnectedWifiInformation", &BaseServices::GetConnectedWifiInformation, &services_object);
    ros::ServiceServer serviceUnsubscribe = n.advertiseService("Unsubscribe", &BaseServices::Unsubscribe, &services_object);
    ros::ServiceServer serviceOnNotificationConfigurationChangeTopic = n.advertiseService("OnNotificationConfigurationChangeTopic", &BaseServices::OnNotificationConfigurationChangeTopic, &services_object);
    ros::ServiceServer serviceOnNotificationMappingInfoTopic = n.advertiseService("OnNotificationMappingInfoTopic", &BaseServices::OnNotificationMappingInfoTopic, &services_object);
    ros::ServiceServer serviceOnNotificationControlModeTopic = n.advertiseService("OnNotificationControlModeTopic", &BaseServices::OnNotificationControlModeTopic, &services_object);
    ros::ServiceServer serviceOnNotificationOperatingModeTopic = n.advertiseService("OnNotificationOperatingModeTopic", &BaseServices::OnNotificationOperatingModeTopic, &services_object);
    ros::ServiceServer serviceOnNotificationSequenceInfoTopic = n.advertiseService("OnNotificationSequenceInfoTopic", &BaseServices::OnNotificationSequenceInfoTopic, &services_object);
    ros::ServiceServer serviceOnNotificationProtectionZoneTopic = n.advertiseService("OnNotificationProtectionZoneTopic", &BaseServices::OnNotificationProtectionZoneTopic, &services_object);
    ros::ServiceServer serviceOnNotificationUserTopic = n.advertiseService("OnNotificationUserTopic", &BaseServices::OnNotificationUserTopic, &services_object);
    ros::ServiceServer serviceOnNotificationControllerTopic = n.advertiseService("OnNotificationControllerTopic", &BaseServices::OnNotificationControllerTopic, &services_object);
    ros::ServiceServer serviceOnNotificationActionTopic = n.advertiseService("OnNotificationActionTopic", &BaseServices::OnNotificationActionTopic, &services_object);
    ros::ServiceServer serviceOnNotificationRobotEventTopic = n.advertiseService("OnNotificationRobotEventTopic", &BaseServices::OnNotificationRobotEventTopic, &services_object);
    ros::ServiceServer serviceGetFwdKinematics = n.advertiseService("GetFwdKinematics", &BaseServices::GetFwdKinematics, &services_object);
    ros::ServiceServer servicePlayCartesianTrajectory = n.advertiseService("PlayCartesianTrajectory", &BaseServices::PlayCartesianTrajectory, &services_object);
    ros::ServiceServer servicePlayCartesianTrajectoryPosition = n.advertiseService("PlayCartesianTrajectoryPosition", &BaseServices::PlayCartesianTrajectoryPosition, &services_object);
    ros::ServiceServer servicePlayCartesianTrajectoryOrientation = n.advertiseService("PlayCartesianTrajectoryOrientation", &BaseServices::PlayCartesianTrajectoryOrientation, &services_object);
    ros::ServiceServer servicePause = n.advertiseService("Pause", &BaseServices::Pause, &services_object);
    ros::ServiceServer serviceResume = n.advertiseService("Resume", &BaseServices::Resume, &services_object);
    ros::ServiceServer serviceGetMeasuredCartesianPose = n.advertiseService("GetMeasuredCartesianPose", &BaseServices::GetMeasuredCartesianPose, &services_object);
    ros::ServiceServer serviceGetCommandedCartesianPose = n.advertiseService("GetCommandedCartesianPose", &BaseServices::GetCommandedCartesianPose, &services_object);
    ros::ServiceServer serviceGetTargetedCartesianPose = n.advertiseService("GetTargetedCartesianPose", &BaseServices::GetTargetedCartesianPose, &services_object);
    ros::ServiceServer serviceSendTwistCommand = n.advertiseService("SendTwistCommand", &BaseServices::SendTwistCommand, &services_object);
    ros::ServiceServer serviceGetMeasuredTwist = n.advertiseService("GetMeasuredTwist", &BaseServices::GetMeasuredTwist, &services_object);
    ros::ServiceServer serviceGetCommandedTwist = n.advertiseService("GetCommandedTwist", &BaseServices::GetCommandedTwist, &services_object);
    ros::ServiceServer servicePlayJointTrajectory = n.advertiseService("PlayJointTrajectory", &BaseServices::PlayJointTrajectory, &services_object);
    ros::ServiceServer servicePlaySelectedJointTrajectory = n.advertiseService("PlaySelectedJointTrajectory", &BaseServices::PlaySelectedJointTrajectory, &services_object);
    ros::ServiceServer serviceGetMeasuredJointAngles = n.advertiseService("GetMeasuredJointAngles", &BaseServices::GetMeasuredJointAngles, &services_object);
    ros::ServiceServer serviceGetCommandedJointAngles = n.advertiseService("GetCommandedJointAngles", &BaseServices::GetCommandedJointAngles, &services_object);
    ros::ServiceServer serviceSendJointSpeedsCommmand = n.advertiseService("SendJointSpeedsCommmand", &BaseServices::SendJointSpeedsCommmand, &services_object);
    ros::ServiceServer serviceSendSelectedJointSpeedCommand = n.advertiseService("SendSelectedJointSpeedCommand", &BaseServices::SendSelectedJointSpeedCommand, &services_object);
    ros::ServiceServer serviceGetMeasuredJointSpeeds = n.advertiseService("GetMeasuredJointSpeeds", &BaseServices::GetMeasuredJointSpeeds, &services_object);
    ros::ServiceServer serviceGetCommandedJointSpeeds = n.advertiseService("GetCommandedJointSpeeds", &BaseServices::GetCommandedJointSpeeds, &services_object);
    ros::ServiceServer serviceSendGripperCommand = n.advertiseService("SendGripperCommand", &BaseServices::SendGripperCommand, &services_object);
    ros::ServiceServer serviceGetMeasuredGripperMovement = n.advertiseService("GetMeasuredGripperMovement", &BaseServices::GetMeasuredGripperMovement, &services_object);
    ros::ServiceServer serviceGetCommandedGripperMovement = n.advertiseService("GetCommandedGripperMovement", &BaseServices::GetCommandedGripperMovement, &services_object);
    ros::ServiceServer serviceSetAdmittance = n.advertiseService("SetAdmittance", &BaseServices::SetAdmittance, &services_object);
    ros::ServiceServer serviceSetTwistWrenchReferenceFrame = n.advertiseService("SetTwistWrenchReferenceFrame", &BaseServices::SetTwistWrenchReferenceFrame, &services_object);
    ros::ServiceServer serviceSetOperatingMode = n.advertiseService("SetOperatingMode", &BaseServices::SetOperatingMode, &services_object);
    ros::ServiceServer serviceApplyEmergencyStop = n.advertiseService("ApplyEmergencyStop", &BaseServices::ApplyEmergencyStop, &services_object);
    ros::ServiceServer serviceClearFaults = n.advertiseService("ClearFaults", &BaseServices::ClearFaults, &services_object);
    ros::ServiceServer serviceGetActiveMap = n.advertiseService("GetActiveMap", &BaseServices::GetActiveMap, &services_object);
    ros::ServiceServer serviceGetControlMode = n.advertiseService("GetControlMode", &BaseServices::GetControlMode, &services_object);
    ros::ServiceServer serviceGetOperatingMode = n.advertiseService("GetOperatingMode", &BaseServices::GetOperatingMode, &services_object);
    ros::ServiceServer serviceSetServoingMode = n.advertiseService("SetServoingMode", &BaseServices::SetServoingMode, &services_object);
    ros::ServiceServer serviceGetServoingMode = n.advertiseService("GetServoingMode", &BaseServices::GetServoingMode, &services_object);
    ros::ServiceServer serviceOnNotificationServoingModeTopic = n.advertiseService("OnNotificationServoingModeTopic", &BaseServices::OnNotificationServoingModeTopic, &services_object);
    ros::ServiceServer serviceGetSequenceState = n.advertiseService("GetSequenceState", &BaseServices::GetSequenceState, &services_object);
    ros::ServiceServer serviceGetProtectionZoneState = n.advertiseService("GetProtectionZoneState", &BaseServices::GetProtectionZoneState, &services_object);
    ros::ServiceServer serviceGetActionExecutionState = n.advertiseService("GetActionExecutionState", &BaseServices::GetActionExecutionState, &services_object);
    ros::ServiceServer serviceRestoreFactorySettings = n.advertiseService("RestoreFactorySettings", &BaseServices::RestoreFactorySettings, &services_object);
    ros::ServiceServer serviceRestoreNetworkFactorySettings = n.advertiseService("RestoreNetworkFactorySettings", &BaseServices::RestoreNetworkFactorySettings, &services_object);
    ros::ServiceServer serviceReboot = n.advertiseService("Reboot", &BaseServices::Reboot, &services_object);
    ros::ServiceServer serviceOnNotificationFactoryTopic = n.advertiseService("OnNotificationFactoryTopic", &BaseServices::OnNotificationFactoryTopic, &services_object);
    ros::ServiceServer serviceGetAllConnectedControllers = n.advertiseService("GetAllConnectedControllers", &BaseServices::GetAllConnectedControllers, &services_object);
    ros::ServiceServer serviceGetControllerState = n.advertiseService("GetControllerState", &BaseServices::GetControllerState, &services_object);
    ros::ServiceServer serviceGetActuatorCount = n.advertiseService("GetActuatorCount", &BaseServices::GetActuatorCount, &services_object);
    ros::ServiceServer serviceStartWifiScan = n.advertiseService("StartWifiScan", &BaseServices::StartWifiScan, &services_object);
    ros::ServiceServer serviceGetConfiguredWifi = n.advertiseService("GetConfiguredWifi", &BaseServices::GetConfiguredWifi, &services_object);
    ros::ServiceServer serviceOnNotificationNetworkTopic = n.advertiseService("OnNotificationNetworkTopic", &BaseServices::OnNotificationNetworkTopic, &services_object);
    ros::ServiceServer serviceGetArmState = n.advertiseService("GetArmState", &BaseServices::GetArmState, &services_object);
    ros::ServiceServer serviceOnNotificationArmStateTopic = n.advertiseService("OnNotificationArmStateTopic", &BaseServices::OnNotificationArmStateTopic, &services_object);
    ros::ServiceServer serviceGetIPv4Information = n.advertiseService("GetIPv4Information", &BaseServices::GetIPv4Information, &services_object);
    ros::ServiceServer serviceSetCountryCode = n.advertiseService("SetCountryCode", &BaseServices::SetCountryCode, &services_object);
    ros::ServiceServer serviceGetCountryCode = n.advertiseService("GetCountryCode", &BaseServices::GetCountryCode, &services_object);
    

    ROS_INFO("Node's services initialized correctly.");

    ros::Publisher pub_base_feedback = n.advertise<kortex_driver::Feedback>("base_feedback", 1000);
    ros::Publisher pub_joint_state = n.advertise<sensor_msgs::JointState>("base_feedback/joint_state", 1000);

    kortex_driver::Feedback base_feedback;
    kortex_driver::RefreshFeedback::Request req;
    kortex_driver::RefreshFeedback::Response res;

    sensor_msgs::JointState joint_state;

    ros::Rate rate(cyclic_data_rate);
    while (!ros::isShuttingDown())
    {
        services_object.RefreshFeedback(req, res);

        base_feedback.frame_id = res.output.frame_id;

        base_feedback.base.arm_voltage = res.output.base.arm_voltage;
        base_feedback.base.arm_current = res.output.base.arm_current;
        base_feedback.base.temperature_cpu = res.output.base.temperature_cpu;
        base_feedback.base.temperature_ambient = res.output.base.temperature_ambient;
        base_feedback.base.imu_acceleration_x = res.output.base.imu_acceleration_x;
        base_feedback.base.imu_acceleration_y = res.output.base.imu_acceleration_y;
        base_feedback.base.imu_acceleration_z = res.output.base.imu_acceleration_z;
        base_feedback.base.imu_angular_velocity_x = res.output.base.imu_angular_velocity_x;
        base_feedback.base.imu_angular_velocity_y = res.output.base.imu_angular_velocity_y;
        base_feedback.base.imu_angular_velocity_z = res.output.base.imu_angular_velocity_z;
        base_feedback.base.tool_pose_x = res.output.base.tool_pose_x;
        base_feedback.base.tool_pose_y = res.output.base.tool_pose_y;
        base_feedback.base.tool_pose_z = res.output.base.tool_pose_z;
        base_feedback.base.tool_pose_theta_x = res.output.base.tool_pose_theta_x;
        base_feedback.base.tool_pose_theta_y = res.output.base.tool_pose_theta_y;
        base_feedback.base.tool_pose_theta_z = res.output.base.tool_pose_theta_z;
        base_feedback.base.tool_external_wrench_force_x = res.output.base.tool_external_wrench_force_x;
        base_feedback.base.tool_external_wrench_force_y = res.output.base.tool_external_wrench_force_y;
        base_feedback.base.tool_external_wrench_force_z = res.output.base.tool_external_wrench_force_z;
        base_feedback.base.tool_external_wrench_torque_x = res.output.base.tool_external_wrench_torque_x;
        base_feedback.base.tool_external_wrench_torque_y = res.output.base.tool_external_wrench_torque_y;
        base_feedback.base.tool_external_wrench_torque_z = res.output.base.tool_external_wrench_torque_z;
        base_feedback.base.fault_bank_a = res.output.base.fault_bank_a;
        base_feedback.base.fault_bank_b = res.output.base.fault_bank_b;
        base_feedback.base.warning_bank_a = res.output.base.warning_bank_a;
        base_feedback.base.warning_bank_b = res.output.base.warning_bank_b;

        base_feedback.actuators.clear();

        joint_state.position.resize(JOINT_COUNT);
        joint_state.velocity.resize(JOINT_COUNT);
        joint_state.effort.resize(JOINT_COUNT);
        joint_state.name.resize(JOINT_COUNT);

        for(int i = 0; i < JOINT_COUNT; i++)
        {
            kortex_driver::ActuatorFeedback temp;
            
            temp.status_flags = res.output.actuators[i].status_flags;
            temp.jitter_comm = res.output.actuators[i].jitter_comm;
            temp.position = res.output.actuators[i].position;
            temp.velocity = res.output.actuators[i].velocity;
            temp.torque = res.output.actuators[i].torque;
            temp.current_motor = res.output.actuators[i].current_motor;
            temp.voltage = res.output.actuators[i].voltage;
            temp.temperature_motor = res.output.actuators[i].temperature_motor;
            temp.temperature_core = res.output.actuators[i].temperature_core;
            temp.fault_bank_a = res.output.actuators[i].fault_bank_a;
            temp.fault_bank_b = res.output.actuators[i].fault_bank_b;
            temp.warning_bank_a = res.output.actuators[i].warning_bank_a;
            temp.warning_bank_b = res.output.actuators[i].warning_bank_b;

            base_feedback.actuators.push_back(temp);

            joint_state.name[i] = "Actuator" + std::to_string(i + 1);
            joint_state.position[i] =  TO_RAD(res.output.actuators[i].position);
            joint_state.velocity[i] = TO_RAD(res.output.actuators[i].velocity);
            joint_state.effort[i] = res.output.actuators[i].torque;
        }
        base_feedback.interconnect.position = res.output.interconnect.position;



        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = std::to_string(res.output.frame_id);

        pub_base_feedback.publish(base_feedback);
        pub_joint_state.publish(joint_state);

        
        ros::spinOnce();
        
        

        rate.sleep();
    }

    return 1;
}