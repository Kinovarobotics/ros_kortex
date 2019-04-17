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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Actuator_Services");
    
    uint32_t cyclic_data_rate = 100;
    uint32_t device_id = 0;

    ros::NodeHandle n;
    bool valid_ip = false;

    if(argc > 3)
    {
        stringstream tempId;
        tempId << argv[3];
        tempId >> device_id;

        if(tempId.fail() || tempId.bad())
        {
            ROS_INFO("ERROR - Bad device ID, shutting down the node...");
            ros::shutdown();
            return 0;
        }
    }
    else if(argc > 2)
    {
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
        ROS_INFO("Connecting to IP = %s - node refresh rate = %s, device ID = %d", argv[1], argv[2], device_id);
    }
    else
    {
        ROS_INFO("You need to provide an IP adresse as the first parameter and optionnaly a second parameter to specify the cyclic \
                  rate and a third parameter(optionnal) to specify a device ID. ex: rosrun package node 127.0.0.1 100 4");
        ros::shutdown();
        return 0;
    }
    
    Actuator_Services services_object(argv[1], n, device_id);

    ros::ServiceServer serviceSetDeviceID = n.advertiseService("SetDeviceID", &Actuator_Services::SetDeviceID, &services_object);
    ros::ServiceServer serviceSetApiOptions = n.advertiseService("SetApiOptions", &Actuator_Services::SetApiOptions, &services_object);
    ros::ServiceServer serviceSetCyclicStatus = n.advertiseService("SetCyclicStatus", &Actuator_Services::SetCyclicStatus, &services_object);
    ros::ServiceServer serviceGetCyclicStatus = n.advertiseService("GetCyclicStatus", &Actuator_Services::GetCyclicStatus, &services_object);

    ros::ServiceServer serviceGetAxisOffsets = n.advertiseService("GetAxisOffsets", &Actuator_Services::GetAxisOffsets, &services_object);
    ros::ServiceServer serviceSetAxisOffsets = n.advertiseService("SetAxisOffsets", &Actuator_Services::SetAxisOffsets, &services_object);
    ros::ServiceServer serviceReadTorqueCalibration = n.advertiseService("ReadTorqueCalibration", &Actuator_Services::ReadTorqueCalibration, &services_object);
    ros::ServiceServer serviceWriteTorqueCalibration = n.advertiseService("WriteTorqueCalibration", &Actuator_Services::WriteTorqueCalibration, &services_object);
    ros::ServiceServer serviceSetTorqueOffset = n.advertiseService("SetTorqueOffset", &Actuator_Services::SetTorqueOffset, &services_object);
    ros::ServiceServer serviceGetControlMode = n.advertiseService("GetControlMode", &Actuator_Services::GetControlMode, &services_object);
    ros::ServiceServer serviceSetControlMode = n.advertiseService("SetControlMode", &Actuator_Services::SetControlMode, &services_object);
    ros::ServiceServer serviceGetActivatedControlLoop = n.advertiseService("GetActivatedControlLoop", &Actuator_Services::GetActivatedControlLoop, &services_object);
    ros::ServiceServer serviceSetActivatedControlLoop = n.advertiseService("SetActivatedControlLoop", &Actuator_Services::SetActivatedControlLoop, &services_object);
    ros::ServiceServer serviceGetVectorDriveParameters = n.advertiseService("GetVectorDriveParameters", &Actuator_Services::GetVectorDriveParameters, &services_object);
    ros::ServiceServer serviceSetVectorDriveParameters = n.advertiseService("SetVectorDriveParameters", &Actuator_Services::SetVectorDriveParameters, &services_object);
    ros::ServiceServer serviceGetEncoderDerivativeParameters = n.advertiseService("GetEncoderDerivativeParameters", &Actuator_Services::GetEncoderDerivativeParameters, &services_object);
    ros::ServiceServer serviceSetEncoderDerivativeParameters = n.advertiseService("SetEncoderDerivativeParameters", &Actuator_Services::SetEncoderDerivativeParameters, &services_object);
    ros::ServiceServer serviceGetControlLoopParameters = n.advertiseService("GetControlLoopParameters", &Actuator_Services::GetControlLoopParameters, &services_object);
    ros::ServiceServer serviceSetControlLoopParameters = n.advertiseService("SetControlLoopParameters", &Actuator_Services::SetControlLoopParameters, &services_object);
    ros::ServiceServer serviceStartFrequencyResponse = n.advertiseService("StartFrequencyResponse", &Actuator_Services::StartFrequencyResponse, &services_object);
    ros::ServiceServer serviceStopFrequencyResponse = n.advertiseService("StopFrequencyResponse", &Actuator_Services::StopFrequencyResponse, &services_object);
    ros::ServiceServer serviceStartStepResponse = n.advertiseService("StartStepResponse", &Actuator_Services::StartStepResponse, &services_object);
    ros::ServiceServer serviceStopStepResponse = n.advertiseService("StopStepResponse", &Actuator_Services::StopStepResponse, &services_object);
    ros::ServiceServer serviceStartRampResponse = n.advertiseService("StartRampResponse", &Actuator_Services::StartRampResponse, &services_object);
    ros::ServiceServer serviceStopRampResponse = n.advertiseService("StopRampResponse", &Actuator_Services::StopRampResponse, &services_object);
    ros::ServiceServer serviceSelectCustomData = n.advertiseService("SelectCustomData", &Actuator_Services::SelectCustomData, &services_object);
    ros::ServiceServer serviceGetSelectedCustomData = n.advertiseService("GetSelectedCustomData", &Actuator_Services::GetSelectedCustomData, &services_object);
    ros::ServiceServer serviceSetCommandMode = n.advertiseService("SetCommandMode", &Actuator_Services::SetCommandMode, &services_object);
    ros::ServiceServer serviceClearFaults = n.advertiseService("ClearFaults", &Actuator_Services::ClearFaults, &services_object);
    ros::ServiceServer serviceSetServoing = n.advertiseService("SetServoing", &Actuator_Services::SetServoing, &services_object);
    ros::ServiceServer serviceMoveToPosition = n.advertiseService("MoveToPosition", &Actuator_Services::MoveToPosition, &services_object);
    ros::ServiceServer serviceGetCommandMode = n.advertiseService("GetCommandMode", &Actuator_Services::GetCommandMode, &services_object);
    ros::ServiceServer serviceGetServoing = n.advertiseService("GetServoing", &Actuator_Services::GetServoing, &services_object);
    ros::ServiceServer serviceGetTorqueOffset = n.advertiseService("GetTorqueOffset", &Actuator_Services::GetTorqueOffset, &services_object);
    ros::ServiceServer serviceRefresh = n.advertiseService("Refresh", &Actuator_Services::Refresh, &services_object);
    ros::ServiceServer serviceRefreshCommand = n.advertiseService("RefreshCommand", &Actuator_Services::RefreshCommand, &services_object);
    ros::ServiceServer serviceRefreshFeedback = n.advertiseService("RefreshFeedback", &Actuator_Services::RefreshFeedback, &services_object);
    ros::ServiceServer serviceRefreshCustomData = n.advertiseService("RefreshCustomData", &Actuator_Services::RefreshCustomData, &services_object);
    

    ROS_INFO("Node's services initialized correctly.");

    ros::Publisher pub_feedback = n.advertise<kortex_actuator_driver::Feedback>("actuator_feedback", 1000);
    ros::Publisher pub_joint_state = n.advertise<sensor_msgs::JointState>("actuator_feedback/joint_state", 1000);

    kortex_actuator_driver::Feedback feedback;
    kortex_actuator_driver::RefreshFeedback::Request req;
    kortex_actuator_driver::RefreshFeedback::Response res;

    sensor_msgs::JointState joint_state;

    joint_state.position.resize(1);
    joint_state.velocity.resize(1);
    joint_state.effort.resize(1);
    joint_state.name.resize(1);

    ros::Rate rate(cyclic_data_rate);
    while (!ros::isShuttingDown())
    {
        try
        {
            if(services_object.IsCyclicActive())
            {
                services_object.RefreshFeedback(req, res);

                feedback.feedback_id = res.output.feedback_id;

                feedback.status_flags = res.output.status_flags;
                feedback.jitter_comm = res.output.jitter_comm;
                feedback.position = res.output.position;
                feedback.velocity = res.output.velocity;
                feedback.torque = res.output.torque;
                feedback.current_motor = res.output.current_motor;
                feedback.voltage = res.output.voltage;
                feedback.temperature_motor = res.output.temperature_motor;
                feedback.temperature_core = res.output.temperature_core;
                feedback.fault_bank_a = res.output.fault_bank_a;
                feedback.fault_bank_b = res.output.fault_bank_b;
                feedback.warning_bank_a = res.output.warning_bank_a;
                feedback.warning_bank_b = res.output.warning_bank_b;
                
                joint_state.header.stamp = ros::Time::now();
                joint_state.header.frame_id = std::to_string(res.output.feedback_id.identifier);

                joint_state.name[0] = "Actuator";
                joint_state.position[0] =  TO_RAD(res.output.position);
                joint_state.velocity[0] = TO_RAD(res.output.velocity);
                joint_state.effort[0] = res.output.torque;

                pub_feedback.publish(feedback);
                pub_joint_state.publish(joint_state);    
            }
        }
            
        catch (KDetailedException& ex)
        {
            ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
        }
        catch (std::runtime_error& ex2)
        {
            ROS_INFO("RUN TIME ERROR: %s\n", ex2.what());
        }

        ros::spinOnce();
        
        rate.sleep();
    }

    return 1;
}