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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DeviceManager");
    
    ros::NodeHandle n;
    bool valid_ip = false;

    if(argc > 1)
    {
        ROS_INFO("Connecting to IP = %s", argv[1]);
    }
    else
    {
        ROS_INFO("You need to provide an IP adresse as a parameter. ex: rosrun package node 192.168.1.1");
        ros::shutdown();
        return 0;
    }
    
    KortexDeviceManager services_object(argv[1], n);

    ros::ServiceServer serviceGetRunMode = n.advertiseService("GetRunMode", &KortexDeviceManager::GetRunMode, &services_object);
    ros::ServiceServer serviceSetRunMode = n.advertiseService("SetRunMode", &KortexDeviceManager::SetRunMode, &services_object);
    ros::ServiceServer serviceGetDeviceType = n.advertiseService("GetDeviceType", &KortexDeviceManager::GetDeviceType, &services_object);
    ros::ServiceServer serviceGetFirmwareVersion = n.advertiseService("GetFirmwareVersion", &KortexDeviceManager::GetFirmwareVersion, &services_object);
    ros::ServiceServer serviceGetBootloaderVersion = n.advertiseService("GetBootloaderVersion", &KortexDeviceManager::GetBootloaderVersion, &services_object);
    ros::ServiceServer serviceGetModelNumber = n.advertiseService("GetModelNumber", &KortexDeviceManager::GetModelNumber, &services_object);
    ros::ServiceServer serviceGetPartNumber = n.advertiseService("GetPartNumber", &KortexDeviceManager::GetPartNumber, &services_object);
    ros::ServiceServer serviceGetSerialNumber = n.advertiseService("GetSerialNumber", &KortexDeviceManager::GetSerialNumber, &services_object);
    ros::ServiceServer serviceGetMACAddress = n.advertiseService("GetMACAddress", &KortexDeviceManager::GetMACAddress, &services_object);
    ros::ServiceServer serviceGetIPv4Settings = n.advertiseService("GetIPv4Settings", &KortexDeviceManager::GetIPv4Settings, &services_object);
    ros::ServiceServer serviceSetIPv4Settings = n.advertiseService("SetIPv4Settings", &KortexDeviceManager::SetIPv4Settings, &services_object);
    ros::ServiceServer serviceGetPartNumberRevision = n.advertiseService("GetPartNumberRevision", &KortexDeviceManager::GetPartNumberRevision, &services_object);
    ros::ServiceServer serviceGetPowerOnSelfTestResult = n.advertiseService("GetPowerOnSelfTestResult", &KortexDeviceManager::GetPowerOnSelfTestResult, &services_object);
    ros::ServiceServer serviceRebootRequest = n.advertiseService("RebootRequest", &KortexDeviceManager::RebootRequest, &services_object);
    ros::ServiceServer serviceSetSafetyEnable = n.advertiseService("SetSafetyEnable", &KortexDeviceManager::SetSafetyEnable, &services_object);
    ros::ServiceServer serviceSetSafetyErrorThreshold = n.advertiseService("SetSafetyErrorThreshold", &KortexDeviceManager::SetSafetyErrorThreshold, &services_object);
    ros::ServiceServer serviceSetSafetyWarningThreshold = n.advertiseService("SetSafetyWarningThreshold", &KortexDeviceManager::SetSafetyWarningThreshold, &services_object);
    ros::ServiceServer serviceSetSafetyConfiguration = n.advertiseService("SetSafetyConfiguration", &KortexDeviceManager::SetSafetyConfiguration, &services_object);
    ros::ServiceServer serviceGetSafetyConfiguration = n.advertiseService("GetSafetyConfiguration", &KortexDeviceManager::GetSafetyConfiguration, &services_object);
    ros::ServiceServer serviceGetSafetyInformation = n.advertiseService("GetSafetyInformation", &KortexDeviceManager::GetSafetyInformation, &services_object);
    ros::ServiceServer serviceGetSafetyEnable = n.advertiseService("GetSafetyEnable", &KortexDeviceManager::GetSafetyEnable, &services_object);
    ros::ServiceServer serviceGetSafetyStatus = n.advertiseService("GetSafetyStatus", &KortexDeviceManager::GetSafetyStatus, &services_object);
    ros::ServiceServer serviceClearAllSafetyStatus = n.advertiseService("ClearAllSafetyStatus", &KortexDeviceManager::ClearAllSafetyStatus, &services_object);
    ros::ServiceServer serviceClearSafetyStatus = n.advertiseService("ClearSafetyStatus", &KortexDeviceManager::ClearSafetyStatus, &services_object);
    ros::ServiceServer serviceGetAllSafetyConfiguration = n.advertiseService("GetAllSafetyConfiguration", &KortexDeviceManager::GetAllSafetyConfiguration, &services_object);
    ros::ServiceServer serviceGetAllSafetyInformation = n.advertiseService("GetAllSafetyInformation", &KortexDeviceManager::GetAllSafetyInformation, &services_object);
    ros::ServiceServer serviceResetSafetyDefaults = n.advertiseService("ResetSafetyDefaults", &KortexDeviceManager::ResetSafetyDefaults, &services_object);
    ros::ServiceServer serviceSetModelNumber = n.advertiseService("SetModelNumber", &KortexDeviceManager::SetModelNumber, &services_object);
    ros::ServiceServer serviceSetPartNumber = n.advertiseService("SetPartNumber", &KortexDeviceManager::SetPartNumber, &services_object);
    ros::ServiceServer serviceSetPartNumberRevision = n.advertiseService("SetPartNumberRevision", &KortexDeviceManager::SetPartNumberRevision, &services_object);
    ros::ServiceServer serviceSetSerialNumber = n.advertiseService("SetSerialNumber", &KortexDeviceManager::SetSerialNumber, &services_object);
    ros::ServiceServer serviceSetMACAddress = n.advertiseService("SetMACAddress", &KortexDeviceManager::SetMACAddress, &services_object);
    ros::ServiceServer serviceReadAllDevices = n.advertiseService("ReadAllDevices", &KortexDeviceManager::ReadAllDevices, &services_object);
    

    ROS_INFO("Node's services initialized correctly.");

    ros::spin();
      
    return 1;
}