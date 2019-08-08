#ifndef _KORTEX_ARM_DRIVER_H_
#define _KORTEX_ARM_DRIVER_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <thread>

#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"
#include "RouterClient.h"

#include "ActuatorConfigClientRpc.h"
#include "BaseClientRpc.h"
#include "DeviceConfigClientRpc.h"
#include "DeviceManagerClientRpc.h"
#include "InterconnectConfigClientRpc.h"
#include "VisionConfigClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "SessionManager.h"

#include "kortex_driver/non-generated/kortex_math_util.h"

#include "kortex_driver/BaseCyclic_Feedback.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"

#include "kortex_driver/generated/actuatorconfig_services.h"
#include "kortex_driver/generated/base_services.h"
#include "kortex_driver/generated/deviceconfig_services.h"
#include "kortex_driver/generated/devicemanager_services.h"
#include "kortex_driver/generated/interconnectconfig_services.h"
#include "kortex_driver/generated/visionconfig_services.h"
#include "kortex_driver/generated/controlconfig_services.h"

#include "kortex_driver/non-generated/pre_computed_joint_trajectory_action_server.h"
#include "kortex_driver/non-generated/robotiq_gripper_command_action_server.h"

#define TCP_PORT 10000
#define UDP_PORT 10001

#define MAX_CONSECUTIVE_TIMEOUTS_BEFORE_SHUTDOWN 3

#define GREEN_COLOR_CONSOLE "\033[92m"
#define RESET_COLOR_CONSOLE "\033[0m"

class KortexArmDriver
{
  public:
    KortexArmDriver() = delete;
    KortexArmDriver(ros::NodeHandle nh);
    ~KortexArmDriver();

    void parseRosArguments();
    void initApi();
    void verifyProductConfiguration();
    void initRosServices();
    void startActionServers();

  private:

    ros::NodeHandle m_node_handle;

    // Api options
    std::string m_ip_address;
    int m_cyclic_data_publish_rate;
    int m_api_rpc_timeout_ms;
    int m_api_session_inactivity_timeout_ms;
    int m_api_connection_inactivity_timeout_ms;

    // Product configuration and ROS params related to the hardware
    std::string m_arm_name;
    std::vector<std::string> m_arm_joint_names;

    std::string m_gripper_name;
    std::vector<std::string> m_gripper_joint_names;
    std::vector<float> m_gripper_joint_limits;

    int m_interconnect_device_id;
    int m_vision_device_id;

    // Kortex Api objects
    Kinova::Api::TransportClientTcp*  m_tcp_transport;
    Kinova::Api::TransportClientUdp*  m_udp_transport;

    Kinova::Api::RouterClient* m_tcp_router;
    Kinova::Api::RouterClient* m_udp_router;

    Kinova::Api::ActuatorConfig::ActuatorConfigClient*          m_actuator_config;
    Kinova::Api::Base::BaseClient*                              m_base;
    Kinova::Api::ControlConfig::ControlConfigClient*            m_control_config;
    Kinova::Api::DeviceConfig::DeviceConfigClient*              m_device_config;
    Kinova::Api::DeviceManager::DeviceManagerClient*            m_device_manager;
    Kinova::Api::InterconnectConfig::InterconnectConfigClient*  m_interconnect_config;
    Kinova::Api::VisionConfig::VisionConfigClient*              m_vision_config;
    Kinova::Api::BaseCyclic::BaseCyclicClient*                  m_base_cyclic;
    Kinova::Api::SessionManager*                                m_tcp_session_manager;
    Kinova::Api::SessionManager*                                m_udp_session_manager;

    // ROS ServiceProxy's
    ActuatorConfigServices*     m_actuator_config_ros_services;
    BaseServices*               m_base_ros_services;
    ControlConfigServices*      m_control_config_ros_services;
    DeviceConfigServices*       m_device_config_ros_services;
    DeviceManagerServices*      m_device_manager_ros_services;
    InterconnectConfigServices* m_interconnect_config_ros_services;
    VisionConfigServices*       m_vision_config_ros_services;

    // Action servers
    PreComputedJointTrajectoryActionServer* m_action_server_follow_joint_trajectory;
    RobotiqGripperCommandActionServer* m_action_server_gripper_command;

    // ROS and thread objects to publish the feedback from the robot
    bool m_node_is_running;
    int m_consecutive_base_cyclic_timeouts;
    std::mutex m_is_trajectory_running_lock;
    std::thread m_publish_feedback_thread;
    ros::Publisher m_pub_base_feedback;
    ros::Publisher m_pub_joint_state;
    KortexMathUtil m_math_util;
    
    // Private methods
    bool isGripperPresent();
    void publishFeedback();
};

#endif
