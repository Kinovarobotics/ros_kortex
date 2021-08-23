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
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"

#include "kortex_driver/generated/robot/actuatorconfig_services.h"
#include "kortex_driver/generated/robot/base_services.h"
#include "kortex_driver/generated/robot/deviceconfig_services.h"
#include "kortex_driver/generated/robot/devicemanager_services.h"
#include "kortex_driver/generated/robot/interconnectconfig_services.h"
#include "kortex_driver/generated/robot/visionconfig_services.h"
#include "kortex_driver/generated/robot/controlconfig_services.h"

#include "kortex_driver/generated/simulation/actuatorconfig_services.h"
#include "kortex_driver/generated/simulation/base_services.h"
#include "kortex_driver/generated/simulation/deviceconfig_services.h"
#include "kortex_driver/generated/simulation/devicemanager_services.h"
#include "kortex_driver/generated/simulation/interconnectconfig_services.h"
#include "kortex_driver/generated/simulation/visionconfig_services.h"
#include "kortex_driver/generated/simulation/controlconfig_services.h"

#include "kortex_driver/non-generated/joint_trajectory_action_server.h"
#include "kortex_driver/non-generated/cartesian_trajectory_action_server.h"
#include "kortex_driver/non-generated/robotiq_gripper_command_action_server.h"
#include "kortex_driver/non-generated/kortex_subscribers.h"
#include "kortex_driver/non-generated/kortex_arm_simulation.h"

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
    void initSubscribers();
    void initRosServices();
    void startActionServers();
    void moveArmWithinJointLimits();

  private:

    ros::NodeHandle m_node_handle;

    // False if in simulation
    bool m_is_real_robot;
    std::unique_ptr<KortexArmSimulation> m_simulator;

    // Api options
    std::string m_ip_address;
    std::string m_username;
    std::string m_password;
    bool m_use_hard_limits;
    int m_cyclic_data_publish_rate;
    int m_api_rpc_timeout_ms;
    int m_api_session_inactivity_timeout_ms;
    int m_api_connection_inactivity_timeout_ms;

    // Product configuration and ROS params related to the hardware
    std::string m_arm_name;
    std::vector<std::string> m_arm_joint_names;

    std::string m_gripper_name;
    std::string m_prefix;
    std::vector<std::string> m_gripper_joint_names;
    std::vector<float> m_gripper_joint_limits_min;
    std::vector<float> m_gripper_joint_limits_max;

    int m_degrees_of_freedom;

    bool m_is_interconnect_module_present;
    int m_interconnect_device_id;

    bool m_is_vision_module_present;
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
    IActuatorConfigServices*     m_actuator_config_ros_services;
    IBaseServices*               m_base_ros_services;
    IControlConfigServices*      m_control_config_ros_services;
    IDeviceConfigServices*       m_device_config_ros_services;
    IDeviceManagerServices*      m_device_manager_ros_services;
    IInterconnectConfigServices* m_interconnect_config_ros_services;
    IVisionConfigServices*       m_vision_config_ros_services;

    // Action servers
    JointTrajectoryActionServer*       m_action_server_follow_joint_trajectory;
    CartesianTrajectoryActionServer*   m_action_server_follow_cartesian_trajectory;
    RobotiqGripperCommandActionServer* m_action_server_gripper_command;

    // Topic subscribers
    KortexSubscribers* m_topic_subscribers;

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
    void publishRobotFeedback();
    void publishSimulationFeedback();
    void registerSimulationHandlers();
};

#endif
