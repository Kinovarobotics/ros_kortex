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

#include "kortex_driver/non-generated/kortex_arm_driver.h"

KortexArmDriver::KortexArmDriver(ros::NodeHandle nh):   m_node_handle(nh), 
                                                        m_node_is_running(true), 
                                                        m_consecutive_base_cyclic_timeouts(0),
                                                        m_is_interconnect_module_present(false),
                                                        m_is_vision_module_present(false),
                                                        m_simulator{}
{
    // Parameter to let the other nodes know this node is up
    ros::param::set("is_initialized", false);

    parseRosArguments();
    
    // If with a real robot, start the robot-specific stuff
    if (m_is_real_robot)
    {
        initApi();
        verifyProductConfiguration();
        initSubscribers();
        startActionServers();
    }    

    // ROS Services are always started
    initRosServices();

    // Enable ROS Service simulation if not with a real robot
    if (!m_is_real_robot)
    {
        m_simulator.reset(new KortexArmSimulation(nh));
        registerSimulationHandlers();
    }

    // Start the thread to publish the feedback and joint states
    m_pub_base_feedback = m_node_handle.advertise<kortex_driver::BaseCyclic_Feedback>("base_feedback", 1000);
    m_pub_joint_state = m_node_handle.advertise<sensor_msgs::JointState>("base_feedback/joint_state", 1000);
    if (m_is_real_robot)
    {
        m_publish_feedback_thread = std::thread(&KortexArmDriver::publishRobotFeedback, this);
        moveArmWithinJointLimits();
    }
    else
    {
        m_publish_feedback_thread = std::thread(&KortexArmDriver::publishSimulationFeedback, this);
    }

    // If we get here and no error was thrown we started the node correctly
    ROS_INFO("%sThe Kortex driver has been initialized correctly!%s", GREEN_COLOR_CONSOLE, RESET_COLOR_CONSOLE);
    ros::param::set("is_initialized", true);
}

KortexArmDriver::~KortexArmDriver()
{
    // Kill the thread that publishes the joint states and feedback
    m_node_is_running = false;
    if (m_publish_feedback_thread.joinable())
    {
        m_publish_feedback_thread.join();
    }

    delete m_actuator_config_ros_services;
    delete m_base_ros_services;
    delete m_control_config_ros_services;
    delete m_device_config_ros_services;
    delete m_device_manager_ros_services;
    if (m_interconnect_config_ros_services)
    {
        delete m_interconnect_config_ros_services;
    }
    if (m_vision_config_ros_services)
    {
        delete m_vision_config_ros_services;
    }

    if (!m_is_real_robot)
    {

        delete m_action_server_follow_joint_trajectory;
        delete m_action_server_follow_cartesian_trajectory;

        if (m_action_server_gripper_command)
        {
            delete m_action_server_gripper_command;
        }

        delete m_topic_subscribers;

        m_tcp_session_manager->CloseSession();
        m_udp_session_manager->CloseSession();
        m_tcp_router->SetActivationStatus(false);
        m_udp_router->SetActivationStatus(false);
        m_tcp_transport->disconnect();
        m_udp_transport->disconnect();
        
        delete m_actuator_config;
        delete m_base;
        delete m_control_config;
        delete m_device_config;
        delete m_device_manager;
        delete m_interconnect_config;
        delete m_vision_config;
        delete m_base_cyclic;
        
        delete m_tcp_session_manager;
        delete m_udp_session_manager;
            
        delete m_tcp_router;
        delete m_udp_router;
        delete m_tcp_transport;
        delete m_udp_transport;
    }
}

void KortexArmDriver::parseRosArguments()
{
    bool sim;
    if (!ros::param::get("~sim", sim))
    {
        std::string error_string = "Simulation was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    m_is_real_robot = !sim;

    // Some parameters are only necessary with a real robot
    if (m_is_real_robot)
    {
        bool use_sim_time = false;
        if (ros::param::get("/use_sim_time", use_sim_time))
        {
            if (use_sim_time)
            {
                std::string error_string = "ROS parameter /use_sim_time is true : you may experience problems and you should set it to false and restart the driver.";
                ROS_WARN("%s", error_string.c_str());
            }
        }

        if (!ros::param::get("~ip_address", m_ip_address))
        {
            std::string error_string = "IP address of the robot was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        if (!ros::param::get("~username", m_username))
        {
            std::string error_string = "Username for the robot session was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        if (!ros::param::get("~password", m_password))
        {
            std::string error_string = "Password for the robot session was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        if (!ros::param::get("~use_hard_limits", m_use_hard_limits))
        {
            std::string error_string = "Usage of hard limits as soft was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        if (!ros::param::get("~api_rpc_timeout_ms", m_api_rpc_timeout_ms))
        {
            std::string error_string = "API RPC timeout duration was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        if (!ros::param::get("~api_session_inactivity_timeout_ms", m_api_session_inactivity_timeout_ms))
        {
            std::string error_string = "API session inactivity timeout duration was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
        
        if (!ros::param::get("~api_connection_inactivity_timeout_ms", m_api_connection_inactivity_timeout_ms))
        {
            std::string error_string = "API connection inactivity timeout duration was not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    

    if (!ros::param::get("~cyclic_data_publish_rate", m_cyclic_data_publish_rate))
    {
        std::string error_string = "Publish rate of the cyclic data was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    if (!ros::param::get("~arm", m_arm_name))
    {
        std::string error_string = "Arm name was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    if (!ros::param::get("~dof", m_degrees_of_freedom))
    {
        std::string error_string = "Number of degrees of freedom was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    if (!ros::param::get("~joint_names", m_arm_joint_names))
    {
        std::string error_string = "Arm joint_names were not specified, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }    

    if (!ros::param::get("~gripper", m_gripper_name))
    {
        std::string error_string = "Gripper name was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    std::string robot_name;
    if (!ros::param::get("~robot_name", robot_name))
    {
        std::string error_string = "Robot name was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    if (!ros::param::get("~prefix", m_prefix))
    {
        std::string error_string = "Prefix name was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    if (isGripperPresent())
    {
        // Get the gripper_joint_names size
        if (!ros::param::get("~gripper_joint_names", m_gripper_joint_names))
        {
            std::string error_string = "Gripper joint names were not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        // Get the gripper_joint_limits size
        if (!ros::param::get("~gripper_joint_limits_min", m_gripper_joint_limits_min))
        {
            std::string error_string = "Gripper joint min limits were not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }

        // Get the gripper_joint_limits size
        if (!ros::param::get("~gripper_joint_limits_max", m_gripper_joint_limits_max))
        {
            std::string error_string = "Gripper joint max limits were not specified in the launch file, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
}

void KortexArmDriver::initApi()
{
    // Create the transport objects and connect to the robot
    // TCP for all the Config services
    // UDP for the Cyclic services (the feedback)
    m_tcp_transport = new Kinova::Api::TransportClientTcp();
    m_udp_transport = new Kinova::Api::TransportClientUdp();
    m_tcp_transport->connect(m_ip_address, TCP_PORT);
    m_udp_transport->connect(m_ip_address, UDP_PORT);

    // Create the routers
	m_tcp_router = new Kinova::Api::RouterClient(m_tcp_transport, [](Kinova::Api::KError err) { ROS_ERROR("Kortex API error was encountered with the TCP router: %s", err.toString().c_str()); });
	m_udp_router = new Kinova::Api::RouterClient(m_udp_transport, [](Kinova::Api::KError err) { ROS_ERROR("Kortex API error was encountered with the UDP router: %s", err.toString().c_str()); });
    
    // Create the Protobuf services we are going to use in the ServiceProxy's
    m_actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(m_tcp_router);
    m_base = new Kinova::Api::Base::BaseClient(m_tcp_router);
    m_control_config = new Kinova::Api::ControlConfig::ControlConfigClient(m_tcp_router);
    m_device_config = new Kinova::Api::DeviceConfig::DeviceConfigClient(m_tcp_router);
    m_device_manager = new Kinova::Api::DeviceManager::DeviceManagerClient(m_tcp_router);
    m_interconnect_config = new Kinova::Api::InterconnectConfig::InterconnectConfigClient(m_tcp_router);
    m_vision_config = new Kinova::Api::VisionConfig::VisionConfigClient(m_tcp_router);
    
    // Create the BaseCyclic Protobuf service for the feedback 
    m_base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(m_udp_router);
    	
    // Create the sessions so we can start using the robot
    auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
	createSessionInfo.set_username(m_username);
	createSessionInfo.set_password(m_password);
	createSessionInfo.set_session_inactivity_timeout(m_api_session_inactivity_timeout_ms);
    createSessionInfo.set_connection_inactivity_timeout(m_api_connection_inactivity_timeout_ms);

	m_tcp_session_manager = new Kinova::Api::SessionManager(m_tcp_router);
	m_udp_session_manager = new Kinova::Api::SessionManager(m_udp_router);

    try 
    {
        m_tcp_session_manager->CreateSession(createSessionInfo);
        ROS_INFO("Session created successfully for TCP services");

        m_udp_session_manager->CreateSession(createSessionInfo);
        ROS_INFO("Session created successfully for UDP services");
    }
    catch(std::runtime_error& ex_runtime)
    {
        std::string error_string = "The node could not connect to the arm. Did you specify the right IP address and is the arm powered on?";
        ROS_ERROR("%s", error_string.c_str());
        throw ex_runtime;
    }
}

void KortexArmDriver::verifyProductConfiguration()
{
    // Retrieve the Product Configuration
    Kinova::Api::ProductConfiguration::CompleteProductConfiguration product_config = m_base->GetProductConfiguration();
    
    // Compare arm model (ModelId)
    if (m_arm_name == "gen3")
    {
        if (product_config.model() != Kinova::Api::ProductConfiguration::ModelId::MODEL_ID_L53)
        {
            std::string error_string = "The arm model specified in the launch file doesn't match the detected arm's model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else if (m_arm_name == "gen3_lite")
    {
        if (product_config.model() != Kinova::Api::ProductConfiguration::ModelId::MODEL_ID_L31)
        {
            std::string error_string = "The arm model specified in the launch file doesn't match the detected arm's model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else
    {
        std::string error_string = "The arm model specified in the launch file is not supported, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    // Compare number of degrees of freedom specified in the launch file with the number of detected actuators
    if (product_config.degree_of_freedom() != m_degrees_of_freedom)
    {
        std::string error_string = "The degrees of freedom specified in the launch file doesn't match the detected number of actuators, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    // Compare gripper type (EndEffectorType)
    if (!isGripperPresent())
    {
        if (product_config.end_effector_type() != Kinova::Api::ProductConfiguration::EndEffectorType::END_EFFECTOR_TYPE_NOT_INSTALLED)
        {
            std::string error_string = "The gripper model specified in the launch file doesn't match the detected arm's gripper model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else if (m_gripper_name == "robotiq_2f_85")
    {
        if (product_config.end_effector_type() != Kinova::Api::ProductConfiguration::EndEffectorType::END_EFFECTOR_TYPE_ROBOTIQ_2F_85)
        {
            std::string error_string = "The gripper model specified in the launch file doesn't match the detected arm's gripper model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else if (m_gripper_name == "robotiq_2f_140")
    {
        if (product_config.end_effector_type() != Kinova::Api::ProductConfiguration::EndEffectorType::END_EFFECTOR_TYPE_ROBOTIQ_2F_140)
        {
            std::string error_string = "The gripper model specified in the launch file doesn't match the detected arm's gripper model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else if (m_gripper_name == "gen3_lite_2f")
    {
        if (product_config.end_effector_type() != Kinova::Api::ProductConfiguration::EndEffectorType::END_EFFECTOR_TYPE_L31_GRIPPER_2F)
        {
            std::string error_string = "The gripper model specified in the launch file doesn't match the detected arm's gripper model, shutting down the node...";
            ROS_ERROR("%s", error_string.c_str());
            throw new std::runtime_error(error_string);
        }
    }
    else 
    {
        std::string error_string = "The gripper model specified in the launch file is not supported, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    // Find all the devices and print the device ID's
    ROS_INFO("-------------------------------------------------");
    ROS_INFO("Scanning all devices in robot... ");

// Loop through the devices to find the device ID's
    auto devices = m_device_manager->ReadAllDevices();
    for (auto device : devices.device_handle())
    {
        if (device.device_type() == Kinova::Api::Common::DeviceTypes::BASE)
        {
            ROS_INFO("Base device was found on device identifier %u", device.device_identifier());
        }
        else if (device.device_type() == Kinova::Api::Common::DeviceTypes::SMALL_ACTUATOR 
                 || device.device_type() == Kinova::Api::Common::DeviceTypes::MEDIUM_ACTUATOR
                 || device.device_type() == Kinova::Api::Common::DeviceTypes::BIG_ACTUATOR
                 || device.device_type() == Kinova::Api::Common::DeviceTypes::XBIG_ACTUATOR)
        {
            ROS_INFO("Actuator device of type %s was found on device identifier %u", Kinova::Api::Common::DeviceTypes_Name(device.device_type()).c_str(), device.device_identifier());
        }
        else if (device.device_type() == Kinova::Api::Common::DeviceTypes::INTERCONNECT)
        {
            m_interconnect_device_id = device.device_identifier();
            m_is_interconnect_module_present = true;
            ROS_INFO("Interconnect device was found on device identifier %u", m_interconnect_device_id);
        }
        else if (device.device_type() == Kinova::Api::Common::DeviceTypes::VISION)
        {
            m_vision_device_id = device.device_identifier();
            m_is_vision_module_present = true;
            ROS_INFO("Vision device was found on device identifier %u", m_vision_device_id);
        }
    }
    ROS_INFO("-------------------------------------------------");
	
	// Set the ROS Param for the degrees of freedom
    m_node_handle.setParam("degrees_of_freedom", int(product_config.degree_of_freedom()));
    m_node_handle.setParam("is_gripper_present", isGripperPresent());
    m_node_handle.setParam("gripper_joint_names", m_gripper_joint_names);
	m_node_handle.setParam("has_vision_module", m_is_vision_module_present);
    m_node_handle.setParam("has_interconnect_module", m_is_interconnect_module_present);
}

void KortexArmDriver::initSubscribers()
{
    m_topic_subscribers = new KortexSubscribers(m_node_handle, m_base);
}

void KortexArmDriver::initRosServices()
{
    ROS_INFO("-------------------------------------------------");
    ROS_INFO("Initializing Kortex Driver's services...");
    if (m_is_real_robot)
    {
        m_actuator_config_ros_services = new ActuatorConfigRobotServices(m_node_handle, m_actuator_config, 0, m_api_rpc_timeout_ms);
        m_base_ros_services = new BaseRobotServices(m_node_handle, m_base, 0, m_api_rpc_timeout_ms);
        m_control_config_ros_services = new ControlConfigRobotServices(m_node_handle, m_control_config, 0, m_api_rpc_timeout_ms);
        m_device_config_ros_services = new DeviceConfigRobotServices(m_node_handle, m_device_config, 0, m_api_rpc_timeout_ms);
        m_device_manager_ros_services = new DeviceManagerRobotServices(m_node_handle, m_device_manager, 0, m_api_rpc_timeout_ms);
        if (m_is_interconnect_module_present)
        {
            m_interconnect_config_ros_services = new InterconnectConfigRobotServices(m_node_handle, m_interconnect_config, m_interconnect_device_id, m_api_rpc_timeout_ms);
        }
        else 
        {
            m_interconnect_config_ros_services = nullptr;
        }
        if (m_is_vision_module_present)
        {
            m_vision_config_ros_services = new VisionConfigRobotServices(m_node_handle, m_vision_config, m_vision_device_id, m_api_rpc_timeout_ms);
        }
        else
        {
            m_vision_config_ros_services = nullptr;
        }
    }
    else
    {
        m_actuator_config_ros_services = new ActuatorConfigSimulationServices(m_node_handle);
        m_base_ros_services = new BaseSimulationServices(m_node_handle);
        m_control_config_ros_services = new ControlConfigSimulationServices(m_node_handle);
        m_device_config_ros_services = new DeviceConfigSimulationServices(m_node_handle);
        m_device_manager_ros_services = new DeviceManagerSimulationServices(m_node_handle);
        if (m_is_interconnect_module_present)
        {
            m_interconnect_config_ros_services = new InterconnectConfigSimulationServices(m_node_handle);
        }
        else 
        {
            m_interconnect_config_ros_services = nullptr;
        }
        if (m_is_vision_module_present)
        {
            m_vision_config_ros_services = new VisionConfigSimulationServices(m_node_handle);
        }
        else
        {
            m_vision_config_ros_services = nullptr;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ROS_INFO("Kortex Driver's services initialized correctly.");
    ROS_INFO("-------------------------------------------------");
}

void KortexArmDriver::startActionServers()
{
    // Start Action servers
    m_action_server_follow_joint_trajectory = new JointTrajectoryActionServer(m_prefix + m_arm_name + "_joint_trajectory_controller/follow_joint_trajectory", m_node_handle, m_base, m_base_cyclic, m_control_config, m_use_hard_limits);
    // Start Gripper Action Server if the arm has a gripper
    
    m_action_server_follow_cartesian_trajectory = new CartesianTrajectoryActionServer("cartesian_trajectory_controller/follow_cartesian_trajectory", m_node_handle, m_base, m_base_cyclic);
    
    m_action_server_gripper_command = nullptr;
    if (isGripperPresent())
    {
        m_action_server_gripper_command = new RobotiqGripperCommandActionServer(m_prefix + m_gripper_name + "_gripper_controller/gripper_cmd", m_gripper_joint_names[0], m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0], m_node_handle, m_base, m_base_cyclic);
    }
}

void KortexArmDriver::moveArmWithinJointLimits()
{
    auto joint_angles = m_base->GetMeasuredJointAngles();

    std::map<int, float> limited_joints;
    if (m_degrees_of_freedom == 6)
    {
        // We add angle limitations for joints 1,2 and 4 on 6 dof (values from User guide)
        limited_joints[1] = 128.9;
        limited_joints[2] = 147.8;
        limited_joints[4] = 120.3;
    } 
    else if (m_degrees_of_freedom == 7)
    {
        // We add angle limitations for joints 1,3 and 5 on 7 dof (values from User guide)
        limited_joints[1] = 128.9;
        limited_joints[3] = 147.8;
        limited_joints[5] = 120.3;
    }
    else 
    {
        ROS_WARN("Unsupported number of actuators. Not moving the arm within joint limits");
        return;
    }

    Kinova::Api::Base::JointSpeeds joint_speeds = Kinova::Api::Base::JointSpeeds();
    Kinova::Api::Base::JointSpeed* joint_speed = joint_speeds.add_joint_speeds();

    static const int TIME_COMPENSATION = 100;
    static const int DEFAULT_JOINT_SPEED = 10;
    static const int TIME_SPEED_RATIO = 1000 / DEFAULT_JOINT_SPEED;
    static const float EPSILON = 0.001;

    float angle;
    for (unsigned int i = 0; i < m_degrees_of_freedom; i++)
    {
        angle = joint_angles.joint_angles(i).value();

        // Angles received by GetMeasuredJointAngles are in range [0,360], but waypoints are in range [-180, 180]
        angle = m_math_util.toDeg(m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(angle)));

        if (limited_joints.count(i))
        {
            float delta = m_math_util.findDistanceToBoundary(angle, limited_joints.at(i));

            if (delta > EPSILON)
            {
                // we add some time to compensate acceleration
                int time_ms = delta * TIME_SPEED_RATIO + TIME_COMPENSATION;
                int speed = DEFAULT_JOINT_SPEED;
                joint_speed->set_joint_identifier(i);

                if (angle > 0)
                {
                    speed *= -1;
                }

                joint_speed->set_value(speed);
                m_base->SendJointSpeedsCommand(joint_speeds);

                std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
                
                joint_speed->set_value(0);
                m_base->SendJointSpeedsCommand(joint_speeds);
            }
        }
    }
}

bool KortexArmDriver::isGripperPresent()
{
    return m_gripper_name != "";
}

void KortexArmDriver::publishRobotFeedback()
{

    Kinova::Api::BaseCyclic::Feedback feedback_from_api;
    sensor_msgs::JointState joint_state;

    ros::Rate rate(m_cyclic_data_publish_rate);
    while (m_node_is_running)
    {
        try 
        {
            feedback_from_api = m_base_cyclic->RefreshFeedback();
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            ROS_WARN("Kortex exception while getting the base_feedback");
		    ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
		    ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
		    ROS_WARN("Error description: %s\n", ex.what());
        }
        catch (std::runtime_error& ex_runtime)
        {
            ROS_DEBUG("Runtime exception detected while getting feedback! %d", ++m_consecutive_base_cyclic_timeouts);
            ROS_DEBUG("%s", ex_runtime.what());
        }
        catch (std::future_error& ex_future)
        {
            ROS_DEBUG("Future exception detected while getting feedback : %s", ex_future.what());
        }

        // If the arm has disconnected, the node cannot continue running
        if (m_consecutive_base_cyclic_timeouts > MAX_CONSECUTIVE_TIMEOUTS_BEFORE_SHUTDOWN)
        {
            std::string error_string = "Too many consecutive timeouts : killing the node.";
            ROS_ERROR("%s", error_string.c_str());
            ros::shutdown();
            return;
        }

        m_consecutive_base_cyclic_timeouts = 0;
        kortex_driver::BaseCyclic_Feedback base_feedback;
        ToRosData(feedback_from_api, base_feedback);

        joint_state.name.resize(base_feedback.actuators.size() + base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.size());
        joint_state.position.resize(base_feedback.actuators.size() + base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.size());
        joint_state.velocity.resize(base_feedback.actuators.size() + base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.size());
        joint_state.effort.resize(base_feedback.actuators.size() + base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.size());
        joint_state.header.stamp = ros::Time::now();

        for (int i = 0; i < base_feedback.actuators.size(); i++)
        {
            joint_state.name[i] = m_arm_joint_names[i];
            joint_state.position[i] = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(base_feedback.actuators[i].position));
            joint_state.velocity[i] = m_math_util.toRad(base_feedback.actuators[i].velocity);
            joint_state.effort[i] = base_feedback.actuators[i].torque;
        }

        if (isGripperPresent())
        {
            for (int i = 0; i < base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.size(); i++)
            {
                int joint_state_index = base_feedback.actuators.size() + i;
                joint_state.name[joint_state_index] = m_gripper_joint_names[i];
                // Arm feedback is between 0 and 100, and limits in URDF are specified in gripper_joint_limits_min[i] and gripper_joint_limits_max[i] parameters
                joint_state.position[joint_state_index] = m_math_util.absolute_position_from_relative(base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[i].position / 100.0, m_gripper_joint_limits_min[i], m_gripper_joint_limits_max[i]);
                joint_state.velocity[joint_state_index] = base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[i].velocity;
                // Not supported for now
                joint_state.effort[joint_state_index] = 0.0;
            }
        }

        m_pub_base_feedback.publish(base_feedback);
        m_pub_joint_state.publish(joint_state);

        rate.sleep();
    }
}

void KortexArmDriver::publishSimulationFeedback()
{
    ros::Rate rate(m_cyclic_data_publish_rate);
    while (m_node_is_running)
    {
        m_pub_base_feedback.publish(m_simulator->GetFeedback());
        rate.sleep();
    }
}

void KortexArmDriver::registerSimulationHandlers()
{
    BaseSimulationServices* base_services_simulation = dynamic_cast<BaseSimulationServices*>(m_base_ros_services);
    // Link the m_simulator handlers to the ROS services callbacks
    // Action services
    base_services_simulation->CreateActionHandler = std::bind(&KortexArmSimulation::CreateAction, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->ReadActionHandler = std::bind(&KortexArmSimulation::ReadAction, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->ReadAllActionsHandler = std::bind(&KortexArmSimulation::ReadAllActions, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->DeleteActionHandler = std::bind(&KortexArmSimulation::DeleteAction, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->UpdateActionHandler = std::bind(&KortexArmSimulation::UpdateAction, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->ExecuteActionFromReferenceHandler = std::bind(&KortexArmSimulation::ExecuteActionFromReference, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->ExecuteActionHandler = std::bind(&KortexArmSimulation::ExecuteAction, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->StopActionHandler = std::bind(&KortexArmSimulation::StopAction, m_simulator.get(), std::placeholders::_1);
    // Other services
    base_services_simulation->PlayCartesianTrajectoryHandler = std::bind(&KortexArmSimulation::PlayCartesianTrajectory, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->PlayJointTrajectoryHandler = std::bind(&KortexArmSimulation::PlayJointTrajectory, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->SendJointSpeedsCommandHandler = std::bind(&KortexArmSimulation::SendJointSpeedsCommand, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->SendGripperCommandHandler = std::bind(&KortexArmSimulation::SendGripperCommand, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->StopHandler = std::bind(&KortexArmSimulation::Stop, m_simulator.get(), std::placeholders::_1);
    base_services_simulation->ApplyEmergencyStopHandler = std::bind(&KortexArmSimulation::ApplyEmergencyStop, m_simulator.get(), std::placeholders::_1);

    // Uncomment this SendTwistCommand handler to use the twist command simulation - not stable
    // base_services_simulation->SendTwistCommandHandler = std::bind(&KortexArmSimulation::SendTwistCommand, m_simulator.get(), std::placeholders::_1);

    // Prospects
    //SendSelectedJointSpeedCommand
    //PlaySelectedJointTrajectory
    //PlayCartesianTrajectoryPosition
    //PlayCartesianTrajectoryOrientation
}