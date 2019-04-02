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
#include "common_ros_converter.h"
#include "common_proto_converter.h"
#include "actuatorconfig_ros_converter.h"
#include "actuatorconfig_proto_converter.h"
#include "actuatorcyclic_ros_converter.h"
#include "actuatorcyclic_proto_converter.h"
Actuator_Services::Actuator_Services(char* ip, ros::NodeHandle& n, uint32_t device_id) : m_n(n)
{
	m_transport = new TransportClientUdp();
	m_transport->connect(ip, 10000);

	m_router = new RouterClient(m_transport, [](KError err) { cout << "_________ callback error _________" << err.toString(); });
	m_CurrentDeviceID = device_id;
	m_apiOptions.timeout_ms = 3000;

	m_actuatorconfig = new ActuatorConfig::ActuatorConfigClient(m_router);
	m_actuatorcyclic = new ActuatorCyclic::ActuatorCyclicClient(m_router);
	//If the Device ID is different than 0, it means that we are using the feature DEVICE ROUTING.
	if(m_CurrentDeviceID != 0)
	{
		m_SessionManager = new SessionManager(m_router);
		auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
		
		createSessionInfo.set_username("admin");
		createSessionInfo.set_password("admin");
		createSessionInfo.set_session_inactivity_timeout(35000);

		m_SessionManager->CreateSession(createSessionInfo);
	}
	
	m_pub_Error = m_n.advertise<kortex_actuator_driver::KortexError>("KortexError", 1000);std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

bool Actuator_Services::SetDeviceID(kortex_actuator_driver::SetDeviceID::Request  &req, kortex_actuator_driver::SetDeviceID::Response &res)
{
	if(m_CurrentDeviceID == 0)
	{
		auto sessionManager = new SessionManager(m_router);
		auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
		
		createSessionInfo.set_username("admin");
		createSessionInfo.set_password("admin");
		createSessionInfo.set_session_inactivity_timeout(35000);

		sessionManager->CreateSession(createSessionInfo);
	}
	
	m_CurrentDeviceID = req.device_id;
}

bool Actuator_Services::SetApiOptions(kortex_actuator_driver::SetApiOptions::Request  &req, kortex_actuator_driver::SetApiOptions::Response &res)
{
	m_apiOptions.timeout_ms = req.input.timeout_ms;

	return true;
}

bool Actuator_Services::GetCyclicStatus(kortex_actuator_driver::GetCyclicStatus::Request  &req, kortex_actuator_driver::GetCyclicStatus::Response &res)
{
	res.status.isActive = m_cyclicActive;
}

bool Actuator_Services::SetCyclicStatus(kortex_actuator_driver::SetCyclicStatus::Request  &req, kortex_actuator_driver::SetCyclicStatus::Response &res)
{
	m_cyclicActive = req.status.isActive;
}

bool Actuator_Services::IsCyclicActive()
{
	return m_cyclicActive;
}




bool Actuator_Services::GetAxisOffsets(kortex_actuator_driver::GetAxisOffsets::Request  &req, kortex_actuator_driver::GetAxisOffsets::Response &res)
{
	Empty input;
	AxisOffsets output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetAxisOffsets(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetAxisOffsets(kortex_actuator_driver::SetAxisOffsets::Request  &req, kortex_actuator_driver::SetAxisOffsets::Response &res)
{
	AxisPosition input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetAxisOffsets(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::ReadTorqueCalibration(kortex_actuator_driver::ReadTorqueCalibration::Request  &req, kortex_actuator_driver::ReadTorqueCalibration::Response &res)
{
	Empty input;
	TorqueCalibration output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->ReadTorqueCalibration(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::WriteTorqueCalibration(kortex_actuator_driver::WriteTorqueCalibration::Request  &req, kortex_actuator_driver::WriteTorqueCalibration::Response &res)
{
	TorqueCalibration input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->WriteTorqueCalibration(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::SetTorqueOffset(kortex_actuator_driver::SetTorqueOffset::Request  &req, kortex_actuator_driver::SetTorqueOffset::Response &res)
{
	TorqueOffset input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetTorqueOffset(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetControlMode(kortex_actuator_driver::GetControlMode::Request  &req, kortex_actuator_driver::GetControlMode::Response &res)
{
	Empty input;
	ControlModeInformation output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlMode(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetControlMode(kortex_actuator_driver::SetControlMode::Request  &req, kortex_actuator_driver::SetControlMode::Response &res)
{
	ControlModeInformation input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlMode(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetActivatedControlLoop(kortex_actuator_driver::GetActivatedControlLoop::Request  &req, kortex_actuator_driver::GetActivatedControlLoop::Response &res)
{
	Empty input;
	ControlLoop output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetActivatedControlLoop(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetActivatedControlLoop(kortex_actuator_driver::SetActivatedControlLoop::Request  &req, kortex_actuator_driver::SetActivatedControlLoop::Response &res)
{
	ControlLoop input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetActivatedControlLoop(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetVectorDriveParameters(kortex_actuator_driver::GetVectorDriveParameters::Request  &req, kortex_actuator_driver::GetVectorDriveParameters::Response &res)
{
	Empty input;
	VectorDriveParameters output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetVectorDriveParameters(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetVectorDriveParameters(kortex_actuator_driver::SetVectorDriveParameters::Request  &req, kortex_actuator_driver::SetVectorDriveParameters::Response &res)
{
	VectorDriveParameters input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetVectorDriveParameters(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetEncoderDerivativeParameters(kortex_actuator_driver::GetEncoderDerivativeParameters::Request  &req, kortex_actuator_driver::GetEncoderDerivativeParameters::Response &res)
{
	Empty input;
	EncoderDerivativeParameters output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetEncoderDerivativeParameters(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetEncoderDerivativeParameters(kortex_actuator_driver::SetEncoderDerivativeParameters::Request  &req, kortex_actuator_driver::SetEncoderDerivativeParameters::Response &res)
{
	EncoderDerivativeParameters input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetEncoderDerivativeParameters(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetControlLoopParameters(kortex_actuator_driver::GetControlLoopParameters::Request  &req, kortex_actuator_driver::GetControlLoopParameters::Response &res)
{
	LoopSelection input;
	ToProtoData(req.input, &input);
	ControlLoopParameters output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlLoopParameters(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetControlLoopParameters(kortex_actuator_driver::SetControlLoopParameters::Request  &req, kortex_actuator_driver::SetControlLoopParameters::Response &res)
{
	ControlLoopParameters input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlLoopParameters(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StartFrequencyResponse(kortex_actuator_driver::StartFrequencyResponse::Request  &req, kortex_actuator_driver::StartFrequencyResponse::Response &res)
{
	FrequencyResponse input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StartFrequencyResponse(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StopFrequencyResponse(kortex_actuator_driver::StopFrequencyResponse::Request  &req, kortex_actuator_driver::StopFrequencyResponse::Response &res)
{
	Empty input;
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StopFrequencyResponse(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StartStepResponse(kortex_actuator_driver::StartStepResponse::Request  &req, kortex_actuator_driver::StartStepResponse::Response &res)
{
	StepResponse input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StartStepResponse(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StopStepResponse(kortex_actuator_driver::StopStepResponse::Request  &req, kortex_actuator_driver::StopStepResponse::Response &res)
{
	Empty input;
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StopStepResponse(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StartRampResponse(kortex_actuator_driver::StartRampResponse::Request  &req, kortex_actuator_driver::StartRampResponse::Response &res)
{
	RampResponse input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StartRampResponse(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::StopRampResponse(kortex_actuator_driver::StopRampResponse::Request  &req, kortex_actuator_driver::StopRampResponse::Response &res)
{
	Empty input;
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->StopRampResponse(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::SelectCustomData(kortex_actuator_driver::SelectCustomData::Request  &req, kortex_actuator_driver::SelectCustomData::Response &res)
{
	CustomDataSelection input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SelectCustomData(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetSelectedCustomData(kortex_actuator_driver::GetSelectedCustomData::Request  &req, kortex_actuator_driver::GetSelectedCustomData::Response &res)
{
	Empty input;
	CustomDataSelection output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetSelectedCustomData(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::SetCommandMode(kortex_actuator_driver::SetCommandMode::Request  &req, kortex_actuator_driver::SetCommandMode::Response &res)
{
	CommandModeInformation input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetCommandMode(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::ClearFaults(kortex_actuator_driver::ClearFaults::Request  &req, kortex_actuator_driver::ClearFaults::Response &res)
{
	Empty input;
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->ClearFaults(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::SetServoing(kortex_actuator_driver::SetServoing::Request  &req, kortex_actuator_driver::SetServoing::Response &res)
{
	Servoing input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetServoing(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::MoveToPosition(kortex_actuator_driver::MoveToPosition::Request  &req, kortex_actuator_driver::MoveToPosition::Response &res)
{
	PositionCommand input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->MoveToPosition(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::GetCommandMode(kortex_actuator_driver::GetCommandMode::Request  &req, kortex_actuator_driver::GetCommandMode::Response &res)
{
	Empty input;
	CommandModeInformation output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetCommandMode(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::GetServoing(kortex_actuator_driver::GetServoing::Request  &req, kortex_actuator_driver::GetServoing::Response &res)
{
	Empty input;
	Servoing output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetServoing(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::GetTorqueOffset(kortex_actuator_driver::GetTorqueOffset::Request  &req, kortex_actuator_driver::GetTorqueOffset::Response &res)
{
	Empty input;
	TorqueOffset output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetTorqueOffset(m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}


bool Actuator_Services::Refresh(kortex_actuator_driver::Refresh::Request  &req, kortex_actuator_driver::Refresh::Response &res)
{
	Command input;
	ToProtoData(req.input, &input);
	Feedback output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorcyclic->Refresh(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::RefreshCommand(kortex_actuator_driver::RefreshCommand::Request  &req, kortex_actuator_driver::RefreshCommand::Response &res)
{
	Command input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		m_actuatorcyclic->RefreshCommand(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool Actuator_Services::RefreshFeedback(kortex_actuator_driver::RefreshFeedback::Request  &req, kortex_actuator_driver::RefreshFeedback::Response &res)
{
	MessageId input;
	ToProtoData(req.input, &input);
	Feedback output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorcyclic->RefreshFeedback(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool Actuator_Services::RefreshCustomData(kortex_actuator_driver::RefreshCustomData::Request  &req, kortex_actuator_driver::RefreshCustomData::Response &res)
{
	MessageId input;
	ToProtoData(req.input, &input);
	CustomData output;
	kortex_actuator_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorcyclic->RefreshCustomData(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
