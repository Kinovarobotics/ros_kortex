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
#include "basecyclic_ros_converter.h"
#include "basecyclic_proto_converter.h"

#include "base_ros_converter.h"
#include "base_proto_converter.h"
BaseServices::BaseServices(char* ip, ros::NodeHandle& n) : m_n(n)
{
	m_transport = new TransportClientUdp();
	m_transport->connect(ip, 10000);

	m_router = new RouterClient(m_transport, [](KError err) { cout << "_________ callback error _________" << err.toString(); });
	m_CurrentDeviceID = 0;
	m_apiOptions.timeout_ms = 3000;

	m_basecyclic = new BaseCyclic::BaseCyclicClient(m_router);
	m_base = new Base::BaseClient(m_router);
	m_SessionManager = new SessionManager(m_router);
	auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();

	createSessionInfo.set_username("admin");
	createSessionInfo.set_password("admin");
	createSessionInfo.set_session_inactivity_timeout(35000);

	m_SessionManager->CreateSession(createSessionInfo);
	std::cout << "\nSession Created\n";

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("KortexError", 1000);
	m_pub_ConfigurationChangeTopic = m_n.advertise<kortex_driver::ConfigurationChangeNotification>("ConfigurationChangeTopic", 1000);
	m_pub_MappingInfoTopic = m_n.advertise<kortex_driver::MappingInfoNotification>("MappingInfoTopic", 1000);
	m_pub_ControlModeTopic = m_n.advertise<kortex_driver::ControlModeNotification>("ControlModeTopic", 1000);
	m_pub_OperatingModeTopic = m_n.advertise<kortex_driver::OperatingModeNotification>("OperatingModeTopic", 1000);
	m_pub_SequenceInfoTopic = m_n.advertise<kortex_driver::SequenceInfoNotification>("SequenceInfoTopic", 1000);
	m_pub_ProtectionZoneTopic = m_n.advertise<kortex_driver::ProtectionZoneNotification>("ProtectionZoneTopic", 1000);
	m_pub_UserTopic = m_n.advertise<kortex_driver::UserNotification>("UserTopic", 1000);
	m_pub_ControllerTopic = m_n.advertise<kortex_driver::ControllerNotification>("ControllerTopic", 1000);
	m_pub_ActionTopic = m_n.advertise<kortex_driver::ActionNotification>("ActionTopic", 1000);
	m_pub_RobotEventTopic = m_n.advertise<kortex_driver::RobotEventNotification>("RobotEventTopic", 1000);
	m_pub_ServoingModeTopic = m_n.advertise<kortex_driver::ServoingModeNotification>("ServoingModeTopic", 1000);
	m_pub_FactoryTopic = m_n.advertise<kortex_driver::FactoryNotification>("FactoryTopic", 1000);
	m_pub_NetworkTopic = m_n.advertise<kortex_driver::NetworkNotification>("NetworkTopic", 1000);
	m_pub_ArmStateTopic = m_n.advertise<kortex_driver::ArmStateNotification>("ArmStateTopic", 1000);std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

bool BaseServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_CurrentDeviceID = req.device_id;

	return true;
}

bool BaseServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_apiOptions.timeout_ms = req.input.timeout_ms;

	return true;
}




bool BaseServices::Refresh(kortex_driver::Refresh::Request  &req, kortex_driver::Refresh::Response &res)
{
	Command input;
	ToProtoData(req.input, &input);
	Feedback output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_basecyclic->Refresh(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::RefreshCommand(kortex_driver::RefreshCommand::Request  &req, kortex_driver::RefreshCommand::Response &res)
{
	Command input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_basecyclic->RefreshCommand(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::RefreshFeedback(kortex_driver::RefreshFeedback::Request  &req, kortex_driver::RefreshFeedback::Response &res)
{
	Empty input;
	Feedback output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_basecyclic->RefreshFeedback(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::RefreshCustomData(kortex_driver::RefreshCustomData::Request  &req, kortex_driver::RefreshCustomData::Response &res)
{
	CustomData input;
	ToProtoData(req.input, &input);
	CustomData output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_basecyclic->RefreshCustomData(input, m_CurrentDeviceID, m_apiOptions);
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



bool BaseServices::CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res)
{
	FullUserProfile input;
	ToProtoData(req.input, &input);
	UserProfileHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateUserProfile(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res)
{
	UserProfile input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateUserProfile(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res)
{
	UserProfileHandle input;
	ToProtoData(req.input, &input);
	UserProfile output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadUserProfile(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res)
{
	UserProfileHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteUserProfile(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res)
{
	Empty input;
	UserProfileList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUserProfiles(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res)
{
	Empty input;
	UserList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUsers(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res)
{
	PasswordChange input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ChangePassword(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res)
{
	Sequence input;
	ToProtoData(req.input, &input);
	SequenceHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateSequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res)
{
	Sequence input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateSequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res)
{
	SequenceHandle input;
	ToProtoData(req.input, &input);
	Sequence output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadSequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res)
{
	SequenceHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteSequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res)
{
	Empty input;
	SequenceList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllSequences(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res)
{
	SequenceTaskHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteSequenceTask(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res)
{
	SequenceHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteAllSequenceTasks(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res)
{
	SequenceHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlaySequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res)
{
	AdvancedSequenceHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayAdvancedSequence(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StopSequence(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PauseSequence(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ResumeSequence(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res)
{
	ProtectionZone input;
	ToProtoData(req.input, &input);
	ProtectionZoneHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateProtectionZone(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res)
{
	ProtectionZone input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateProtectionZone(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res)
{
	ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
	ProtectionZone output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadProtectionZone(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res)
{
	ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteProtectionZone(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res)
{
	Empty input;
	ProtectionZoneList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllProtectionZones(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res)
{
	Mapping input;
	ToProtoData(req.input, &input);
	MappingHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateMapping(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res)
{
	MappingHandle input;
	ToProtoData(req.input, &input);
	Mapping output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadMapping(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res)
{
	Empty input;
	MappingList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMappings(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res)
{
	Map input;
	ToProtoData(req.input, &input);
	MapHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateMap(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res)
{
	MappingHandle input;
	ToProtoData(req.input, &input);
	MapList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMaps(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res)
{
	ActivateMapHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ActivateMap(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res)
{
	Action input;
	ToProtoData(req.input, &input);
	ActionHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateAction(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res)
{
	ActionHandle input;
	ToProtoData(req.input, &input);
	Action output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAction(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res)
{
	RequestedActionType input;
	ToProtoData(req.input, &input);
	ActionList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllActions(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res)
{
	ActionHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteAction(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res)
{
	Action input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateAction(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res)
{
	ActionHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ExecuteActionFromReference(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res)
{
	Action input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ExecuteAction(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PauseAction(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StopAction(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ResumeAction(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res)
{
	NetworkHandle input;
	ToProtoData(req.input, &input);
	IPv4Configuration output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Configuration(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res)
{
	FullIPv4Configuration input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetIPv4Configuration(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res)
{
	CommunicationInterfaceConfiguration input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetCommunicationInterfaceEnable(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res)
{
	NetworkHandle input;
	ToProtoData(req.input, &input);
	CommunicationInterfaceConfiguration output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->IsCommunicationInterfaceEnable(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res)
{
	Empty input;
	WifiInformationList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAvailableWifi(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res)
{
	Ssid input;
	ToProtoData(req.input, &input);
	WifiInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetWifiInformation(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res)
{
	WifiConfiguration input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->AddWifiConfiguration(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res)
{
	Ssid input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteWifiConfiguration(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res)
{
	Empty input;
	WifiConfigurationList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConfiguredWifis(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res)
{
	Ssid input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ConnectWifi(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DisconnectWifi(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res)
{
	Empty input;
	WifiInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetConnectedWifiInformation(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::Unsubscribe(kortex_driver::Unsubscribe::Request  &req, kortex_driver::Unsubscribe::Response &res)
{
	NotificationHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Unsubscribe(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ConfigurationChangeNotification) > callback = std::bind(&BaseServices::cb_ConfigurationChangeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationConfigurationChangeTopic(callback, input);
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
void BaseServices::cb_ConfigurationChangeTopic(Base::ConfigurationChangeNotification notif)
{
	kortex_driver::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::MappingInfoNotification) > callback = std::bind(&BaseServices::cb_MappingInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationMappingInfoTopic(callback, input);
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
void BaseServices::cb_MappingInfoTopic(Base::MappingInfoNotification notif)
{
	kortex_driver::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationControlModeTopic(kortex_driver::OnNotificationControlModeTopic::Request  &req, kortex_driver::OnNotificationControlModeTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ControlModeNotification) > callback = std::bind(&BaseServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControlModeTopic(callback, input);
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
void BaseServices::cb_ControlModeTopic(Base::ControlModeNotification notif)
{
	kortex_driver::ControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::OperatingModeNotification) > callback = std::bind(&BaseServices::cb_OperatingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationOperatingModeTopic(callback, input);
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
void BaseServices::cb_OperatingModeTopic(Base::OperatingModeNotification notif)
{
	kortex_driver::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::SequenceInfoNotification) > callback = std::bind(&BaseServices::cb_SequenceInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationSequenceInfoTopic(callback, input);
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
void BaseServices::cb_SequenceInfoTopic(Base::SequenceInfoNotification notif)
{
	kortex_driver::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ProtectionZoneNotification) > callback = std::bind(&BaseServices::cb_ProtectionZoneTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationProtectionZoneTopic(callback, input);
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
void BaseServices::cb_ProtectionZoneTopic(Base::ProtectionZoneNotification notif)
{
	kortex_driver::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::UserNotification) > callback = std::bind(&BaseServices::cb_UserTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationUserTopic(callback, input);
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
void BaseServices::cb_UserTopic(Base::UserNotification notif)
{
	kortex_driver::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ControllerNotification) > callback = std::bind(&BaseServices::cb_ControllerTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControllerTopic(callback, input);
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
void BaseServices::cb_ControllerTopic(Base::ControllerNotification notif)
{
	kortex_driver::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ActionNotification) > callback = std::bind(&BaseServices::cb_ActionTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationActionTopic(callback, input);
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
void BaseServices::cb_ActionTopic(Base::ActionNotification notif)
{
	kortex_driver::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::RobotEventNotification) > callback = std::bind(&BaseServices::cb_RobotEventTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationRobotEventTopic(callback, input);
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
void BaseServices::cb_RobotEventTopic(Base::RobotEventNotification notif)
{
	kortex_driver::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic.publish(ros_msg);
}

bool BaseServices::GetFwdKinematics(kortex_driver::GetFwdKinematics::Request  &req, kortex_driver::GetFwdKinematics::Response &res)
{
	Empty input;
	TransformationMatrix output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetFwdKinematics(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res)
{
	ConstrainedPose input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectory(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res)
{
	ConstrainedPosition input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryPosition(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res)
{
	ConstrainedOrientation input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryOrientation(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::Pause(kortex_driver::Pause::Request  &req, kortex_driver::Pause::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Pause(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::Resume(kortex_driver::Resume::Request  &req, kortex_driver::Resume::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Resume(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res)
{
	Empty input;
	Pose output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredCartesianPose(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCommandedCartesianPose(kortex_driver::GetCommandedCartesianPose::Request  &req, kortex_driver::GetCommandedCartesianPose::Response &res)
{
	Empty input;
	Pose output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCommandedCartesianPose(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetTargetedCartesianPose(kortex_driver::GetTargetedCartesianPose::Request  &req, kortex_driver::GetTargetedCartesianPose::Response &res)
{
	Empty input;
	Pose output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetTargetedCartesianPose(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res)
{
	TwistCommand input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendTwistCommand(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetMeasuredTwist(kortex_driver::GetMeasuredTwist::Request  &req, kortex_driver::GetMeasuredTwist::Response &res)
{
	Empty input;
	Twist output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredTwist(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCommandedTwist(kortex_driver::GetCommandedTwist::Request  &req, kortex_driver::GetCommandedTwist::Response &res)
{
	Empty input;
	Twist output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCommandedTwist(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res)
{
	ConstrainedJointAngles input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayJointTrajectory(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res)
{
	ConstrainedJointAngle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlaySelectedJointTrajectory(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res)
{
	Empty input;
	JointAngles output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredJointAngles(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCommandedJointAngles(kortex_driver::GetCommandedJointAngles::Request  &req, kortex_driver::GetCommandedJointAngles::Response &res)
{
	Empty input;
	JointAngles output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCommandedJointAngles(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SendJointSpeedsCommmand(kortex_driver::SendJointSpeedsCommmand::Request  &req, kortex_driver::SendJointSpeedsCommmand::Response &res)
{
	JointSpeeds input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendJointSpeedsCommmand(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res)
{
	JointSpeed input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendSelectedJointSpeedCommand(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetMeasuredJointSpeeds(kortex_driver::GetMeasuredJointSpeeds::Request  &req, kortex_driver::GetMeasuredJointSpeeds::Response &res)
{
	Empty input;
	JointSpeeds output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredJointSpeeds(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCommandedJointSpeeds(kortex_driver::GetCommandedJointSpeeds::Request  &req, kortex_driver::GetCommandedJointSpeeds::Response &res)
{
	Empty input;
	JointSpeeds output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCommandedJointSpeeds(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res)
{
	GripperCommand input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendGripperCommand(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res)
{
	GripperRequest input;
	ToProtoData(req.input, &input);
	Gripper output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredGripperMovement(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCommandedGripperMovement(kortex_driver::GetCommandedGripperMovement::Request  &req, kortex_driver::GetCommandedGripperMovement::Response &res)
{
	GripperRequest input;
	ToProtoData(req.input, &input);
	Gripper output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCommandedGripperMovement(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res)
{
	Admittance input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetAdmittance(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetTwistWrenchReferenceFrame(kortex_driver::SetTwistWrenchReferenceFrame::Request  &req, kortex_driver::SetTwistWrenchReferenceFrame::Response &res)
{
	CartesianReferenceFrameRequest input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetTwistWrenchReferenceFrame(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res)
{
	OperatingModeInformation input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetOperatingMode(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ApplyEmergencyStop(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::ClearFaults(kortex_driver::ClearFaults::Request  &req, kortex_driver::ClearFaults::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ClearFaults(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetActiveMap(kortex_driver::GetActiveMap::Request  &req, kortex_driver::GetActiveMap::Response &res)
{
	MappingHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->GetActiveMap(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetControlMode(kortex_driver::GetControlMode::Request  &req, kortex_driver::GetControlMode::Response &res)
{
	Empty input;
	ControlModeInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControlMode(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res)
{
	Empty input;
	OperatingModeInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetOperatingMode(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res)
{
	ServoingModeInformation input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetServoingMode(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res)
{
	Empty input;
	ServoingModeInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetServoingMode(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ServoingModeNotification) > callback = std::bind(&BaseServices::cb_ServoingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationServoingModeTopic(callback, input);
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
void BaseServices::cb_ServoingModeTopic(Base::ServoingModeNotification notif)
{
	kortex_driver::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic.publish(ros_msg);
}

bool BaseServices::GetSequenceState(kortex_driver::GetSequenceState::Request  &req, kortex_driver::GetSequenceState::Response &res)
{
	SequenceHandle input;
	ToProtoData(req.input, &input);
	SequenceInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetSequenceState(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetProtectionZoneState(kortex_driver::GetProtectionZoneState::Request  &req, kortex_driver::GetProtectionZoneState::Response &res)
{
	ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
	ProtectionZoneInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetProtectionZoneState(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetActionExecutionState(kortex_driver::GetActionExecutionState::Request  &req, kortex_driver::GetActionExecutionState::Response &res)
{
	Empty input;
	ActionExecutionState output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetActionExecutionState(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->RestoreFactorySettings(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::RestoreNetworkFactorySettings(kortex_driver::RestoreNetworkFactorySettings::Request  &req, kortex_driver::RestoreNetworkFactorySettings::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->RestoreNetworkFactorySettings(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::Reboot(kortex_driver::Reboot::Request  &req, kortex_driver::Reboot::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Reboot(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::FactoryNotification) > callback = std::bind(&BaseServices::cb_FactoryTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationFactoryTopic(callback, input);
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
void BaseServices::cb_FactoryTopic(Base::FactoryNotification notif)
{
	kortex_driver::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic.publish(ros_msg);
}

bool BaseServices::GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res)
{
	Empty input;
	ControllerList output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConnectedControllers(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res)
{
	ControllerHandle input;
	ToProtoData(req.input, &input);
	ControllerState output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerState(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res)
{
	Empty input;
	ActuatorInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetActuatorCount(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res)
{
	Empty input;
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StartWifiScan(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res)
{
	Ssid input;
	ToProtoData(req.input, &input);
	WifiConfiguration output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetConfiguredWifi(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::NetworkNotification) > callback = std::bind(&BaseServices::cb_NetworkTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationNetworkTopic(callback, input);
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
void BaseServices::cb_NetworkTopic(Base::NetworkNotification notif)
{
	kortex_driver::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic.publish(ros_msg);
}

bool BaseServices::GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res)
{
	Empty input;
	ArmStateInformation output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetArmState(m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Base::ArmStateNotification) > callback = std::bind(&BaseServices::cb_ArmStateTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationArmStateTopic(callback, input);
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
void BaseServices::cb_ArmStateTopic(Base::ArmStateNotification notif)
{
	kortex_driver::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic.publish(ros_msg);
}

bool BaseServices::GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res)
{
	NetworkHandle input;
	ToProtoData(req.input, &input);
	IPv4Information output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Information(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::SetCountryCode(kortex_driver::SetCountryCode::Request  &req, kortex_driver::SetCountryCode::Response &res)
{
	CountryCode input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetCountryCode(input, m_CurrentDeviceID, m_apiOptions);
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

bool BaseServices::GetCountryCode(kortex_driver::GetCountryCode::Request  &req, kortex_driver::GetCountryCode::Response &res)
{
	Empty input;
	CountryCode output;
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCountryCode(m_CurrentDeviceID, m_apiOptions);
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
