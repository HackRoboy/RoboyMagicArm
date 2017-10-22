#include "motorCommand.hpp"

#include <roboy_communication_middleware/ControlMode.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <std_srvs/SetBool.h>

#include <iostream>
#include <fstream>

const int MotorCommand::c_usedMotors[c_motorCountUsed] = {8, 10, 5, 3, 12, 11};

MotorCommand::MotorCommand(std::string nodeName)
: m_isInitialized(false),
  m_counter(0)
{
    init(nodeName);
}

MotorCommand::~MotorCommand()
{
}

void MotorCommand::init(std::string nodeName)
{
    if (!m_isInitialized)
    {
        for (int i = 0; i < c_motorCountUsed; ++i)
            m_motorSetPoints[i] = 0;

        for (int i = 0; i < c_motorCountTotal; ++i)
            for (int j = 0; j < 4; ++j)
            	m_motorData[i][j] = 0;

        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodeName);

        m_nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle);

        m_motorCommand = m_nodeHandle->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
        m_motorControl = m_nodeHandle->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/middleware/ControlMode");
        m_emergencyStop = m_nodeHandle->serviceClient<std_srvs::SetBool>("/roboy/middleware/EmergencyStop");
        m_motorStatus = m_nodeHandle->subscribe("/roboy/middleware/MotorStatus", 1, &MotorCommand::motorStatusCb, this);

        m_isInitialized = true;
    }
}

void MotorCommand::motorStatusCb(const roboy_communication_middleware::MotorStatus::ConstPtr &msg)
{
	//ROS_INFO_THROTTLE(5, "Receiving motor status");
	m_counter++;
	//ROS_INFO("Count: %d", m_counter);
	for (int motor = 0; motor < c_motorCountTotal; motor++)
	{
		m_motorData[motor][0] = msg->position[motor];
		m_motorData[motor][1]= msg->velocity[motor];
		m_motorData[motor][2] = msg->displacement[motor];
		m_motorData[motor][3] = msg->pwmRef[motor];
		//ROS_INFO("Motor %d: %d", motor, m_motorData[motor][0]);
	}
}

void MotorCommand::setMotor(int motorId, int motorSetPoint)
{
    m_motorSetPoints[motorId] = motorSetPoint;
}

void MotorCommand::increaseMotor(int motorId, int increment)
{
    m_motorSetPoints[motorId] += increment;
}

int MotorCommand::getMotor(int motorId)
{
    return m_motorSetPoints[motorId];
}

void MotorCommand::publishMotorCmd()
{
    roboy_communication_middleware::MotorCommand msg;

    for(std::size_t i = 0; i != c_motorCountUsed; ++i)
    {
        msg.motors.push_back(c_usedMotors[i]);
        msg.setPoints.push_back(m_motorSetPoints[i]);
    }

    m_motorCommand.publish(msg);
}


void MotorCommand::callEmergencyStop(bool stop)
{
	std_srvs::SetBool msg;
    msg.request.data = stop;

    if (stop)
        ROS_INFO("Emergency stop locked");
    else
        ROS_INFO("Emergency stop unlocked");

    m_emergencyStop.call(msg);
}

void MotorCommand::processMotorStatus()
{
	ros::spinOnce();
}

void MotorCommand::mapMeasPosToDesiredPos()
{
    for(std::size_t i = 0; i != c_motorCountUsed; ++i)
    {
    	m_motorSetPoints[i] = m_motorData[c_usedMotors[i]][0];
    	m_motorSetPointsInit[i] = m_motorData[c_usedMotors[i]][0];
    }
}

void MotorCommand::resetToInitialPos()
{
    for(std::size_t i = 0; i != c_motorCountUsed; ++i)
    {
    	m_motorSetPoints[i] = m_motorSetPointsInit[0];
    }
}

void MotorCommand::setControlMode(int mode)
{
	roboy_communication_middleware::ControlMode msg;

	switch (mode)
	{
	case 1:
		msg.request.control_mode = POSITION;
		break;
	case 2:
		msg.request.control_mode = VELOCITY;
		break;
	case 3:
		msg.request.control_mode = DISPLACEMENT;
		break;
	default:
		return;
	}
	msg.request.setPoint = 0;

	memset(m_motorSetPoints, 0, sizeof(int) * c_motorCountUsed);
	publishMotorCmd();

	m_motorControl.call(msg);
    ROS_INFO("Control mode: %d (1: Pos, 2: Vel, 3: Dis)", mode);
}

void MotorCommand::saveCurrentPosition()
{
    for (std::size_t i = 0; i < c_motorCountUsed; ++i)
    {
    	m_lut[i].push_back(m_motorSetPoints[i]);
    }
    ROS_INFO("Saved set: %d %d %d %d %d %d", m_lut[0].back(),
    		m_lut[1].back(), m_lut[2].back(), m_lut[3].back(), m_lut[4].back(), m_lut[5].back());
}

void MotorCommand::deleteLastEntry()
{
	if (m_lut[0].size() > 0)
	    ROS_INFO("Erased set: %d %d %d %d %d %d", m_lut[0].back(),
	    		m_lut[1].back(), m_lut[2].back(), m_lut[3].back(), m_lut[4].back(), m_lut[5].back());
	else
	    ROS_INFO("No entries to delete");

    for (std::size_t i = 0; i < c_motorCountUsed; ++i)
    {
    	if (m_lut[i].size() > 0)
    		m_lut[i].pop_back();
    }
}

void MotorCommand::exportLUT(unsigned int name)
{
	std::ofstream file;
	std::ostringstream fileName;
	fileName << name << ".csv";
	file.open(fileName.str().c_str());
	for(std::size_t i = 0; i != m_lut[0].size(); i++)
	{
	    for (std::size_t motor = 0; motor < c_motorCountUsed - 1; ++motor)
	    {
	    	file << m_lut[motor].at(i) << ",";
	    }
	    file << m_lut[5].at(i) << "\n";
	}
	file.close();

    for (std::size_t motor = 0; motor < c_motorCountUsed; ++motor)
    {
    	m_lut[motor].clear();
    }
    ROS_INFO("Exported as %d.csv file in your current dir", name);
}
