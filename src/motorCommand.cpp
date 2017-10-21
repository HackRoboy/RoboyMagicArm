#include "motorCommand.hpp"

#include <roboy_communication_middleware/ControlMode.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <std_srvs/SetBool.h>

const int MotorCommand::c_usedMotors[c_motorCountUsed] = {8, 10, 5, 3, 12, 11};

MotorCommand::MotorCommand(std::string nodeName)
: m_isInitialized(false)
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
        {
            m_motorSetPoints[i] = 0;
        }

        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodeName);

        m_nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle);

        m_motorCommand = m_nodeHandle->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
        m_motorControl = m_nodeHandle->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/middleware/ControlMode");
        m_emergencyStop = m_nodeHandle->serviceClient<std_srvs::SetBool>("/roboy/middleware/EmergencyStop");

        m_isInitialized = true;
    }
}

void MotorCommand::setMotor(int motorId, int motorSetPoint)
{
    m_motorSetPoints[motorId] = motorSetPoint;
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

