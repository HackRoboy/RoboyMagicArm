#ifndef MOTORCOMMAND_HPP_
#define MOTORCOMMAND_HPP_

#include "ros/ros.h"
#include <string>

class MotorCommand
{

public:
	MotorCommand(std::string nodeName);
	virtual ~MotorCommand();

	//// Set current set point of given motorId
	void setMotor(int motorId, int motorSetPoint);

	//// Returns current set point of given motorId
	int getMotor(int motorId);

	//// Publishes one set point message
	void publishMotorCmd();

	static const int c_motorCountUsed = 6;

private:
	//// Initializes ROS node and memory
	void init(std::string nodeName);

	int m_motorSetPoints[c_motorCountUsed]; //> Actual set points

    ros::NodeHandlePtr m_nodeHandle; //> Handle for ROS node
    ros::Publisher m_motorCommand; //> Motor Command Publisher
    ros::ServiceClient m_motorControl; //> Motor Control Service Client
    ros::ServiceClient m_emergencyStop; //> Motor Emergency Stop Service Client

    bool m_isInitialized;

    static const int c_motorCountTotal = 14;
    static const int c_usedMotors[c_motorCountUsed];
};

#endif /* MOTORCOMMAND_HPP_ */
