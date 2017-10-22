#ifndef MOTORCOMMAND_HPP_
#define MOTORCOMMAND_HPP_

#include "ros/ros.h"
#include <roboy_communication_middleware/MotorStatus.h>
#include <string>
#include <vector>

class MotorCommand
{

public:
    MotorCommand(std::string nodeName);
    virtual ~MotorCommand();

    //// Set current set point of given motorId
    void setMotor(int motorId, int motorSetPoint);

    //// Increases current set point of given motorId with given increment
    void increaseMotor(int motorId, int increment);

    //// Returns current set point of given motorId
    int getMotor(int motorId);

    //// Publishes one set point message
    void publishMotorCmd();

    //// Stops the robot or clears the lock via argument via Roboy service
    void callEmergencyStop(bool stop = true);

    //// Sets the control mode via Roboy service
    void setControlMode(int mode);

    //// Function for processing subscribed callbacks
    void processMotorStatus();

    //// Sets desired motor set point to measured position
    void mapMeasPosToDesiredPos();

    //// Resets position to initial saved position from mapMeasPosToDesiredPos()
    void resetToInitialPos();

    //// Saves current set points for using as a look-up table
    void saveCurrentPosition();

    //// Saves current set points for using as a look-up table
    void deleteLastEntry();

    //// Export LUT to csv
    void exportLUT(unsigned int name);

    static const int c_motorCountUsed = 6;

private:
    //// Initializes ROS node and memory
    void init(std::string nodeName);

    //// Callback function for receiving motor status message
    void motorStatusCb(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

    // MotorCommand memory
    int m_motorSetPoints[c_motorCountUsed]; //> Actual set points
    int m_motorSetPointsInit[c_motorCountUsed]; //> Initial set points

    ros::NodeHandlePtr m_nodeHandle; //> Handle for ROS node
    ros::Publisher m_motorCommand; //> Motor Command Publisher
    ros::ServiceClient m_motorControl; //> Motor Control Service Client
    ros::ServiceClient m_emergencyStop; //> Motor Emergency Stop Service Client
    ros::Subscriber m_motorStatus; //> Motor Status subscriber

    bool m_isInitialized;

    static const int c_motorCountTotal = 14;
    static const int c_usedMotors[c_motorCountUsed];

    // MotorStatus memory
    int m_counter;
    int m_motorData[c_motorCountTotal][4];

    enum controlMode{POSITION, VELOCITY, DISPLACEMENT};

    std::vector<int> m_lut[c_motorCountUsed];
};

#endif /* MOTORCOMMAND_HPP_ */
