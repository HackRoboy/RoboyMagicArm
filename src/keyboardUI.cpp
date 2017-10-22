#include <stdio.h>
#include <math.h>
#include <ncurses.h>  //add in CMake!  Ref. https://answers.ros.org/question/69022/how-to-link-to-ncurses-library-in-the-make-file/
#include "motorCommand.hpp"
#include "ros/ros.h"
#include <sstream>

static const int c_usedMotorCount = 6;
static const int c_scalePos[c_usedMotorCount] = {-2000, 2000, 2000, -2000, 2000, 2000};
static const int c_initialForce[c_usedMotorCount] = {0, 320, 250, -290, 0, 100};




int main ()
{
    ROS_INFO("Magic arm node started");
    MotorCommand motorCommand(std::string("magic_arm"));
    ROS_INFO("Magic arm node initialized");

    int c; //user keyboard input
    int s = 1;
	ros::Rate secondSleep(1);
	ros::Rate thirdSecondSleep(0.3);

	unsigned int countExports = 1;


	motorCommand.setControlMode(3); // dis
    for (int i = 0; i < 6; ++i)
        motorCommand.setMotor(i, 0);
    motorCommand.publishMotorCmd();
	for(int count = 0; count < 5; ++count)
		secondSleep.sleep();
    for (int i = 0; i < 6; ++i)
        motorCommand.setMotor(i, c_initialForce[i]);
    motorCommand.publishMotorCmd();
	secondSleep.sleep();
    ROS_INFO("Magic arm moves to initial force controlled position");

    for(int count = 0; count < 10; ++count)
    	secondSleep.sleep();

	motorCommand.processMotorStatus();

	motorCommand.setControlMode(1); // dis
	thirdSecondSleep.sleep();
	motorCommand.mapMeasPosToDesiredPos();
    motorCommand.publishMotorCmd();
    ROS_INFO("Magic arm switched to position control");
    ROS_INFO("M1 (8)\tM2 (10)\tM3 (5)\tM4 (3)\tM5 (12)\tM6 (11)\n\t\t\tUP:\tA\tS\tD\tF\tG\tH\n\t\t\tDOWN:\tZ\tX\tC\tV\tB\tN\n");
    do
    {

    	motorCommand.processMotorStatus();


        c = getchar();   //need "Enter" to cont.
        int direction = 0;
        int motor = 0;
        switch(c)
        {
            //---Controlling Motor1~6---
            case 'A':
            case 'a':
            	motor = 0;
            	direction = 1;
                break;
            case 'Z':
            case 'z':
            	motor = 0;
            	direction = -1;
                break;
            case 'S':
            case 's':
            	motor = 1;
            	direction = 1;
                break;
            case 'X':
            case 'x':
            	motor = 1;
            	direction = -1;
                break;
            case 'D':
            case 'd':
            	motor = 2;
            	direction = 1;
                break;
            case 'C':
            case 'c':
            	motor = 2;
            	direction = -1;
                break;
            case 'F':
            case 'f':
            	motor = 3;
            	direction = 1;
                break;
            case 'V':
            case 'v':
            	motor = 3;
            	direction = -1;
                break;
            case 'G':
            case 'g':
            	motor = 4;
            	direction = 1;
                break;
            case 'B':
            case 'b':
            	motor = 4;
            	direction = -1;
                break;
            case 'H':
            case 'h':
            	motor = 5;
            	direction = 1;
                break;
            case 'N':
            case 'n':
            	motor = 5;
            	direction = -1;
                break;
            
            //---For scaling----
            case '0':
                s = 1;
                break;
            case '1':
                s = 1;
                break;
            case '2':
                s = 2;
                break;
            case '3':
                s = 3;
                break;
            case '4':
                s = 4;
                break;
            case '5':
                s = 5;
                break;
            case '6':
                s = 6;
                break;
            case '7':
                s = 7;
                break;
            case '8':
                s = 8;
                break;
            case '9':
                s = 9;
                break;
            case 'i':
            	motorCommand.setControlMode(1); // pos
                continue;
            case 'o':
            	motorCommand.setControlMode(2); // vel
                continue;
            case 'p':
            	motorCommand.setControlMode(3); // dis
                continue;
            case 'R':
            case 'r':
            	motorCommand.saveCurrentPosition();
            	continue;
            case 'T':
            case 't':
            	motorCommand.deleteLastEntry();
            	continue;
            case 'E':
            case 'e':
            	motorCommand.exportLUT(countExports);
            	countExports++;
            	continue;
            case 'l':
            case 'L':
            	motorCommand.resetToInitialPos();
                motorCommand.publishMotorCmd();
            	continue;
            case '.':
            	motorCommand.callEmergencyStop(true);
            	continue;
            case '-':
            	motorCommand.callEmergencyStop(false);
            	continue;
            default:
                continue;
        }

        motorCommand.increaseMotor(motor, direction * c_scalePos[motor] * s);

        std::stringstream motorSetPoints;
        motorSetPoints << "motorAngle[1~6] = [";
        for (int i = 0; i < 6; ++i)
        {
            motorSetPoints << ' ' << motorCommand.getMotor(i);
        }
        motorSetPoints << "]";
        motorSetPoints << " Scaling factor: " << s;

        motorCommand.publishMotorCmd();

        ROS_INFO("%s", motorSetPoints.str().c_str());

        ROS_INFO("M1 (8)\tM2 (10)\tM3 (5)\tM4 (3)\tM5 (12)\tM6 (11)\n\t\t\tUP:\tA\tS\tD\tF\tG\tH\n\t\t\tDOWN:\tZ\tX\tC\tV\tB\tN\n");

    }while ((c!='q') && (c!='Q'));


    return 0;
}
