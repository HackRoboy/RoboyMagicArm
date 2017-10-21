#include <stdio.h>
#include <math.h>
#include <ncurses.h>  //add in CMake!  Ref. https://answers.ros.org/question/69022/how-to-link-to-ncurses-library-in-the-make-file/
#include "motorCommand.hpp"
#include "ros/ros.h"
#include <sstream>

int main ()
{
    ROS_INFO("Magic arm node started");
    MotorCommand motorCommand(std::string("magic_arm"));
    ROS_INFO("Magic arm node initialized");

    ROS_INFO("M1 (8)\tM2 (10)\tM3 (5)\tM4 (3)\tM5 (12)\tM6 (11)\n\t\t\tUP:\tA\tS\tD\tF\tG\tH\n\t\t\tDOWN:\tZ\tX\tC\tV\tB\tN\n");

    int c; //user keyboard input
    int motorAngle[6] = { 0 };  //int/float?!
    int s, base; //"scale" = base ^ pow
    s = 1;
    base = 2;

    do
    {
        c = getchar();   //need "Enter" to cont.
        switch(c)
        {
            //---Controlling Motor1~6---
            case 'A':
            case 'a':
                motorAngle[0] = motorAngle[0] + s;
                break;
            case 'Z':
            case 'z':
                motorAngle[0] = motorAngle[0] - s;
                break;
            case 'S':
            case 's':
                motorAngle[1] = motorAngle[1] + s;
                break;
            case 'X':
            case 'x':
                motorAngle[1] = motorAngle[1] - s;
                break;
            case 'D':
            case 'd':
                motorAngle[2] = motorAngle[2] + s;
                break;
            case 'C':
            case 'c':
                motorAngle[2] = motorAngle[2] - s;
                break;
            case 'F':
            case 'f':
                motorAngle[3] = motorAngle[3] + s;
                break;
            case 'V':
            case 'v':
                motorAngle[3] = motorAngle[3] - s;
                break;
            case 'G':
            case 'g':
                motorAngle[4] = motorAngle[4] + s;
                break;
            case 'B':
            case 'b':
                motorAngle[4] = motorAngle[4] - s;
                break;
            case 'H':
            case 'h':
                motorAngle[5] = motorAngle[5] + s;
                break;
            case 'N':
            case 'n':
                motorAngle[5] = motorAngle[5] - s;
                break;
            
            //---For scaling----
            case '0':
                s = 1;
                break;
            case '1':
                s = pow(base,1);
                break;
            case '2':
                s = pow(base,2);
                break;
            case '3':
                s = pow(base,3);
                break;
            case '4':
                s = pow(base,4);
                break;
            case '5':
                s = pow(base,5);
                break;
            case '6':
                s = pow(base,6);
                break;
            case '7':
                s = pow(base,7);
                break;
            case '8':
                s = pow(base,8);
                break;
            case '9':
                s = pow(base,9);
                break;
            default:
                continue;
        }

        std::stringstream motorSetPoints;
        motorSetPoints << "motorAngle[1~6] = [";
        for (int i = 0; i < 6; ++i)
        {
            motorSetPoints << ' ' << motorAngle[i] ;
            motorCommand.setMotor(i, motorAngle[i]);
        }
        motorSetPoints << "]\n";

        motorCommand.publishMotorCmd();

        ROS_INFO("%s", motorSetPoints.str().c_str());


    }while ((c!='q') && (c!='Q'));


    return 0;
}
