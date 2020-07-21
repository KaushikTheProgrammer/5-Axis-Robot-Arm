#include "RobotAxis.hpp"
#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

#define PI 3.14159265

/*
 * Stepper init:    (direction, trigger, microstepping)
 * RobotAxis init:  (&Stepper, step_angle, axis_length)
 *                  
 * Motor Hardware Setup:
 *                  
 *         Motor Gearbox * External Gearing * Microstepping
 *
 * Base scale      5:1 *              1:1 *             8 =      40:1
 * Arm1 scale  19.19:1 *           2.45:1 *             8 = 377.403:1
 * Arm2 scale      5:1 *              4:1 *             8 =     160:1
 * Arm3 scale      1:1 *              4:1 *             8 =      32:1
 * Gripper scale   1:1 *             40:1 *             8 =     320:1
 * 
 * 1.8 * scale = step_angle
 */

 /*
 *  Positive Angle is CW
 *  Negative Angle is CCW
 *  Directions based on top view and arm1Motor back view
 */


// Microstepping modes for each motor
const float baseMicroStep = 8;
const float arm1MicroStep = 8;
const float arm2MicroStep = 8;
const float arm3MicroStep = 8;
const float gripperMicroStep = 8;

// Step angle for each motor
const float baseStepAngle = 0.045;
const float arm1StepAngle = 0.00477;
const float arm2StepAngle = 0.01125;
const float arm3StepAngle = 0.0563;
const float gripperStepAngle = 0.00563;

// Control pins for each motor
const int baseDir = 15;
const int arm1Dir = 1;
const int arm2Dir = 5;
const int arm3Dir = 10;
const int gripperDir = 26;
const int baseTrig = 16;
const int arm1Trig = 4;
const int arm2Trig = 6;
const int arm3Trig = 11;
const int gripperTrig = 27;

const int baseLength = 148.5;
const int arm1Length = 145;
const int arm2Length = 125;
const int arm3Length = 128.581;
const int gripperLength = 50;

// Motors for each axis
Stepper baseMotor = Stepper(baseDir, baseTrig, baseMicroStep);
Stepper arm1Motor = Stepper(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2Motor = Stepper(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3Motor = Stepper(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripperMotor = Stepper(gripperDir, gripperTrig, gripperMicroStep);

// Objects representig each axis
RobotAxis baseJoint = RobotAxis(baseMotor, baseStepAngle, baseLength);
RobotAxis arm1Joint = RobotAxis(arm1Motor, arm1StepAngle, arm1Length);
RobotAxis arm2Joint = RobotAxis(arm2Motor, arm2StepAngle, arm2Length);
RobotAxis arm3Joint = RobotAxis(arm3Motor, arm3StepAngle, arm3Length);
RobotAxis gripperJoint = RobotAxis(gripperMotor, gripperStepAngle, gripperLength);

int main() {
	wiringPiSetup();
	std::cout << "WiringPi Ready" << std::endl;
    arm2Joint.setDirection(false);
    // arm1Joint.setAcceleration(0.01);


    RobotAxis joints[5] = {baseJoint, arm1Joint, arm2Joint, arm3Joint, gripperJoint};

    while(true) {
        std::cout << "Enter your command: ";
        std::string input;
        getline(std::cin, input);
        std::cout << input << std::endl;
        
        char command = input.at(0);
        int axis = (int)input.at(2) - 48;
        int angle = std::stoi(input.substr(4));
        // std::cout << command << std::endl;
        // std::cout << axis << std::endl;
        // std::cout << angle << std::endl;
        
        switch(command) {
            case 'a':
                joints[axis - 1].goToAngle(angle);
                break;
           
            case 'r':
                joints[axis - 1].rotate(angle);
                break;
            
            case 'q':
                exit (EXIT_SUCCESS);
           
            default:
                break;
        }

        std::cout << joints[axis - 1].getCurrentAngle() << std::endl;

        double baseAngle = joints[0].getCurrentAngle();
        double gripperAngle = joints[4].getCurrentAngle();
            
        double arm1Angle = (90 - joints[1].getCurrentAngle());
        double arm2Angle = arm1Angle - joints[2].getCurrentAngle();
        double arm3Angle = arm2Angle - joints[3].getCurrentAngle();
        std::cout << arm1Angle << std::endl;
        std::cout << arm2Angle << std::endl;
        std::cout << arm3Angle << std::endl;

        arm1Angle *= PI / 180.0;
        arm2Angle *= PI / 180.0;
        arm3Angle *= PI / 180.0;

        // Add in gripper motion
        double x_ee = (arm1Length * cos(arm1Angle)) + (arm2Length * cos(arm2Angle)) + (arm3Length * cos(arm3Angle));
        double y_ee = (arm1Length * sin(arm1Angle)) + (arm2Length * sin(arm2Angle)) + (arm3Length * sin(arm3Angle));
        std::cout << "x-coordinate of end effector: " << x_ee << std::endl;
        std::cout << "y-coordinate of end effector: " << y_ee << std::endl;
        std::cout << "base theta: " << baseAngle << std::endl;
    }

	
	return 0;
}
