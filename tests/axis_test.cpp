#include "RobotAxis.hpp"
#include <iostream>


const float baseMultiplier = -2.7778 * 8; // steps per degree <-- with ALL gearing included
const float baseHeight = 148.5;
const float arm1Length = 145;//145
const float arm1Multiplier = 26.064 * 8; // steps per degree <-- with ALL gearing included
const float arm2Length = 125;//125
const float arm2Multiplier = -12.3016 * 8; // steps per degree <-- with ALL gearing included
const float arm3Length = 128.581;//128.581
const float arm3Multiplier = 2.46032 * 8; // steps per degree <-- with ALL gearing included

/*
 * Stepper init:    (direction, trigger, microstepping)
 * RobotAxis init:  (&Stepper, step_angle, axis_length)
 *                  
 * Motor Hardware Setup:
 *                  
 *         Motor Gearbox * External Gearing * Microstepiing
 *
 * Base gearing      5:1 *              1:1 *             8 =    40:1
 * Arm1 gearing     19:1 *           1.45:1 *             8 = 220.4:1
 * Arm2 gearing      5:1 *              4:1 *             8 =   160:1
 * Arm3 gearing      1:1 *              4:1 *             8 =    32:1
 * Gripper           1:1 *             40:1 *             8 =   320:1
 * 
 *
 *
 *
 *
 */

// Motors for each axis
Stepper baseMotor(15, 16, 8);
Stepper arm1Motor(1, 4, 8);
Stepper arm2Motor(5, 6, 8);
Stepper arm3Motor(10, 11, 8);
Stepper gripperMotor(26, 27, 8);

// Objects representig each axis
RobotAxis baseJoint(baseMotor, 1.8f, 148.5f);


void setup() {
	std::cout << "Pins Initialized" << std::endl;
    wiringPiSetup();
}

int main() {
		
	return 0;
}
