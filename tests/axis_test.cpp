#include "RobotAxis.hpp"
#include <iostream>

/*
 * Stepper init:    (direction, trigger, microstepping)
 * RobotAxis init:  (&Stepper, step_angle, axis_length)
 *                  
 * Motor Hardware Setup:
 *                  
 *         Motor Gearbox * External Gearing * Microstepping
 *
 * Base scale      5:1 *              1:1 *             8 =    40:1
 * Arm1 scale     19:1 *           1.45:1 *             8 = 220.4:1
 * Arm2 scale      5:1 *              4:1 *             8 =   160:1
 * Arm3 scale      1:1 *              4:1 *             8 =    32:1
 * Gripper scale   1:1 *             40:1 *             8 =   320:1
 * 
 * 1.8 * scale = step_angle
 */


// Microstepping modes for each motor
const float baseMicroStep = 8;
const float arm1MicroStep = 8;
const float arm2MicroStep = 8;
const float arm3MicroStep = 8;
const float gripperMicroStep = 8;

// Step angle for each motor
const float baseStepAngle = 0.045;
const float arm1StepAngle = 0.00817;
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
Stepper baseMotor = Stepper(baseDir, baseTrig, baseMicroStep, 0.2);
Stepper arm1Motor = Stepper(arm1Dir, arm1Trig, arm1MicroStep, 0.2);
Stepper arm2Motor = Stepper(arm2Dir, arm2Trig, arm2MicroStep, 0.2);
Stepper arm3Motor = Stepper(arm3Dir, arm3Trig, arm3MicroStep, 0.2);
Stepper gripperMotor = Stepper(gripperDir, gripperTrig, gripperMicroStep, 0.2);

// Objects representig each axis
RobotAxis baseJoint = RobotAxis(baseMotor, baseStepAngle, baseLength);
RobotAxis arm1Joint = RobotAxis(arm1Motor, arm1StepAngle, arm1Length);
RobotAxis arm2Joint = RobotAxis(arm2Motor, arm2StepAngle, arm2Length);
RobotAxis arm3Joint = RobotAxis(arm3Motor, arm3StepAngle, arm3Length);
RobotAxis gripperJoint = RobotAxis(gripperMotor, gripperStepAngle, gripperLength);

/*
 *  Positive Angle is CW
 *  Negative Angle is CCW
 */

int main() {
    wiringPiSetup();
    baseJoint.setDirection(false);
    
    baseJoint.goToAngle(45, 0.0625);
    std::cout << baseJoint.getCurrentAngle() << std::endl;
    baseJoint.goToAngle(0, 0.0625);
    std::cout << baseJoint.getCurrentAngle() << std::endl;
		
	return 0;
}
