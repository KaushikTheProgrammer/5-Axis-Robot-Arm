#include "RobotAxis.hpp"
#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <cmath>
#include <string>

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
Stepper baseMotor = Stepper(baseDir, baseTrig, 4);
Stepper arm1Motor = Stepper(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2Motor = Stepper(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3Motor = Stepper(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripperMotor = Stepper(gripperDir, gripperTrig, gripperMicroStep);

// Objects representig each axis
RobotAxis baseJoint = RobotAxis(baseMotor, .45, baseLength);
RobotAxis arm1Joint = RobotAxis(arm1Motor, arm1StepAngle, arm1Length);
RobotAxis arm2Joint = RobotAxis(arm2Motor, arm2StepAngle, arm2Length);
RobotAxis arm3Joint = RobotAxis(arm3Motor, arm3StepAngle, arm3Length);
RobotAxis gripperJoint = RobotAxis(gripperMotor, gripperStepAngle, gripperLength);


int main() {
    wiringPiSetup();
    baseJoint.setAcceleration(0.5);
    float vel;
    std::cout << "Enter omega:";
    std::cin >> vel;
    baseJoint.setMaxVelocity(vel);
    while(true) {
		int stepCount;
		std::cin >> stepCount;
		baseJoint.goToAngle(stepCount, vel);
	}
}
