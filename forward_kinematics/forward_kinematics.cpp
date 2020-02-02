#include "Stepper/Stepper.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <ctime>
#include <cmath>

#define PI 3.14159265

const int baseSwitch = 24;
const int baseDir = 9;
const int baseTrig = 8;
const int baseMicroStep = 8;
const float baseMultiplier = -2.7778 * baseMicroStep; // steps per degree <-- with ALL gearing included
const float baseHeight = 148.5;
const float baseDPS = 1; // baseMultiplier;

const float arm1Length = 145;//145
const int arm1Switch = 25;
const int arm1Dir = 2;
const int arm1Trig = 0;
const int arm1MicroStep = 8;
const float arm1Multiplier = 26.064 * arm1MicroStep; // steps per degree <-- with ALL gearing included
const float arm1Zero = 65.0;

const float arm2Length = 125;//125
const int arm2Switch = 27;
const int arm2Dir = 13;
const int arm2Trig = 12;
const int arm2MicroStep = 8;
const float arm2Multiplier = -12.3016 * arm2MicroStep; // steps per degree <-- with ALL gearing included

const float arm3Length = 128.581;//128.581
const int arm3Switch = 28;
const int arm3Dir = 22;
const int arm3Trig = 21;
const int arm3MicroStep = 8;
const float arm3Multiplier = 2.46032 * arm3MicroStep; // steps per degree <-- with ALL gearing included

const int gripperDir = 10;
const int gripperTrig = 6;
const int gripperMicroStep = 8;
const float gripperMultiplier = 1.0 * (float) gripperMicroStep;

Stepper base(baseDir, baseTrig, baseMicroStep);
Stepper arm1(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripper(gripperDir, gripperTrig, gripperMicroStep);

int angleToStep(float desiredAngle, float axisMultiplier) {
	return (int) (desiredAngle * axisMultiplier);
}

void goToAngle(Stepper &axis, float desiredAngle, float axisMultiplier) {
	int stepsToTake = angleToStep(desiredAngle, axisMultiplier) - axis.getCurrentPosition();
	axis.relStep(stepsToTake);
}


void setup() {
	pinMode(baseSwitch, INPUT);
	pinMode(arm1Switch, INPUT);
	pinMode(arm2Switch, INPUT);
	pinMode(arm3Switch, INPUT);
	std::cout << "Pins Initialized" << std::endl;
}

int main() {
	wiringPiSetup();
	std::cout << "WiringPi Ready" << std::endl;
	setup();
	
	std::cout << "Welcome to Robot Arm Forward Kinematics Demo!" << std::endl;

	arm1.setAcceleration(5);
	arm1.setMaxVelocity(30);
	
	arm2.setAcceleration(3);
	arm2.setMaxVelocity(9);

	arm3.setAcceleration(0.5);
	arm3.setMaxVelocity(1.6);

	base.setAcceleration(3);
	base.setMaxVelocity(8.5);
	
	int baseAngle = 0;
	int arm1Angle = 0;
	int arm2Angle = 0;
	int arm3Angle = 0;
	int gripperAngle = 0;

	while(true) {
		std::cout << "Base Angle: ";
		std::cin >> baseAngle;

		std::cout << "Arm1 Angle: ";
		std::cin >> arm1Angle;

		std::cout << "Arm2 Angle: ";
		std::cin >> arm2Angle;
		
		std::cout << "Arm3 Angle: ";
		std::cin >> arm3Angle;

		std::cout << "Gripper Angle: ";
		std::cin >> gripperAngle;

		std::thread baseThread(goToAngle, std::ref(base), baseAngle, baseMultiplier);
		std::thread arm1Thread(goToAngle, std::ref(arm1), arm1Angle, arm1Multiplier);
		std::thread arm2Thread(goToAngle, std::ref(arm2), arm2Angle, arm2Multiplier);
		std::thread arm3Thread(goToAngle, std::ref(arm3), arm3Angle, arm3Multiplier);
		std::thread gripperThread(goToAngle, std::ref(gripper), gripperAngle, gripperMultiplier);
		
		baseThread.join();
		arm1Thread.join();
		arm2Thread.join();
		arm3Thread.join();
		gripperThread.join();
	}
		
	return 0;
}
