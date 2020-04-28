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
const int arm1Dir = 28;
const int arm1Trig = 27;
const int arm1MicroStep = 8;
const float arm1Multiplier = 26.064 * arm1MicroStep; // steps per degree <-- with ALL gearing included
const float arm1Zero = 65.0;

const float arm2Length = 125;//125
const int arm2Switch = 27;
const int arm2Dir = 11;
const int arm2Trig = 10;
const int arm2MicroStep = 8;
const float arm2Multiplier = -12.3016 * arm2MicroStep; // steps per degree <-- with ALL gearing included

const float arm3Length = 128.581;//128.581
const int arm3Switch = 28;
const int arm3Dir = 23;
const int arm3Trig = 22;
const int arm3MicroStep = 8;
const float arm3Multiplier = 2.46032 * arm3MicroStep; // steps per degree <-- with ALL gearing included

const int gripperDir = 2;
const int gripperTrig = 0;
const int gripperMicroStep = 8;

Stepper base(baseDir, baseTrig, baseMicroStep);
Stepper arm1(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripper(gripperDir, gripperTrig, gripperMicroStep);

int angleToStep(float desiredAngle, float axisMultiplier) {
	return (int) (desiredAngle * axisMultiplier);
}

void goToAngle(Stepper &axis, float desiredAngle, float axisMultiplier) {
	desiredAngle = 90 - desiredAngle;
	int stepsToTake = angleToStep(desiredAngle, axisMultiplier) - axis.getCurrentPosition();
	axis.relStep(stepsToTake);
}


void setup() {
	wiringPiSetup();
	std::cout << "Pins Initialized" << std::endl;
}

void configureAxes() {
	base.setAcceleration(3);
	base.setMaxVelocity(8.5);
    
	arm1.setAcceleration(5);
	arm1.setMaxVelocity(30);
	
	arm2.setAcceleration(3);
	arm2.setMaxVelocity(9);

	arm3.setAcceleration(0.5);
	arm3.setMaxVelocity(1.6);
}

int main() {
	setup();
	configureAxes();

	base.relStep(2000);
		
	return 0;
}