#include "Stepper/Stepper.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <ctime>

const int baseSwitch = 24;
const int baseDir = 9;
const int baseTrig = 8;
const int baseMicroStep = 4;

const float arm1Length = 145;
const int arm1Switch = 25;
const int arm1Dir = 2;
const int arm1Trig = 0;
const int arm1MicroStep = 4;

const float arm2Length = 125;
const int arm2Switch = 27;
const int arm2Dir = 13;
const int arm2Trig = 12;
const int arm2MicroStep = 4;

const float arm3Length = 128.581;
const int arm3Switch = 28;
const int arm3Dir = 22;
const int arm3Trig = 21;
const int arm3MicroStep = 4;

const int gripperDir = 10;
const int gripperTrig = 6;
const int gripperMicroStep = 8;

Stepper base(baseDir, baseTrig, baseMicroStep);
Stepper arm1(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripper(gripperDir, gripperTrig, gripperMicroStep);

void home(Stepper& axis, int homeSwitch) {
	int switchCount = 0;
	while(switchCount != 20) {
		int state = digitalRead(homeSwitch);
		if (state == 1) {
			switchCount += 1;
		}
		axis.relStep(1);
	}
}

float angleToStep(Stepper& axis, float angle) {
	return angle * ( (float) axis.getMaxSteps() / 360);
}


void setup() {
	pinMode(baseSwitch, INPUT);
	pinMode(arm1Switch, INPUT);
	pinMode(arm2Switch, INPUT);
	pinMode(arm3Switch, INPUT);
}

int main() {
	wiringPiSetup();
	setup();

	home(base, baseSwitch);
	home(arm1, arm1Switch);
	home(arm2, arm2Switch);
	home(arm3, arm3Switch);
	
	return 0;
}
