#include "Stepper/Stepper.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <ctime>

Stepper base(9, 8, 4);
Stepper arm1(2, 0, 4);
Stepper arm2(13, 12, 4);
Stepper arm3(22, 21, 4);
Stepper gripper(10, 6, 8);

const int baseSwitch = 24;
const int arm1Switch = 25;
const int arm2Switch = 27;
const int arm3Switch = 28;

const float arm1Length = 145;
const float arm2Length = 125;
const float arm3Length = 128.581;

void home(Stepper& axis, int homeSwitch) {
	int switchCount = 0;
	while(switchCount != 20) {
		int state = digitalRead(homeSwitch);;
		if (state == 1) {
			switchCount += 1;
		}
		axis.relStep(1);
	}
	
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
	gripper.setAcceleration(5);
	gripper.absStep(16000);
	gripper.absStep(0);
	gripper.absStep(8000);
	gripper.absStep(0);
	
	
	
	return 0;
}
