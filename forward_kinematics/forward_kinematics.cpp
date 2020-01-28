#include "Stepper/Stepper.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <ctime>
#include <cmath>

const int baseSwitch = 24;
const int baseDir = 9;
const int baseTrig = 8;
const int baseMicroStep = 8;
const float baseMultiplier = 2.7778 * baseMicroStep; // steps per degree <-- with ALL gearing included
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
const float arm2Multiplier = 12.3016 * arm2MicroStep; // steps per degree <-- with ALL gearing included

const float arm3Length = 128.581;//128.581
const int arm3Switch = 28;
const int arm3Dir = 22;
const int arm3Trig = 21;
const int arm3MicroStep = 8;
const float arm3Multiplier = 2.46032 * arm3MicroStep; // steps per degree <-- with ALL gearing included

const int gripperDir = 10;
const int gripperTrig = 6;
const int gripperMicroStep = 8;


#define PI 3.14159265

// gripper max = 14500

Stepper base(baseDir, baseTrig, baseMicroStep);
Stepper arm1(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripper(gripperDir, gripperTrig, gripperMicroStep);

void home(Stepper& axis, int homeSwitch) {
	int switchCount = 0;
	while(switchCount != 15) {
		int state = digitalRead(homeSwitch);
		if (state == 1) {
			switchCount += 1;
		}
		axis.relStep(1);
	}
	axis.setCurrentPosition(0);
}

int angleToStep(float desiredAngle, float axisMultiplier) {
	return (int) (desiredAngle * axisMultiplier);
}

void goToAngle(Stepper &axis, float desiredAngle, float axisMultiplier) {
	int stepsToTake = angleToStep(desiredAngle, axisMultiplier) - axis.getCurrentPosition();
	std::cout << stepsToTake << std::endl;
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
	arm1.setAcceleration(2.5);
	arm1.relStep(-18766);//18766
	
	// goToAngle(arm1, 90, arm1Multiplier);
	
	/*while(true) {
		
		std::cout << "1 - Home All Axes, 2 - Enter joint angles, 3 - Quit Demo" << std::endl;
		
		int option;
		std::cout << "What would you like to do? ";
		std::cin >> option;
		
		switch(option) {
			case 1:
			{
				std::cout << "Base Homing" << std::endl;
				//home(base, baseSwitch);
				
				std::cout << "Arm1 Homing" << std::endl;
				home(arm1, arm1Switch);
				
				std::cout << "Arm2 Homing" << std::endl;
				//home(arm2, arm2Switch);
				
				std::cout << "Arm3 Homing" << std::endl;
				//home(arm3, arm3Switch);
				
				std::cout << "Robot Homing Finished" << std::endl;
				
				arm1.relStep(-angleToStep(arm1Zero, arm1Multiplier));
				arm1.setCurrentPosition(0);
			}
			break;
			
			case 2:
			{	
				float baseAngle;
				float arm1Angle;
				float arm2Angle;
				float arm3Angle;
				int gripperAngle;
				
				std::cout << "Please enter an angle for the Base: ";
				std::cin >> baseAngle;
				
				std::cout << "Please enter an angle for Joint 1: ";
				std::cin >> arm1Angle;
				
				
				std::cout << "Please enter an angle for Joint 2: ";
				std::cin >> arm2Angle;
				
				std::cout << "Please enter an angle for Joint 3: ";
				std::cin >> arm3Angle;
				
				std::cout << "Gripper Open(1) or Close(2): ";
				std::cin >> gripperAngle;
				arm1Angle -= 90;
				arm2Angle -= 90;
				arm3Angle -= 90;
				
				//arm2Angle += arm1Angle
				
				std::cout << "Moving to those angles...." << std::endl;
				// goToAngle here
				//goToAngle(base, baseAngle, baseMultiplier);
				//goToAngle(arm1, arm1Angle, arm1Multiplier);
				goToAngle(arm2, arm2Angle, arm2Multiplier);
				goToAngle(arm3, arm3Angle, arm3Multiplier);
				
				if (gripperAngle == 1) {
					gripper.absStep(0);
				} else if (gripperAngle == 2) {
					gripper.absStep(14000);
				}
				
				
				std::cout << "Destination Reached!" << std::endl;
				std::cout << "Calculating Final Positions...." << std::endl;
				
				float p1X = arm1Length * cos(arm1Angle * PI / 180.0);
				float p1Y = arm1Length * sin(arm1Angle * PI / 180.0) + baseHeight;
				
				float p2X = p1X + arm2Length * cos(arm2Angle * PI / 180.0);
				float p2Y = p1Y + arm2Length * sin(arm2Angle * PI / 180.0);
				
				float p3X = p2X + arm3Length * cos(arm3Angle * PI / 180.0);
				float p3Y = p2Y + arm3Length * sin(arm3Angle * PI / 180.0);
				
				std::cout << "Point 1: (" << p1X << ", " << p1Y << ")" << std::endl;
				std::cout << "Point 2: (" << p2X << ", " << p2Y << ")" << std::endl;
				std::cout << "Point 3: (" << p3X << ", " << p3Y << ")" << std::endl;
			}
			break;
		
			case 3:
				exit(0);
			
			default:
				std::cout << "Invalid Input! Please choose a valid option" << std::endl;
		}	
		
	}*/
		
	return 0;
}
