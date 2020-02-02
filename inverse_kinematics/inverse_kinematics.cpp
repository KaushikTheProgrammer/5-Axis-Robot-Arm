#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <cmath>
#include <functional>
#include "vmath/vmath.h"
#include "Stepper/Stepper.hpp"

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

Stepper base(baseDir, baseTrig, baseMicroStep);
Stepper arm1(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripper(gripperDir, gripperTrig, gripperMicroStep);

std::reference_wrapper<Stepper> armJoints[] = {arm1, arm2, arm3};
float armMultipliers[] = {arm1Multiplier, arm2Multiplier, arm3Multiplier};

void moveJoint(Vector2f joint, float prev[], float curr[], const float target[]) {
    Vector2f deltaVector = Vector2f(target[0] - curr[0], target[1] - curr[1]);
    curr[0] += deltaVector.x;
    curr[1] += deltaVector.y;
    deltaVector.x = curr[0] - prev[0];
    deltaVector.y = curr[1] - prev[1];
    deltaVector.normalize();
    deltaVector *= joint.length();
    prev[0] = curr[0] - deltaVector.x;
    prev[1] = curr[1] - deltaVector.y;
}

void showJoint(float joint[]) {
    std::cout << "(" << joint[0] << ","<< joint[1] << ")" << std::endl;
}

void fabrik(const Vector2f joint1, const Vector2f joint2, const Vector2f joint3, const float base[], float j0[], float j1[], float j2[], float j3[], const float target[], float threshold) {
    float totalDist = sqrt(pow(base[0] - target[0], 2) + pow(base[1] - target[1], 2));
    std::cout << totalDist << std::endl;

    if(totalDist < (joint1.length() + joint2.length() + joint3.length())) {
        float remainingDist = sqrt(pow(j2[0] - target[0], 2) + pow(j2[1] - target[1], 2));
        while(remainingDist > threshold) {
            // Reach back
            moveJoint(joint3, j2, j3, target);
            moveJoint(joint2, j1, j2, j2);
            moveJoint(joint1, j0, j1, j1);

            // Reach forward
            moveJoint(joint1, j1, j0, base);
            moveJoint(joint2, j2, j1, j1);
            moveJoint(joint3, j3, j2, j2);
            
            // showJoint(j0);
            // showJoint(j1);
            // showJoint(j2);
            // showJoint(j3);
            
            // Calculate remaining distance
            remainingDist = sqrt(pow(j3[0] - target[0], 2) + pow(j3[1] - target[1], 2));
            std::cout << remainingDist << std::endl;
        }
    }
    // std::cout << "Final Positions" << std::endl;
    // showJoint(j0);
    // showJoint(j1);
    // showJoint(j2);
    // showJoint(j3);
}

float calculateAngle(float joint[]) {
    return std::atan((joint[1] - baseHeight) / joint[0]) * 180 / PI;
}

int getPriority(const float axisMultiplier) {
    if(axisMultiplier == arm1Multiplier) {
        return 0;
    } else if(axisMultiplier == arm2Multiplier) {
        return 1;
    } else if(axisMultiplier == arm3Multiplier) {
        return 2;
    }
}

int angleToStep(float desiredAngle, float axisMultiplier) {
	return (int) (desiredAngle * axisMultiplier);
}

float stepToAngle(int currentPosition, float axisMultiplier) {
    return currentPosition / axisMultiplier;
}

void goToAngle(Stepper &axis, float desiredAngle, float axisMultiplier) {
	int stepsToTake = angleToStep(desiredAngle, axisMultiplier) - axis.getCurrentPosition();
	axis.relStep(stepsToTake);
}

float lawOfCos(float a, float b, float c) {
    return std::acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2*a*b)) * 180 / PI;
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
	
	std::cout << "Welcome to Robot Arm Inverse Kinematics Demo!" << std::endl;

	const float basePosition[] = {0, baseHeight};
    const Vector2f j1 = Vector2f(p1[0] - basePosition[0], p1[1] - basePosition[1]);
    const Vector2f j2 = Vector2f(p2[0] - p1[0], p2[1] - p1[1]);
    const Vector2f j3 = Vector2f(p3[0] - p2[0], p3[1] - p2[1]);

    float threshold = 0.01;
    float p0[] = {0, baseHeight};
    float p1[] = {0, baseHeight + arm1Length};
    float p2[] = {0, baseHeight + arm1Length + arm2Length};
    float p3[] = {0, baseHeight + arm1Length + arm2Length + arm3Length};

    arm1.setAcceleration(5);
	arm1.setMaxVelocity(30);
	
	arm2.setAcceleration(3);
	arm2.setMaxVelocity(9);

	arm3.setAcceleration(0.5);
	arm3.setMaxVelocity(1.6);

	base.setAcceleration(3);
	base.setMaxVelocity(8.5);
	float xCoord;
	float yCoord;
	
	while(true) {
		std::cin >> xCoord;
		std::cin >> yCoord;
		
		float targetPosition[] = {xCoord, yCoord};
	
        std::cout << "Arm1 Position " << showJoint(p1) << std::endl;
        std::cout << "Arm2 Position " << showJoint(p2) << std::endl;
        std::cout << "Arm3 Position " << showJoint(p3) << std::endl;
        
		fabrik(j1, j2, j3, basePosition, p0, p1, p2, p3, targetPosition, threshold);
		
		float arm1Angle = 90 - calculateAngle(p1);
		float arm2Angle = 180 - lawOfCos(arm1Length, arm2Length, sqrt(pow(p2[1] - basePosition[1], 2) + pow(p2[0] - basePosition[0], 2)));
		float arm3Angle = 180 - lawOfCos(arm2Length, arm3Length, sqrt(pow(p3[1] - p1[1], 2) + pow(p3[0] - p1[0], 2)));
		
      
		std::cout << "FinalAngles" << std::endl;

		std::cout << arm1Angle << std::endl;
		std::cout << arm2Angle << std::endl;
		std::cout << arm3Angle << std::endl;
		
		std::thread arm1Thread(goToAngle, std::ref(arm1), arm1Angle, arm1Multiplier);
		std::thread arm2Thread(goToAngle, std::ref(arm2), arm2Angle, arm2Multiplier);
		std::thread arm3Thread(goToAngle, std::ref(arm3), arm3Angle, arm3Multiplier);
		
		arm1Thread.join();
		arm2Thread.join();
		arm3Thread.join();

        // std::cout << "Arm1 Position " << stepToAngle(arm1.getCurrentPosition(), arm1Multiplier) << std::endl;
        // std::cout << "Arm2 Position " << stepToAngle(arm2.getCurrentPosition(), arm2Multiplier) << std::endl;
        // std::cout << "Arm3 Position " << stepToAngle(arm3.getCurrentPosition(), arm3Multiplier) << std::endl;
	}

	return 0;
}
