#include "RobotAxis.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <cmath>
#include <functional>
#include "vmath/vmath.h"


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

#define PI 3.14159265

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

const int baseHeight = 148.5;
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
RobotAxis baseAxis = RobotAxis(baseMotor, baseStepAngle, baseHeight);
RobotAxis arm1Axis = RobotAxis(arm1Motor, arm1StepAngle, arm1Length);
RobotAxis arm2Axis = RobotAxis(arm2Motor, arm2StepAngle, arm2Length);
RobotAxis arm3Axis = RobotAxis(arm3Motor, arm3StepAngle, arm3Length);
RobotAxis gripperAxis = RobotAxis(gripperMotor, gripperStepAngle, gripperLength);


/*
 * Move and reorient joint endpoint to a new target point
 */
void moveJoint(float jointLength, float start[], float end[], const float target[]) {
    // Displacement from joint end point to target
    Vector2f deltaVector = Vector2f(target[0] - end[0], target[1] - end[1]);
    end[0] += deltaVector.x;
    end[1] += deltaVector.y;
    
    // Reorient current joint to align with previous joint endpoint
    deltaVector.x = end[0] - start[0];
    deltaVector.y = end[1] - start[1];
    
    // Rescale to joint length
    deltaVector.normalize();
    deltaVector *= jointLength;

    // Reassign previous joint endpoint with current joint start
    start[0] = end[0] - deltaVector.x;
    start[1] = end[1] - deltaVector.y;
}

void showJoint(float joint[]) {
    std::cout << "(" << joint[0] << ","<< joint[1] << ")" << std::endl;
}

/*
 * Iteratively moves the arm to the target point 
 * Inputs: Each joints starting coordinates
 */
void fabrik(const float start[], float basePos[], float joint1Pos[], float joint2Pos[], float joint3Pos[], 
            const float target[], float threshold) {
    
    // Check if target is reachable
    float totalDist = sqrt(pow(start[0] - target[0], 2) + pow(start[1] - target[1], 2));
    
    if(totalDist < (arm1Length + arm2Length + arm3Length)) {
        
        // Inital distance from the target
        float remainingDist = sqrt(pow(joint2Pos[0] - target[0], 2) + pow(joint2Pos[1] - target[1], 2));
        
        // Keep moving and reorienting until close enough to the target
        while(remainingDist > threshold) {
            
            /* 
             * REACH FORWARD
             * Move joint 3 endpoint to the target point
             * Reorient and move joint 2 endpoint to joint 1 start point
             * Reorient and move joint 1 endpoint to joint 2 start point
             */
            moveJoint(arm3Length, joint2Pos, joint3Pos, target);
            moveJoint(arm2Length, joint1Pos, joint2Pos, joint2Pos);
            moveJoint(arm1Length, basePos, joint1Pos, joint1Pos);

            /* 
             * REACH BACKWARD
             * Move joint 1 start point to robot home point
             * Reorient and move joint 2 start point to joint 1 endpoint
             * Reorient and move joint 3 start point to joint 2 endpoint
             */
            moveJoint(arm1Length, joint1Pos, basePos, start);
            moveJoint(arm2Length, joint2Pos, joint1Pos, joint1Pos);
            moveJoint(arm3Length, joint3Pos, joint2Pos, joint2Pos);
            
            showJoint(basePos);
            showJoint(joint1Pos);
            showJoint(joint2Pos);
            showJoint(joint3Pos);
            
            // Calculate remaining distance
            remainingDist = sqrt(pow(joint3Pos[0] - target[0], 2) + pow(joint3Pos[1] - target[1], 2));
            std::cout << "Remaining Distance: " << remainingDist << std::endl;
        }
    } else {
        std::cout << "Unreachable target: " << totalDist << " > " <<  (arm1Length + arm2Length + arm3Length) << std::endl;
    }
}



float calculateAngle(float joint[]) {
    return std::atan((joint[1] - baseHeight) / joint[0]) * 180 / PI;
}

float lawOfCos(float a, float b, float c) {
    return std::acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2*a*b)) * 180 / PI;
}


int main() {
	wiringPiSetup();
	std::cout << "WiringPi Ready" << std::endl;
	std::cout << "Welcome to Robot Arm Inverse Kinematics Demo!" << std::endl;
    
    float jointPositions[4][2] = { {0, baseHeight}, 
                                   {0, baseHeight + arm1Length}, 
                                   {0, baseHeight + arm1Length + arm2Length}, 
                                   {0, baseHeight + arm1Length + arm2Length + arm3Length} };
    
    const float start[2] = {0, baseHeight};
    
    while(true) {
        float target[2];

        std::cout << "X coordinate: ";
        std::cin >> target[0];

        std::cout << "Y coordinate: ";
        std::cin >> target[1];
        
        fabrik(start, jointPositions[0], jointPositions[1], jointPositions[2], jointPositions[3], target, 0.001);
    }

    
    // Vector2f arm1Vector = Vector2f(jointPositions[1][1] - jointPositions[0][0], jointPositions[1][1] - jointPositions[0][1]);
    // Vector2f arm2Vector = Vector2f(jointPositions[2][2] - jointPositions[1][1], jointPositions[2][1] - jointPositions[1][1]);
    // Vector2f arm3Vector = Vector2f(jointPositions[3][3] - jointPositions[0][2], jointPositions[3][1] - jointPositions[2][1]);



	return 0;
}


// const float basePosition[] = {0, baseHeight};

//     float threshold = 0.01;
//     float p0[] = {0, baseHeight};
//     float p1[] = {0, baseHeight + arm1Length};
//     float p2[] = {0, baseHeight + arm1Length + arm2Length};
//     float p3[] = {0, baseHeight + arm1Length + arm2Length + arm3Length};
//     const Vector2f j1 = Vector2f(p1[0] - basePosition[0], p1[1] - basePosition[1]);
//     const Vector2f j2 = Vector2f(p2[0] - p1[0], p2[1] - p1[1]);
//     const Vector2f j3 = Vector2f(p3[0] - p2[0], p3[1] - p2[1]);

//     arm1.setAcceleration(5);
// 	arm1.setMaxVelocity(30);
	
// 	arm2.setAcceleration(3);
// 	arm2.setMaxVelocity(9);

// 	arm3.setAcceleration(0.5);
// 	arm3.setMaxVelocity(1.6);

// 	base.setAcceleration(3);
// 	base.setMaxVelocity(8.5);
	
// 	int arrayLength = 6;
	        
// 	float xCoord[arrayLength] = {300, 300, 320, 250, 280,0};
// 	float yCoord[arrayLength] = {300, 200, 220, 300, 350, baseHeight + arm1Length + arm2Length + arm3Length - 5};
// 	float rot[arrayLength] = {-45, 45, 0, 15, -65, 35};
// 	for(int i = 0; i < arrayLength; i += 1) {	
// 		float targetPositions[3] = {xCoord[i], yCoord[i], rot[i]};
		
// 		fabrik(j1, j2, j3, basePosition, p0, p1, p2, p3, targetPositions, threshold);
	
// 		float baseAngle = targetPositions[2];
// 		float arm1Angle = 90 - calculateAngle(p1);
// 		float arm2Angle = 180 - lawOfCos(arm1Length, arm2Length, sqrt(pow(p2[1] - basePosition[1], 2) + pow(p2[0] - basePosition[0], 2)));
// 		float arm3Angle = 180 - lawOfCos(arm2Length, arm3Length, sqrt(pow(p3[1] - p1[1], 2) + pow(p3[0] - p1[0], 2)));
		
// 		std::cout << "FinalAngles" << std::endl;
// 		std::cout << arm1Angle << std::endl;
// 		std::cout << arm2Angle << std::endl;
// 		std::cout << arm3Angle << std::endl;

// 		showJoint(p1);
// 		showJoint(p2);
// 		showJoint(p3); 
		
// 		std::thread baseThread(goToAngle, std::ref(base), baseAngle, baseMultiplier);
// 		std::thread arm1Thread(goToAngle, std::ref(arm1), arm1Angle, arm1Multiplier);
// 		std::thread arm2Thread(goToAngle, std::ref(arm2), arm2Angle, arm2Multiplier);
// 		std::thread arm3Thread(goToAngle, std::ref(arm3), arm3Angle, arm3Multiplier);
		
// 		baseThread.join();
// 		arm1Thread.join();
// 		arm2Thread.join();
// 		arm3Thread.join();
// 	}
	


// 	std::cout << "Arm1 Position " << stepToAngle(arm1.getCurrentPosition(), arm1Multiplier) << std::endl;
// 	std::cout << "Arm2 Position " << stepToAngle(arm2.getCurrentPosition(), arm2Multiplier) << std::endl;
// 	std::cout << "Arm3 Position " << stepToAngle(arm3.getCurrentPosition(), arm3Multiplier) << std::endl;
	