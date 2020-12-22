#include "RobotAxis.hpp"
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <cmath>
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


#define PI 3.14159265

// Microstepping modes for each motor
const float baseMicroStep = 8;
const float arm1MicroStep = 8;
const float arm2MicroStep = 8;
const float arm3MicroStep = 8;
const float gripperMicroStep = 8;

// Step angle for each motor
const double baseStepAngle = 0.045;
const double arm1StepAngle = 0.00477;
const double arm2StepAngle = 0.01125;
const double arm3StepAngle = 0.0563;
const double gripperStepAngle = 0.00563;

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

// Arm Constants
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


void showJoint(float joint[]) {
    std::cout << "(" << joint[0] << ","<< joint[1] << ")" << std::endl;
}


/*
 * Move and reorient joint endpoint to a new target point
 */
void moveJoint(float jointLength, float start[], float end[], const float target[]) {
    
    // Move joint end point to target
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

/*
 * Iteratively moves the arm to the target point 
 * Inputs: The starting coordinates of joint endpoints
 * start[] is the constant home position of the arm
 */
bool fabrik(const float start[], float basePos[], float joint1Pos[], float joint2Pos[], float joint3Pos[], 
            const float target[], float threshold, bool debug = 0) {
    
    // Check if target is reachable
    float totalDist = sqrt(pow(start[0] - target[0], 2) + pow(start[1] - target[1], 2));
    
    if(totalDist <= (arm1Length + arm2Length + arm3Length)) {
        
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

            // Calculate remaining distance
            remainingDist = sqrt(pow(joint3Pos[0] - target[0], 2) + pow(joint3Pos[1] - target[1], 2));
            
            if (debug) {
                showJoint(basePos);
                showJoint(joint1Pos);
                showJoint(joint2Pos);
                showJoint(joint3Pos);
                std::cout << "Remaining Distance: " << remainingDist << std::endl;
            }   
        }
        return true;
    } else {
        std::cout << "Unreachable target: " << totalDist << " > " <<  (arm1Length + arm2Length + arm3Length) << std::endl;
        return false;
    }
}


float calculateAngle(float jointStart[], float jointEnd[]) {
    float angle = std::atan((jointEnd[1] - jointStart[1]) / (jointEnd[0] - jointStart[0])) * 180 / PI;
    if (jointEnd[0] - jointStart[0] < 0) {
        angle = angle + 180;
    }
    return angle;
}

float lawOfCos(float l1, float l2, float a[], float b[], float c[]) {
    float cSide = sqrt(pow(c[0] - a[0], 2) + pow(c[1] - a[1], 2));
    
    float theta = 180 - (acos( (pow(cSide, 2) - pow(l1, 2) - pow(l2, 2)) / (-2 * l1 * l2) ) * 180 / PI);
    std::cout << "c: " << cSide << std::endl;
    std::cout << "acos: " << 180 - theta << std::endl;
    if( c[1] > ((b[1] - a[1] / b[0] - a[0]) * (c[0] - b[0]) + b[1]) ) {
        std::cout << "switiching angle" << std::endl;
        theta = -theta;
    }
    
    return theta;
}

void rotateAxis(RobotAxis& axis, float angle) {
    axis.goToAngle(angle);
}

int main(int argc, char** argv) {
	wiringPiSetup();
	std::cout << "WiringPi Ready" << std::endl;
	std::cout << "Welcome to Robot Arm Inverse Kinematics Demo!" << std::endl;

    arm3Axis.setAcceleration(3);
    arm3Axis.setMaxVelocity(5);

    arm2Axis.setDirection(false);
    arm2Axis.setMaxVelocity(0.15);

    arm1Axis.setMaxVelocity(0.1);
    
    // float pos[3] = {-40, 40, 0};

    // for(float p : pos) {
    //     arm1Axis.goToAngle(p);
    // }

    float jointPositions[4][2] = { {0, baseHeight}, 
                                   {0, baseHeight + arm1Length}, 
                                   {0, baseHeight + arm1Length + arm2Length}, 
                                   {0, baseHeight + arm1Length + arm2Length + arm3Length} };
    
    showJoint(jointPositions[0]);
    showJoint(jointPositions[1]);
    showJoint(jointPositions[2]);
    showJoint(jointPositions[3]);
    
    // NEED TO FIX STARTING POINT

    const float start[2] = {0, baseHeight};
    
    while(true) {
        float target[2];

        std::cout << "X coordinate: ";
        std::cin >> target[0];

        std::cout << "Y coordinate: ";
        std::cin >> target[1];

        // for(int y = 546; y >= -249; y -= 1) {
        //     int lBound = -sqrt(158006.25 - pow(y, 2));
        //     for(int x = lBound; x <= -lBound; x += 1) {

        //     }
        // }

        fabrik(start, jointPositions[0], jointPositions[1], jointPositions[2], jointPositions[3], target, 0.001, 0);
        showJoint(jointPositions[0]);
        showJoint(jointPositions[1]);
        showJoint(jointPositions[2]);
        showJoint(jointPositions[3]);
        
        float arm1Angle = 90 - calculateAngle(jointPositions[0], jointPositions[1]);
        float arm2Angle = lawOfCos(arm1Length, arm2Length, jointPositions[0], jointPositions[1], jointPositions[2]);
        float arm3Angle = lawOfCos(arm2Length, arm3Length, jointPositions[1], jointPositions[2], jointPositions[3]);
        
        std::cout << arm1Angle << std::endl;
        std::cout << arm2Angle << std::endl;
        std::cout << arm3Angle << std::endl;
        

        std::thread arm1Thread(rotateAxis, std::ref(arm1Axis), arm1Angle);
        std::thread arm2Thread(rotateAxis, std::ref(arm2Axis), arm2Angle);
        std::thread arm3Thread(rotateAxis, std::ref(arm3Axis), arm3Angle);
        
        arm1Thread.join();
        arm2Thread.join();
        arm3Thread.join();


        arm1Axis.goToAngle(arm1Angle);
        arm2Axis.goToAngle(arm2Angle);
        arm3Axis.goToAngle(arm3Angle);
    }
	return 0;
}