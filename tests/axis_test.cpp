#include "RobotAxis.hpp"
#include <iostream>
#include <ncurses.h>
#include <thread>
#include <atomic>
#include <string>
#include <chrono>

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

 /*
 *  Positive Angle is CW
 *  Negative Angle is CCW
 *  Directions based on top view and arm1Motor back view
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


std::atomic<bool> isBaseComplete(true);

void baseThreadRotate(float desiredAngle) {
    isBaseComplete.store(false);
    baseJoint.rotate(desiredAngle);
    isBaseComplete.store(true);
}

int main() {
    wiringPiSetup();
    baseJoint.setDirection(false);
    arm1Joint.setDirection(true);
    arm2Joint.setDirection(false);
    arm3Joint.setDirection(true);
    gripperJoint.setDirection(true);

    float rps = 0.5;
    float delayMicro = 1000000 / (rps * 8000);
    
    auto prevMicros = std::chrono::steady_clock::now();;
    while(true) {
        auto currentMicros = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(currentMicros - prevMicros).count() >= delayMicro) {
            currentMicros = std::chrono::steady_clock::now();
            baseJoint.rotate(1, rps);
            prevMicros = currentMicros;
        }
    }

    // float baseAngle = 0;
    // float arm1Angle = 0;
    // float arm2Angle = 0;
    // float arm3Angle = 0;
    // float gripperAngle = 0;
    
    // initscr();
    // noecho();
    // cbreak();
    // keypad(stdscr, TRUE);
    // scrollok(stdscr,TRUE);
    
    // std::thread baseThread(baseThreadRotate, baseAngle);

    // int ch;
    // auto prevMillis = std::chrono::steady_clock::now();

    // while(true) {
    //     auto currentMillis = std::chrono::steady_clock::now();
        
    //     if(std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - prevMillis).count() >= 1) {
    //         prevMillis = currentMillis;
            
    //         ch = getch();
    //         switch(ch) {
    //             case KEY_RIGHT:
    //                 baseAngle = 5;
    //                 break;
                
    //             case KEY_LEFT:
    //                 baseAngle = -5;
    //                 break;
                
    //             case KEY_UP:
    //                 arm1Angle += 1;
    //                 break;
                
    //             case KEY_DOWN:
    //                 arm1Angle -= 1;
    //                 break;
                
    //             case 119:
    //                 arm2Angle += 1;
    //                 break;
                
    //             case 115:
    //                 arm2Angle -= 1;
    //                 break;
    //         }
    //     }

    //     printw("Target Base Angle %f  Actual Base Angle %f \n", baseAngle, baseJoint.getCurrentAngle());
    //     refresh();

    //     if(isBaseComplete.load()) {
    //         baseThread.detach();
    //         baseThread = std::thread(baseThreadRotate, baseAngle);
    //     }
    // }

	return 0;
}
