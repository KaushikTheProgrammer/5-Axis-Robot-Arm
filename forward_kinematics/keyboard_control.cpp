#include "RobotAxis.hpp"
#include <iostream>
#include <ncurses.h>
#include <thread>
#include <atomic>
#include <string>
#include <chrono>
#include <stdlib.h>

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
Stepper baseMotor = Stepper(baseDir, baseTrig, baseMicroStep);
Stepper arm1Motor = Stepper(arm1Dir, arm1Trig, arm1MicroStep);
Stepper arm2Motor = Stepper(arm2Dir, arm2Trig, arm2MicroStep);
Stepper arm3Motor = Stepper(arm3Dir, arm3Trig, arm3MicroStep);
Stepper gripperMotor = Stepper(gripperDir, gripperTrig, gripperMicroStep);

// Objects representig each axis
RobotAxis baseJoint = RobotAxis(baseMotor, baseStepAngle, baseLength);
RobotAxis arm1Joint = RobotAxis(arm1Motor, arm1StepAngle, arm1Length);
RobotAxis arm2Joint = RobotAxis(arm2Motor, arm2StepAngle, arm2Length);
RobotAxis arm3Joint = RobotAxis(arm3Motor, arm3StepAngle, arm3Length);
RobotAxis gripperJoint = RobotAxis(gripperMotor, gripperStepAngle, gripperLength);


// void baseThreadRotate(float desiredAngle) {
//     isBaseComplete.store(false);
//     baseJoint.rotate(desiredAngle);
//     isBaseComplete.store(true);
// }

int main() {
    wiringPiSetup();
    baseJoint.setDirection(false);
    arm1Joint.setDirection(true);
    arm2Joint.setDirection(false);
    arm3Joint.setDirection(true);
    gripperJoint.setDirection(true);

    // float rps = 0.5;
    // float delayMicro = 1000000 / (rps * 8000);
    
    // auto prevMillis = std::chrono::steady_clock::now();
    // while(true) {
    //     auto currentMillis = std::chrono::steady_clock::now();
    //     if(std::chrono::duration_cast<std::chrono::microseconds>(currentMillis - prevMillis).count() >= delayMicro) {
    //         currentMillis = std::chrono::steady_clock::now();
    //         baseJoint.rotate(1, rps);
    //         prevMillis = currentMillis;
    //     }
    // }

    float baseAngle = 0;
    float arm1Angle = 0;
    float arm2Angle = 0;
    float arm3Angle = 0;
    float gripperAngle = 0;
    
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    scrollok(stdscr,TRUE);
    nodelay(stdscr, TRUE);

    int ch;
   
    while(true) {
        ch = getch();
        switch(ch) {
            case KEY_RIGHT:
                baseAngle += 1;
                break;
            
            case KEY_LEFT:
                baseAngle -= 1;
                break;
            
            case KEY_UP:
                arm1Angle += 1;
                break;
            
            case KEY_DOWN:
                arm1Angle -= 1;
                break;
            
            case 119:
                arm2Angle += 1;
                break;
            
            case 115:
                arm2Angle -= 1;
                break;
            
            case 100:
                arm3Angle += 1;
                break;
            
            case 97:
                arm3Angle -= 1;
                break;
            
            case 101:
                gripperAngle += 1;
                break;
            
            case 113:
                gripperAngle -= 1;
                break;
            
            case ERR:
                break;

            case 27:
                endwin();
                exit(EXIT_SUCCESS);
                break;
        }
        baseJoint.goToAngle(baseAngle, 0.1);
        arm1Joint.goToAngle(arm1Angle, 0.05);
        arm2Joint.goToAngle(arm2Angle, 0.05);
        arm3Joint.goToAngle(arm3Angle, 0.09);
        gripperJoint.goToAngle(gripperAngle, 0.1);
        printw("baseAngle %f arm1Angle %f %i %i \n", baseAngle, baseJoint.getCurrentAngle(), baseMotor.getCurrentPosition());
        refresh();
    }
	return 0;
}

        // currentMillis = std::chrono::steady_clock::now();
        // if(std::chrono::duration_cast<std::chrono::microseconds>(currentMillis - prevMillis).count() >= 100) {
        //     prevMillis = currentMillis;
        //     ch = getch();
        //     switch(ch) {
        //         case KEY_RIGHT:
        //             baseAngle += 1;
        //             break;
                
        //         case KEY_LEFT:
        //             baseAngle -= 1;
        //             break;
                
        //         case KEY_UP:
        //             arm1Angle += 1;
        //             break;
                
        //         case KEY_DOWN:
        //             arm1Angle -= 1;
        //             break;
                
        //         case 119:
        //             arm2Angle += 1;
        //             break;
                
        //         case 115:
        //             arm2Angle -= 1;
        //             break;
                
        //         case ERR:
        //             break;

        //         case 27:
        //             endwin();
        //             exit(EXIT_SUCCESS);
        //             break;
        //     }
        // }
        // auto before = std::chrono::steady_clock::now();
        // baseJoint.goToAngle(baseAngle, 0.1);
        // auto after = std::chrono::steady_clock::now();
        // float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
        // if(elapsed != 0) {
        //     printw("Time taken %f \n", elapsed);
        //     refresh();
        // }
        
    // }
