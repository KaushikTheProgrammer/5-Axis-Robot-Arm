#include "Stepper/Stepper.hpp"
#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <cmath>



Stepper testStepper = Stepper(15, 16, 40);

int main() {
    wiringPiSetup();
    testStepper.setAcceleration(0.5);
    testStepper.relStep(1000);
    
    // auto start = std::chrono::steady_clock::now();
    // for(int i = 0; i < 1000; i += 1) {
    //     testStepper.calculateParameters(i);   
    // }
    
    // auto end = std::chrono::steady_clock::now();
    // double executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // std::cout << "Operations took: " << executionTime << " seconds" << std::endl;
}










// auto funcStart = std::chrono::steady_clock::now();
    
//     for(int i = 1; i < 1000000; i += 1) {
//         int x = calcInvSqrt(i);
//     }
    
//     auto funcEnd = std::chrono::steady_clock::now();
//     double funcTime = std::chrono::duration_cast<std::chrono::milliseconds>(funcEnd - funcStart).count();

//     auto start = std::chrono::steady_clock::now();
    
//     for(int i = 1; i < 1000000; i += 1) {
//         int y = 1 / sqrt(i);
//     }
    
//     auto end = std::chrono::steady_clock::now();
//     double execTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    
//     std::cout << "Function took: " << funcTime << " seconds and Standard took " << execTime << " seconds" << std::endl;