#include "Stepper/Stepper.hpp"
#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <cmath>



Stepper testStepper = Stepper(15, 16, 4);

int main() {
    wiringPiSetup();
    testStepper.setAcceleration(1.5);
    testStepper.calculateParameters(799);
}