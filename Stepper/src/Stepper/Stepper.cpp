/**
	Stepper.cpp
	Purpose: Library to be used with the TB6600 Stepper Driver and a Raspberry Pi

	@author Kaushik Prakash
	@version 1.0 03/01/19
*/


#include "Stepper.hpp"
#include <cmath>
#include <wiringPi.h>
#include <algorithm>
#include <iostream>

Stepper::Stepper() {}

Stepper::Stepper(const int DIRECTION_PIN, const int PULSE_PIN, const int MICRO_STEP_SIZE) {	
    wiringPiSetup();
    _directionPin = DIRECTION_PIN;
    _pulsePin = PULSE_PIN;
    
    _microStepSize = MICRO_STEP_SIZE;
    _maxSteps = 200 * _microStepSize;
    _initVel = 0.01 * _maxSteps;
    _maxVel = 1 * _maxSteps;
    _accel = 0.2 * _maxSteps;

    _currPosition = 0;
    _accelThreshold = 10;
    _minVelDelay = 1 / (0.1 * _maxSteps);

    pinMode(_directionPin, OUTPUT);
    pinMode(_pulsePin, OUTPUT);
}

/**
 * Calculates delays for every step in the routine
 * @param STEPS Number of steps to take
*/
void Stepper::calculateParameters(int STEPS) {
	STEPS = std::abs(STEPS); // Direction doesn't matter

    if(STEPS > _accelThreshold) {
        double multiplier = 1 / sqrt(2 * _accel);
        double maxVelDelay = (1 / _maxVel) * 1000000;

        for(int stepNumber = 0; stepNumber < (STEPS * 0.5); stepNumber += 1) {
            double currDelay = multiplier * 1 / sqrt(2 * static_cast<double>(stepNumber + 1)) * 1000000;
            if(currDelay < maxVelDelay) {
                currDelay = maxVelDelay;
            }
            _allDelays.push_back(currDelay);
        }

        int lastAccel = _allDelays.size() - 1;
        
        // Cruise for one step if needed
        if(STEPS % 2 != 0) {
            _allDelays.push_back(_allDelays.back());
        }

        // Decelerate at the same rate
        for(int i = lastAccel; i >= 0; i -= 1) {
            _allDelays.push_back(_allDelays[i]);
        }
        
    } else {
        for(int stepNumber = 0; stepNumber < STEPS; stepNumber += 1) {
            _allDelays.push_back(_minVelDelay);      
        }
    }
    // Print allDelays
    // for (int i=0; i<_allDelays.size() ; i++) {
    //     std::cout << i << " " <<  _allDelays[i] << 1 / (_allDelays[i] * _maxSteps) << std::endl;
    // }
    

    // Uncomment for testing; CHANGE THIS AFTER TESTING
    // std::vector<double>().swap(_allDelays); // Remove all delays for this routine and force a reallocation

 
    // float currDelay; // Delay period for each step
    // int accelSteps = _propAccel * STEPS; // Step number after which acceleration should stop
    // int velSteps = STEPS - accelSteps; // Step number after which constant velocity should stop
    
    // _multiplier = _maxAccel / static_cast<float>(pow(_frqcy, 2));   // Recalculate multiplier 
	// _maxVelDelay = (_frqcy / _maxVel);	// Recalculate max velocity
	
    // currDelay = (_frqcy / pow( pow(_initVel, 2) + (2 * _maxAccel), 0.5));	    // Delay for the first step [17]
    // _allDelays.push_back(static_cast<int>(currDelay * 10000));
    
    // int stepNumber = 1;
    
    // for(stepNumber; stepNumber < accelSteps; stepNumber += 1) {
    // 	currDelay = currDelay * (1 + (-1 * _multiplier * pow(currDelay, 2)));	// [20]
    	
    //     // Motor should not move faster than the maximum speed
	// 	if(currDelay <= _maxVelDelay) {
	// 		currDelay = _maxVelDelay;
	// 	}
    // 	_allDelays.push_back(static_cast<int>(currDelay * 10000));
    // }

    // for(stepNumber; stepNumber < velSteps; stepNumber += 1) {
    // 	_allDelays.push_back(static_cast<int>(currDelay * 10000));
    // }
    
    // for(stepNumber; stepNumber < STEPS; stepNumber += 1) {
    // 	currDelay = currDelay * (1 + (_multiplier * pow(currDelay, 2)));	// [20]
    // 	_allDelays.push_back(static_cast<int>(currDelay * 10000));
    // }   
}

/**
	Steps through the entire stepper routine for a given number of steps
	@param STEPS Number of steps to take
*/
void Stepper::relStep(const int STEPS) {
    if(STEPS != 0) {
        calculateParameters(STEPS);	// Calculate delays for every step

        bool isForward = true;
        
        if(STEPS < 0) {
            isForward = false;
        }
        
        for(float stepDelay : _allDelays) {
            pulse(isForward, stepDelay);
        }

        std::vector<double>().swap(_allDelays); // Remove all delays for this routine and force a reallocation
    }
}

/**
 * Moves the stepper at constant velocity
 * @param REVPS speed of the stepper in rev/s 
*/
void Stepper::velStep(int STEPS, double REVPS) {
	
	float velDelay = 1000000 / (REVPS * _maxSteps);
    bool isForward = true;
    
    if(STEPS < 0) {
        isForward = false;
    }
    
    STEPS = abs(STEPS);

    for(int i = 0; i < STEPS; i += 1) {
        pulse(isForward, velDelay);
    }
}

/**
 * Move the stepper by 1 step in the given direction with the given delay
*/
void Stepper::pulse(bool IS_CLOCKWISE, int PULSE_DELAY) {
    if(IS_CLOCKWISE) {
        digitalWrite(_directionPin, LOW);
        digitalWrite(_pulsePin, HIGH);
        delayMicroseconds(PULSE_DELAY);
        digitalWrite(_pulsePin, LOW);
        _currPosition += 1;
    } else {
        digitalWrite(_directionPin, HIGH);
        digitalWrite(_pulsePin, HIGH);
        delayMicroseconds(PULSE_DELAY);
        digitalWrite(_pulsePin, LOW);
        _currPosition -= 1;
    }
}

/**
 * Tells the stepper motor to go to DESIRED_POSITION from _currPosition, which is 0 on startup
 * @param DESIRED_POSITION Desired final position of the shaft
*/
void Stepper::absStep(const int DESIRED_POSITION) {
    const int NUM_STEPS = DESIRED_POSITION - _currPosition; // Number of steps to take to reach DESIRED_POSITION
    relStep(NUM_STEPS);
    _currPosition = DESIRED_POSITION;
}

/**
 * Sets max acceleration and converts from rev/s^2 to steps/s^2
 * @param ACCELERATION acceleration in rev/s^2
*/
void Stepper::setAcceleration(const double ACCELERATION) {
    _accel = ACCELERATION * _maxSteps;
}

/**
 * Sets max velocity and converts from rev/s to steps/s
 * @param MAX_VELOCITY velocity in rev/s
*/
void Stepper::setMaxVelocity(const double MAX_VELOCITY) {
    _maxVel = MAX_VELOCITY * _maxSteps;
}

void Stepper::setMaxSteps(const int MAX_STEPS) {
    _maxSteps = MAX_STEPS;
}

/**
 * Return current position of the motor
*/
int Stepper::getCurrentPosition() {
    return _currPosition;
}

/**
 * Return number of steps for 1 full rotation
*/
int Stepper::getMaxSteps() {
    return _maxSteps;
}

int Stepper::getMicroStepSize() {
    return _microStepSize;
}

/**
 * Set current position of the motor
*/
void Stepper::setCurrentPosition(int POSITION) {
    _currPosition = POSITION;
}

void Stepper::setThreshold(int STEPS) {
    _accelThreshold = STEPS;
}

void Stepper::setMinVel(double REVPS) {
    _minVelDelay = 1 / (REVPS * _maxSteps);
}

