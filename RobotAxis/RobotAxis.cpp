#include "RobotAxis.hpp"
#include <iostream>

RobotAxis::RobotAxis(Stepper &AXIS_MOTOR, const double STEP_ANGLE, const double AXIS_LENGTH) : _axisMotor(AXIS_MOTOR) {
    _maxAngle = 90;
    _minAngle = -90;
    _stepAngle = STEP_ANGLE;
    _axisLength = AXIS_LENGTH;
    _currentAngle = 0;
    _isPositive = true;
    _axisMotor.setMaxSteps(360 / STEP_ANGLE);
}

void RobotAxis::rotate(double DESIRED_ANGLE, double OMEGA) {	
    // std::cout << "Max Angle: " << _maxAngle << std::endl;
    // std::cout << "Min Angle: " << _minAngle << std::endl;
    int steps = (_isPositive ? 1 : -1) * (static_cast<int>(DESIRED_ANGLE / _stepAngle));
    // std::cout << steps << std::endl;
    
    if(OMEGA != 0) { 
        _axisMotor.velStep(steps, OMEGA);
    } else { 
        _axisMotor.relStep(steps); 
    }
    
    updatePosition();
}

void RobotAxis::goToAngle(double DESIRED_ANGLE, double OMEGA) {
    // if      (DESIRED_ANGLE > _maxAngle) { DESIRED_ANGLE = _maxAngle; }
    // else if (DESIRED_ANGLE < _minAngle) { DESIRED_ANGLE = _minAngle; }
    rotate(DESIRED_ANGLE - _currentAngle, OMEGA);
}

void RobotAxis::updatePosition() {
    _currentAngle = (_isPositive ? 1 : -1) * _axisMotor.getCurrentPosition() * _stepAngle;
}



// void RobotAxis::goToAngle(double DESIRED_ANGLE, double OMEGA) {
//     if(DESIRED_ANGLE > _maxAngle) { DESIRED_ANGLE = _maxAngle; }
//     else if(DESIRED_ANGLE < _minAngle) { DESIRED_ANGLE = _minAngle; }
//     rotate(DESIRED_ANGLE - _currentAngle, OMEGA);
// }

void RobotAxis::setDirection(bool IS_POSITIVE) {
    _isPositive = IS_POSITIVE;
}

void RobotAxis::setMaxVelocity(double MAX_VELOCITY) {
    _axisMotor.setMaxVelocity(MAX_VELOCITY);
}

void RobotAxis::setAcceleration(double ACCELERATION) {
    _axisMotor.setAcceleration(ACCELERATION);
}

void RobotAxis::setMaxAngle(int MAX_ANGLE) {
    _maxAngle = MAX_ANGLE;
}

void RobotAxis::setMinAngle(int MIN_ANGLE) {
    _minAngle = MIN_ANGLE;
}

double RobotAxis::getCurrentAngle() {    return _currentAngle;   }
double RobotAxis::getJointLength()  {    return _axisLength;     }
double RobotAxis::getStepAngle()    {    return _stepAngle;      }