#include "RobotAxis.hpp"
#include <iostream>

RobotAxis::RobotAxis(Stepper &AXIS_MOTOR, const float STEP_ANGLE, const float AXIS_LENGTH) : _axisMotor(AXIS_MOTOR) {
    _stepAngle = STEP_ANGLE;
    _axisLength = AXIS_LENGTH;
    _currentAngle = 0;
    _isPositive = true;
    _axisMotor.setMaxSteps(360 / STEP_ANGLE);
}

void RobotAxis::rotate(float DESIRED_ANGLE) {
    // return (_isPositive ? 1 : -1) * (static_cast<int>(DESIRED_ANGLE / _stepAngle));
	_axisMotor.relStep((_isPositive ? 1 : -1) * (static_cast<int>(DESIRED_ANGLE / _stepAngle)));
    updatePosition();
}

void RobotAxis::rotate(float DESIRED_ANGLE, float OMEGA) {
	_axisMotor.velStep((_isPositive ? 1 : -1) * (static_cast<int>(DESIRED_ANGLE / _stepAngle)), OMEGA);
    updatePosition();
}

void RobotAxis::updatePosition() {
    _currentAngle = (_isPositive ? 1.0f : -1.0f) * _axisMotor.getCurrentPosition() * _stepAngle;
}

void RobotAxis::goToAngle(float DESIRED_ANGLE) {
    rotate(DESIRED_ANGLE - _currentAngle);
}

void RobotAxis::goToAngle(float DESIRED_ANGLE, float OMEGA) {
    rotate(DESIRED_ANGLE - _currentAngle, OMEGA);
}

void RobotAxis::setDirection(bool IS_POSITIVE) {
    _isPositive = IS_POSITIVE;
}

void RobotAxis::setMaxVelocity(float MAX_VELOCITY) {
    _axisMotor.setMaxVelocity(MAX_VELOCITY);
}

void RobotAxis::setAcceleration(float ACCELERATION) {
    _axisMotor.setAcceleration(ACCELERATION);
}

float RobotAxis::getCurrentAngle() {    return _currentAngle;   }
float RobotAxis::getJointLength()  {    return _axisLength;     }
float RobotAxis::getStepAngle()    {    return _stepAngle;      }