#include "RobotAxis.hpp"
#include <iostream>

RobotAxis::RobotAxis(Stepper &AXIS_MOTOR, const float STEP_ANGLE, const float AXIS_LENGTH) : _axisMotor(AXIS_MOTOR) {
    _stepAngle = STEP_ANGLE;
    _axisLength = AXIS_LENGTH;
    _currentAngle = 0;
    _axisMotor.setMaxSteps(360 / STEP_ANGLE);
}

void RobotAxis::rotate(float DESIRED_ANGLE) {
	_axisMotor.relStep((int) DESIRED_ANGLE / _stepAngle);
    _currentAngle = _axisMotor.getCurrentPosition() * _stepAngle;
}

void RobotAxis::rotate(float DESIRED_ANGLE, float OMEGA) {
    std::cout << (int) (DESIRED_ANGLE / _stepAngle) << std::endl;
	_axisMotor.velStep((int) DESIRED_ANGLE / _stepAngle, OMEGA);
    _currentAngle = _axisMotor.getCurrentPosition() * _stepAngle;
}

void RobotAxis::goToAngle(float DESIRED_ANGLE) {
    _axisMotor.relStep((DESIRED_ANGLE - _currentAngle) / _stepAngle);
    _currentAngle = _axisMotor.getCurrentPosition() * _stepAngle;
}

void RobotAxis::goToAngle(float DESIRED_ANGLE, float OMEGA) {
    _axisMotor.velStep((DESIRED_ANGLE - _currentAngle) / _stepAngle, OMEGA);
    _currentAngle = _axisMotor.getCurrentPosition() * _stepAngle;
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