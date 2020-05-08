/**
	RobotAxis.hpp
	Purpose: To easily represent and manipulate various axes of a robot arm

	@author Kaushik Prakash
	@version 1.0 04/30/20
*/

#include "Stepper/Stepper.hpp"
#include <wiringPi.h>


class RobotAxis {
    public:
        RobotAxis(Stepper& AXIS_MOTOR, const float STEP_ANGLE, const float AXIS_LENGTH);
        void rotate(float DESIRED_ANGLE);               // Rotate DESIRED_ANGLE degrees
        void rotate(float DESIRED_ANGLE, float OMEGA);  // Rotate at constant angular velocity to DESIRED_ANGLE
        void goToAngle(float DESIRED_ANGLE);
        void goToAngle(float DESIRED_ANGLE, float OMEGA);
        void setMaxVelocity(float MAX_VELOCITY);
        void setAcceleration(float ACCELERATION);
        void setDirection(bool IS_POSITIVE);
        float getCurrentAngle();
        float getJointLength();                    // Returns the length of the joint
        float getStepAngle();                      // Returns the step angle for joint


	private:
        void updatePosition();
        Stepper &_axisMotor;                         // Motor responsible for axis
        float _stepAngle;                           // Degrees per step including gearing
        float _axisLength;                          // Center distance length of physical axis
        float _currentAngle;                        // Axis angle with respect to previous axis vertical    
        bool _isPositive;
};
