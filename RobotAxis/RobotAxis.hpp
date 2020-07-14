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
        RobotAxis(Stepper& AXIS_MOTOR, const double STEP_ANGLE, const double AXIS_LENGTH);
        void rotate(double DESIRED_ANGLE);               // Rotate DESIRED_ANGLE degrees
        void rotate(double DESIRED_ANGLE, double OMEGA);  // Rotate at constant angular velocity to DESIRED_ANGLE
        void goToAngle(double DESIRED_ANGLE);
        void goToAngle(double DESIRED_ANGLE, double OMEGA);
        void setMaxVelocity(double MAX_VELOCITY);
        void setAcceleration(double ACCELERATION);
        void setDirection(bool IS_POSITIVE);
        void setMinVelocity(double MIN_VELOCITY);
        double getCurrentAngle();
        double getJointLength();                    // Returns the length of the joint
        double getStepAngle();                      // Returns the step angle for joint


	private:
        void updatePosition();
        Stepper &_axisMotor;                         // Motor responsible for axis
        double _stepAngle;                           // Degrees per step including gearing
        double _axisLength;                          // Center distance length of physical axis
        double _currentAngle;                        // Axis angle with respect to previous axis vertical    
        bool _isPositive;
};
