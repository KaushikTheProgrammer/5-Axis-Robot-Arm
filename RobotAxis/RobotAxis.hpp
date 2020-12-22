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
        void rotate(double DESIRED_ANGLE, double OMEGA = 0);  // Rotate at constant angular velocity to DESIRED_ANGLE
        void goToAngle(double DESIRED_ANGLE, double OMEGA = 0);
        void setMaxVelocity(double MAX_VELOCITY);
        void setAcceleration(double ACCELERATION);
        void setDirection(bool IS_POSITIVE);
        void setMinVelocity(double MIN_VELOCITY);
        void setMaxAngle(int MAX_ANGLE);
        void setMinAngle(int MIN_ANGLE);
        double getCurrentAngle();
        double getJointLength();                    // Returns the length of the joint
        double getStepAngle();                      // Returns the step angle for joint


	private:
        Stepper &_axisMotor;                         // Motor responsible for axis
        void updatePosition();
        int _maxAngle;
        int _minAngle;
        double _stepAngle;                           // Degrees per step including gearing
        double _axisLength;                          // Center distance length of physical axis
        double _currentAngle;                        // Axis angle with respect to previous axis vertical    
        bool _isPositive;
};
