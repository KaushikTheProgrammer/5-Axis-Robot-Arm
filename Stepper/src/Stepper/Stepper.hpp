/**
	Stepper.hpp
	Purpose: Library to be used with the TB6600 Stepper Driver and a Raspberry Pi

	@author Kaushik Prakash
	@version 1.0 03/01/19
*/

#include <vector>

class Stepper {

	public:
        Stepper();
		Stepper(const int DIRECTION_PIN, const int PULSE_PIN, const int MICRO_STEP_SIZE);
		void pulse(bool direction, int pulseDelay);             // Take 1 step
		void relStep(int STEPS);                          // Step a certain number of times
		void velStep(const int STEPS, double REVPS);                          // Step a certain number of times
		void absStep(const int DESIRED_POSITION);               // Step to a certain position
        void setAcceleration(const double MAX_ACCELERATION);  // Set max acceleration rev/s^2
		void setMaxVelocity(const double MAX_VELOCITY);          // Set max velocity in rev/s
		void setCurrentPosition(int POSITION);
        void setMaxSteps(const int MAX_STEPS);
		
		int getCurrentPosition();
		int getMaxSteps();
        int getMicroStepSize();
        void setThreshold(int STEPS);
        void setMinVel(double REVPS);

	private:
		std::vector<double> _allDelays;  							// Contains time intervals for every step in the routine
        void calculateParameters(int STEPS);  // Calculate time intervals for each step
		double _initVel;    				// Starting velocity in rev/s
		double _maxVel;        	// Max velocity in rev/s
		double _accel;	// Max acceleration in rev/s^2
		double _currDelay;   				// Delay of the next step to be taken
		double _minVelDelay;
		int _maxSteps; 					// Number of steps in a revolution after microstepping
		int _currPosition;				// Current Position of Stepper Shaft
		int _microStepSize; 			// Microstepping size in microsteps/step: 1/2, 1/4 would be 2, 4 etc.
		int _directionPin;  				// Direction pin on the RPI using WiringPi pin numbers
		int _pulsePin;         				// Signal pin on the RPI using WiringPi pin numbers
        int _accelThreshold;
};
