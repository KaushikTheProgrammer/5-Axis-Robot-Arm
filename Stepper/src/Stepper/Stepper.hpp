/**
	Stepper.hpp
	Purpose: Library to be used with the TB6600 Stepper Driver and a Raspberry Pi

	@author Kaushik Prakash
	@version 1.7 07/03/20
*/

#include <vector>

class Stepper {
	public:
        Stepper();
		Stepper(const int DIRECTION_PIN, const int PULSE_PIN, 
                const int MICRO_STEP_SIZE);
		void pulse(bool direction, int pulseDelay);             // Take 1 step either CW or CCW
		void relStep(int STEPS);                                // Pulse STEPS steps with appropriate acceleration profile
		void velStep(const int STEPS, double REVPS);            // Pulse STEPS steps at REVPS
		void absStep(const int DESIRED_POSITION);               // Step to a certain position using relStep
        void setAcceleration(const double MAX_ACCELERATION);    // Set max acceleration rev/s^2
		void setMaxVelocity(const double MAX_VELOCITY);         // Set max velocity in rev/s
		void setCurrentPosition(int POSITION);                  // Sets current position
        void setMaxSteps(const int MAX_STEPS);                  // Set maximum motor steps
		void setAccelThreshold(int STEPS);                      // Set minimum step count for acceleration
        void setMinVelocity(double REVPS);                      // Set min velocity in rev/s
		int getCurrentPosition();                               // Return current motor position
		int getMaxSteps();                                      // Return max motor steps
        int getMicroStepSize();                                 // Return micro step size
        void calculateParameters(int STEPS);                    // Calculate time intervals for each step

	private:
		std::vector<double> _allDelays;  						// Contains time intervals for every step in the routine
        // void calculateParameters(int STEPS);                    // Calculate time intervals for each step
		double _maxVel;        	                                // Max velocity in rev/s
		double _accel;	                                        // Max acceleration in rev/s^2
		double _minVel;                                         // Min Velocity in rev/s
		int _maxSteps; 					                        // Max possible motor steps
		int _currPosition;				                        // Current motor position
		int _microStepSize; 			                        // Microstepping size in microsteps/step: 1/2, 1/4 would be 2, 4 etc.
		int _directionPin;  				                    // Direction pin on the RPI using WiringPi pin numbers
		int _pulsePin;         				                    // Signal pin on the RPI using WiringPi pin numbers
        int _accelThreshold;                                    // Minimum number of steps required to accelerate
};
