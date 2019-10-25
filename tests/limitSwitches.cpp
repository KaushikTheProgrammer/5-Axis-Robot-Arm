#include <iostream>
#include <wiringPi.h>

const int baseSwitch = 24;
const int arm1Switch = 25;
const int arm2Switch = 27;
const int arm3Switch = 28;

int main() {
	wiringPiSetup();
	
	pinMode(baseSwitch, INPUT);
	pinMode(arm1Switch, INPUT);
	pinMode(arm2Switch, INPUT);
	pinMode(arm3Switch, INPUT);
	
	while(true) {
		std::cout << digitalRead(baseSwitch) << digitalRead(arm1Switch) 
			<< digitalRead(arm2Switch) << digitalRead(arm3Switch) 
			<< std::endl;
	}
	
	

	return 0;
}
