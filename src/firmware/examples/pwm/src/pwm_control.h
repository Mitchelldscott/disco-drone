#include <Arduino.h>

#ifndef PWM_CONTROL
#define PWM_CONTROL


class PWM_Controller{

	public:
		PWM_Controller(int, int, int, int);
		void pretty_print();
		void updateIO();
		void disable();

		byte data[8];

	private:
		int pins[4];

};

#endif