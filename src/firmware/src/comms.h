#include <Arduino.h>

#ifndef COMMS_H
#define COMMS_H

class COMMS{

	public:
		COMMS();
		void read_packet(byte*, int);
		void send_packet(byte*);
		void send_floats(String, float*, int);
};

#endif