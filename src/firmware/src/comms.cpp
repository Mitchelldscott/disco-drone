#include "comms.h"


COMMS::COMMS(){

}

void COMMS::read_packet(byte* buffer, int nbytes){
	if (Serial.available() > 0) {
		if (Serial.read() == 85) {
			if (Serial.read() == 85) {
				if (Serial.read() == 58){
					for (int i = 0; i < nbytes; i++) {
						buffer[i] = byte(Serial.read());
					}
				}
			}
		}
		Serial.clear();
	}
}


void COMMS::send_packet(byte* buffer){
	Serial.write(*buffer);
}

void COMMS::send_floats(String title, float* data, int len){
	Serial.print(title);
	for (int i = 0; i < len; i++){
		Serial.write(',');		
		Serial.print(String(data[i]));
	}
	Serial.write('\n');
}