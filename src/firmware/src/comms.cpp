#include "comms.h"

COMMS::COMMS(){

}


// void COMMS::read_packet(byte* buffer){
// 	if (Serial.available()) {
// 		Serial.read();
// 	}
// }


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