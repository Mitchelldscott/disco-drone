#include "pwm_control.h"


uint16_t bytes_to_uint16(byte byte1, byte byte2) {
	return (uint16_t(byte2) << 8) | uint16_t(byte1); 
}


PWM_Controller::PWM_Controller(int p1, int p2, int p3, int p4){
	pinMode(p1, OUTPUT);
	pinMode(p2, OUTPUT);
	pinMode(p3, OUTPUT);
	pinMode(p4, OUTPUT);
	analogWriteResolution(12);
	analogWriteFrequency(p1, 36621.09);
	analogWriteFrequency(p2, 36621.09);	
	analogWriteFrequency(p3, 36621.09);
	analogWriteFrequency(p4, 36621.09);
	pins[0] = p1;
	pins[1] = p2;
	pins[2] = p3;
	pins[3] = p4;
}


void PWM_Controller::pretty_print(){
	Serial.print("PWM Info: {");
	Serial.print(pins[0]); Serial.print(", "); Serial.print(bytes_to_uint16(data[0], data[1])); Serial.print("}, {");
	Serial.print(pins[1]); Serial.print(", "); Serial.print(bytes_to_uint16(data[2], data[3])); Serial.print("}, {");
	Serial.print(pins[2]); Serial.print(", "); Serial.print(bytes_to_uint16(data[4], data[5])); Serial.print("}, {");
	Serial.print(pins[3]); Serial.print(", "); Serial.print(bytes_to_uint16(data[6], data[7])); Serial.println("}");
}


void PWM_Controller::updateIO(){
	analogWrite(pins[0], bytes_to_uint16(data[0], data[1]));
	analogWrite(pins[1], bytes_to_uint16(data[2], data[3]));
	analogWrite(pins[2], bytes_to_uint16(data[4], data[5]));
	analogWrite(pins[3], bytes_to_uint16(data[6], data[7]));
}


void PWM_Controller::disable(){
	for (int i = 0; i < 8; i++){
		data[i] = 0;
	}
}