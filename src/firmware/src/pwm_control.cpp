#include "pwm_control.h"


uint16_t bytes_to_pwm(byte byte1, byte byte2) {
	return map((uint16_t(byte2) << 8) | uint16_t(byte1), 0, pow(2, 16), 18000, 32000); 
}


PWM_Controller::PWM_Controller(int p1, int p2, int p3, int p4){
	pinMode(p1, OUTPUT);
	pinMode(p2, OUTPUT);
	pinMode(p3, OUTPUT);
	pinMode(p4, OUTPUT);
	analogWriteResolution(15);
	analogWriteFrequency(p1, 4577.64); // 4577.64
	analogWriteFrequency(p2, 4577.64);	
	analogWriteFrequency(p3, 4577.64);
	analogWriteFrequency(p4, 4577.64);
	pins[0] = p1;
	pins[1] = p2;
	pins[2] = p3;
	pins[3] = p4;
}

PWM_Controller::PWM_Controller(int p1, int p2, int p3, int p4, int resolution, int freq){
	pinMode(p1, OUTPUT);
	pinMode(p2, OUTPUT);
	pinMode(p3, OUTPUT);
	pinMode(p4, OUTPUT);
	analogWriteResolution(resolution);
	analogWriteFrequency(p1, freq);
	analogWriteFrequency(p2, freq);	
	analogWriteFrequency(p3, freq);
	analogWriteFrequency(p4, freq);
	pins[0] = p1;
	pins[1] = p2;
	pins[2] = p3;
	pins[3] = p4;
}


void PWM_Controller::pretty_print(){
	Serial.print("PWM Info: {");
	Serial.print(pins[0]); Serial.print(", "); Serial.print(bytes_to_pwm(data[0], data[1])); Serial.print("}, {");
	Serial.print(pins[1]); Serial.print(", "); Serial.print(bytes_to_pwm(data[2], data[3])); Serial.print("}, {");
	Serial.print(pins[2]); Serial.print(", "); Serial.print(bytes_to_pwm(data[4], data[5])); Serial.print("}, {");
	Serial.print(pins[3]); Serial.print(", "); Serial.print(bytes_to_pwm(data[6], data[7])); Serial.println("}");
}

void PWM_Controller::pretty_print_bytes(){
	Serial.print("PWM bytes: {"); Serial.print(data[0]); Serial.print("}, {");
	Serial.print(data[1]); Serial.print("}, {");
	Serial.print(data[2]); Serial.print("}, {");
	Serial.print(data[3]); Serial.print("}, {");
	Serial.print(data[4]); Serial.print("}, {");
	Serial.print(data[5]); Serial.print("}, {");
	Serial.print(data[6]); Serial.print("}, {");
	Serial.print(data[7]); Serial.println("}");
}


void PWM_Controller::updateIO(){
	analogWrite(pins[0], bytes_to_pwm(data[0], data[1]));
	analogWrite(pins[1], bytes_to_pwm(data[2], data[3]));
	analogWrite(pins[2], bytes_to_pwm(data[4], data[5]));
	analogWrite(pins[3], bytes_to_pwm(data[6], data[7]));
}


void PWM_Controller::disable(){
	for (int i = 0; i < 8; i++){
		data[i] = 0;
	}
	
	updateIO();
}