#include <Arduino.h>

#include "comms.h"
#include "imu_9dof.h"
#include "pwm_control.h"

#define MOTOR_1 12
#define MOTOR_2 11
#define MOTOR_3 10
#define MOTOR_4 9

unsigned long top_time;
unsigned long serial_time;
unsigned long cycle_rate = 3000;
unsigned long serial_rate = 100;

COMMS comms;
IMU_9DOF imu;
PWM_Controller pwm(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 12, 500);

void set_pwm_signal() {
	pwm.updateIO();
}

void read_serial(){
	// Serial.println("read serial call");
	comms.read_packet(pwm.data, 8);
	// Serial.println("read serial return");
}

void read_lis3mdl(){
	// Serial.println("read mag call");
	imu.read_lis3mdl();
	// Serial.println("read mag return");
}

void read_lsm6dsox_gyro(){
	// Serial.println("read gyro call");
	imu.read_lsm6dsox_gyro();
	// Serial.println("read gyro return");
}

void read_lsm6dsox_accel(){
	// Serial.println("read accel call");
	imu.read_lsm6dsox_accel();	
	// Serial.println("read accel return");
}

void blink(){
	static bool status = false;
	static unsigned long t = millis();

	if (millis() - t > 250){
		status = !status;
		t = millis();
	}

	digitalWrite(LED_BUILTIN, status);
}

void setup(){
	serial_time = millis();

	Serial.begin(1000000);
	while (!Serial);

	// set built in LED pin to output mode
	pinMode(LED_BUILTIN, OUTPUT);
	// pinMode(24, OUTPUT);
	// digitalWrite(25, 1);
	set_pwm_signal();
}


void loop(){
	if (!Serial){
		pwm.disable();
		while(!Serial);
	}

	top_time = micros();

	blink();

	read_lis3mdl();
	read_lsm6dsox_gyro();
	read_lsm6dsox_accel();

	set_pwm_signal();

	if (millis() - serial_time > serial_rate) {
		comms.send_floats("II", imu.data, 9);
		read_serial();
		pwm.pretty_print();
		serial_time = millis();
	}
	
	if (micros() - top_time > cycle_rate) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}

	while (micros() - top_time < cycle_rate){}
}