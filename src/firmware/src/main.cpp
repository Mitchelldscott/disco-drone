#include <Arduino.h>
#include "imu_9dof.h"

unsigned long top_time;
unsigned long serial_time;
unsigned long cycle_rate = 250;
unsigned long serial_rate = 1;

IMU_9DOF imu;

void read_lis3mdl(){
	imu.read_lis3mdl();
}

void read_lsm6dsox_gyro(){
	imu.read_lsm6dsox_gyro();
}

void read_lsm6dsox_accel(){
	imu.read_lsm6dsox_accel();
}

IntervalTimer lis3mdl_tmr;
IntervalTimer lsm6dsox_gyro_tmr;
IntervalTimer lsm6dsox_accel_tmr;


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

	lis3mdl_tmr.begin(read_lis3mdl, 10000);
	lsm6dsox_gyro_tmr.begin(read_lsm6dsox_gyro, 4000);
	delayMicroseconds(2000);
	lsm6dsox_accel_tmr.begin(read_lsm6dsox_accel, 4000);
}


void loop(){
	while (!Serial);

	top_time = micros();

	blink();

	if (millis() - serial_time > serial_rate) {
		// Stop the interrupts and print the jawns
		noInterrupts();
		imu.pretty_print_buffer();
		interrupts();
		serial_time = millis();
	}
	
	if (micros() - top_time > cycle_rate) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}
	while (micros() - top_time < cycle_rate){}
}