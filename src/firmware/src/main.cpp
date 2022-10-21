#include <Arduino.h>

#include "comms.h"
#include "imu_9dof.h"

unsigned long top_time;
unsigned long serial_time;
unsigned long cycle_rate = 1000;
unsigned long serial_rate = 1;

COMMS comms;
IMU_9DOF imu;

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

	lis3mdl_tmr.begin(read_lis3mdl, 3000);
	delayMicroseconds(854);
	lsm6dsox_gyro_tmr.begin(read_lsm6dsox_gyro, 3000);
	delayMicroseconds(855);
	lsm6dsox_accel_tmr.begin(read_lsm6dsox_accel, 3000);
}


void loop(){
	while (!Serial);

	top_time = micros();

	blink();

	if (millis() - serial_time > serial_rate) {
		// Stop the interrupts and print the jawns
		noInterrupts();
		comms.send_floats("II", imu.data, 9);
		interrupts();
		serial_time = millis();
	}
	
	if (micros() - top_time > cycle_rate) {
		Serial.print("Overtime ");
		Serial.println(micros() - top_time);
	}
	while (micros() - top_time < cycle_rate){}
}