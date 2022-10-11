#include <Arduino.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

#ifndef IMU_9DOF_H
#define IMU_9DOF_H

class IMU_9DOF{

	public:
		IMU_9DOF();
		void read_lis3mdl();
		void read_lsm6dsox_accel();
		void read_lsm6dsox_gyro();
		void pretty_print_buffer();

		float buffer[9];

	private:

		Adafruit_LIS3MDL lis3mdl;
		Adafruit_LSM6DSOX lsm6dsox;
};

#endif