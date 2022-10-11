#include "imu_9dof.h"

IMU_9DOF::IMU_9DOF() {
	/*
		Set up the jawns and the jimmys, config is hardcoded atm
		but that can change to use a more global definition.
	*/

	Wire.setClock(1000000);
	

	// LSM6DSOX Setup
	lsm6dsox.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire, 0);

	/** The accelerometer data range 
	typedef enum accel_range {
		LSM6DS_ACCEL_RANGE_2_G,
		LSM6DS_ACCEL_RANGE_4_G,
		LSM6DS_ACCEL_RANGE_8_G,
		LSM6DS_ACCEL_RANGE_16_G
	} lsm6ds_accel_range_t; */

	lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);

	/** The gyro data range
	typedef enum gyro_range {
		LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
		LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
		LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
		LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
		LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
		ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
	} lsm6ds_gyro_range_t; */

	lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

	/** The accelerometer data rate
	typedef enum data_rate {
		LSM6DS_RATE_SHUTDOWN,
		LSM6DS_RATE_12_5_HZ,
		LSM6DS_RATE_26_HZ,
		LSM6DS_RATE_52_HZ,
		LSM6DS_RATE_104_HZ,
		LSM6DS_RATE_208_HZ,
		LSM6DS_RATE_416_HZ,
		LSM6DS_RATE_833_HZ,
		LSM6DS_RATE_1_66K_HZ,
		LSM6DS_RATE_3_33K_HZ,
		LSM6DS_RATE_6_66K_HZ,
	} lsm6ds_data_rate_t; */

	lsm6dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
	lsm6dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);

	/** The high pass filter bandwidth
	typedef enum hpf_range {
		LSM6DS_HPF_ODR_DIV_50 = 0,
		LSM6DS_HPF_ODR_DIV_100 = 1,
		LSM6DS_HPF_ODR_DIV_9 = 2,
		LSM6DS_HPF_ODR_DIV_400 = 3,
	} lsm6ds_hp_filter_t; */

	/*!
		@brief Sets the INT1 and INT2 pin activation mode
		@param active_low true to set the pins  as active high, false to set the
		mode to active low
		@param open_drain true to set the pin mode as open-drain, false to set the
		mode to push-pull
	*/
	lsm6dsox.configIntOutputs(false, true);

	/*!
		@brief Enables and disables the data ready interrupt on INT 1.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
		@param step_detect true to output the step detection interrupt (default off)
		@param wakeup true to output the wake up interrupt (default off)
	*/
	lsm6dsox.configInt1(false, false, false, false, false);

	/*!
		@brief Enables and disables the data ready interrupt on INT 2.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
	*/
	lsm6dsox.configInt2(false, false, false);

	// LIS3MDL Setup
	lis3mdl.begin_I2C(LIS3MDL_I2CADDR_DEFAULT, &Wire);

	/** The magnetometer ranges
	typedef enum {
		LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
		LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
		LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
		LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
	} lis3mdl_range_t; */

	lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);


	/** The magnetometer data rate, includes FAST_ODR bit
	typedef enum {
		LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
		LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
		LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
		LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
		LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
		LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
		LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
		LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
		LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
		LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
		LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
		LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
	} lis3mdl_dataRate_t; */

	lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);

	/** The magnetometer performance mode
	typedef enum {
		LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
		LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
		LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
		LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
	} lis3mdl_performancemode_t; */

	lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);

	/** The magnetometer operation mode
	typedef enum {
		LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
		LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
		LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
	} lis3mdl_operationmode_t; */

	lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
	
	lis3mdl.configInterrupt(false, false, false, // enable z axis
											true, // polarity
											false, // don't latch
											true); // enabled!
}

void IMU_9DOF::read_lsm6dsox_accel(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	if (lsm6dsox.accelerationAvailable()){
		lsm6dsox.readAcceleration(buffer[0], buffer[1], buffer[2]);
	}

	// Serial.print("LSM6DSOX accel read time: "); Serial.println(micros() - read_start);
}

void IMU_9DOF::read_lsm6dsox_gyro(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	if (lsm6dsox.gyroscopeAvailable()){
		lsm6dsox.readGyroscope(buffer[3], buffer[4], buffer[5]);
	}

	// Serial.print("LSM6DSOX gyro read time: "); Serial.println(micros() - read_start);
}

void IMU_9DOF::read_lis3mdl(){
	/*
		Get the jawns from the jimmys
	*/
	// unsigned long read_start = micros();
	// lis3mdl.read();
	if (lis3mdl.magneticFieldAvailable()){
		lis3mdl.readMagneticField(buffer[6], buffer[7], buffer[8]);
	}

	// Serial.print("LIS3MDL mag read time: "); Serial.println(micros() - read_start);
}

void IMU_9DOF::pretty_print_buffer(){
	/*
		In case you want to show the jawns coming from the jimmys.
	*/
	Serial.println("=========== IMU_9DOF Buffer ===========");
	
	Serial.print("Acceleration\t");
	Serial.print(buffer[0]); Serial.print("\t"); 
	Serial.print(buffer[1]); Serial.print("\t"); 
	Serial.println(buffer[2]);

	Serial.print("Gyroscope\t");
	Serial.print(buffer[3]); Serial.print("\t"); 
	Serial.print(buffer[4]); Serial.print("\t"); 
	Serial.println(buffer[5]);

	Serial.print("Magnetometer\t");
	Serial.print(buffer[6]); Serial.print("\t"); 
	Serial.print(buffer[7]); Serial.print("\t"); 
	Serial.println(buffer[8]);
	
	Serial.println("=======================================");
	
}