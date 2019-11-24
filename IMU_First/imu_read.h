#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <math.h>
#include <stdint.h>

#define PI 3.141592654

#define DEV_ADD 0x68

#define X_ACC_ADD_H 0x3b
#define Y_ACC_ADD_H 0x3d
#define Z_ACC_ADD_H 0x3f
#define X_ACC_ADD_L 0x3c
#define Y_ACC_ADD_L 0x3e
#define Z_ACC_ADD_L 0x40

#define X_GYRO_ADD_H 0x43
#define Y_GYRO_ADD_H 0x45
#define Z_GYRO_ADD_H 0x47
#define X_GYRO_ADD_L 0x44
#define Y_GYRO_ADD_L 0x46
#define Z_GYRO_ADD_L 0x48

#define X_MAG_ADD_H 0x04
#define Y_MAG_ADD_H 0x06
#define Z_MAG_ADD_H 0x08
#define X_MAG_ADD_L 0x03
#define Y_MAG_ADD_L 0x05
#define Z_MAG_ADD_L 0x07

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

#define DIV_SMPLR 0x19

#define CONFIG_MPU 0x1A
#define MPU_POWER1 0x6b
#define MPU_POWER2 0x6c

#define INT_PIN_CONF 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A

#define MAG_CNTL1 0x0a
#define MAG_CNTL2 0x0b
#define MAG_ASTC 0x0c

#define MAG_SENSE_X 0x10
#define MAG_SENSE_Y 0x11
#define MAG_SENSE_Z 0x12

#define MAG_STATUS1 0x02
#define MAG_STATUS2 0x09

#define sampleFreq	100.0f		// sample frequency in Hz
#define betaDef	1.5f		// 2 * proportional gain

#define sonar_size 15

int i2c_file;
int uart_file;
float Ares = 2.0/32768.0;
float Gres = 250.0/32768.0;
float Mres = (10.0*4912.0)/32760.0;
float mag_Calib[3] = {0,0,0};
float mag_bias[3] = {0,0,0};

const float r = 100.0;
volatile float beta = betaDef;				// algorithm gain
volatile float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float roll,pitch, yaw;
volatile float roll1, pitch1, yaw1;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

void initialize_i2c(int address);

void write_i2c(uint8_t reg_add, uint8_t value);

void MadgwickAHRSupdateOpt(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

void initialize_mpu();

void initialize_mag(float *destination);

void read_data_mag(int16_t *destination);

void read_data_acc(int16_t *destination);

void read_data_gyro(int16_t *destination);

float invSqrtOpt(float x);

void MadgwickAHRSupdateIMUOpt(float gx, float gy, float gz, float ax, float ay, float az);

void toEulerAngle(float q0, float q1, float q2, float q3);

void toEulerianAngle(float q0,float q1,float q2, float q3);

void initialize_uart();

void set_serial_blocking(int if_block);

int set_serial_attr(int speed, int parity);

void extraction_uart(float *destination);
