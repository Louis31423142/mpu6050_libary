#include <stdint.h>

#ifndef MPU6050_I2C_LIB
#define MPU6050_I2C_LIB

void mpu6050_reset(void);
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
int mpu6050_initialise(int SDA_pin, int SCL_pin, int GYRO_FS, int ACCEL_FS);

#endif