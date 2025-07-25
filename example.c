#include <stdio.h>
#include <mpu6050_i2c_lib.h>
#include "pico/stdlib.h"
#include <math.h>

#define delta_ms 10
#define alpha 0.05 // for complimentary filter, big alpha gives faster reponse but more noise

int main(void) {
    stdio_init_all();

    // initialise to min FS ie +=250deg/s and +=2g
    mpu6050_initialise(4,5,0,0);
    mpu6050_reset();
    // get gyro offsets
    float roll_offset;
    float pitch_offset;
    mpu6050_get_gyro_offset(10000, &roll_offset, &pitch_offset);

    // get initial roll and pitch values based on accelerometer
    int16_t acceleration[3], gyro[3], temp;
    mpu6050_read_raw(acceleration, gyro, &temp);
    float roll = atan2(acceleration[1] , acceleration[2]) * 57.3;
    float pitch = atan2((- acceleration[0]) , sqrt(acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2])) * 57.3;

    while (true) {
        mpu6050_fusion_output(roll_offset, pitch_offset, alpha, delta_ms, &roll, &pitch);
        printf("Roll: %f, Pitch: %f\n", roll, pitch);
        sleep_ms(delta_ms);
    }
        
}