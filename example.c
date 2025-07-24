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

    // initialise variables
    int16_t acceleration[3], gyro[3], temp;
    float roll_rate;
    float pitch_rate;
    float accel_roll;
    float accel_pitch;

    // get gyro offsets
    float roll_offset = 0;
    float pitch_offset = 0;
    int NUM = 10000;

    for (int i = 0; i < NUM; i++) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        roll_offset += (gyro[0] / 32768.0 * 250.0)/NUM;
        pitch_offset += (gyro[1] / 32768.0 * 250.0)/NUM;
    }

    printf("roll offset %f\n", roll_offset);
    printf("pitch offset %f\n", pitch_offset);

    // set intial roll and pitch values based on accelerometer
    mpu6050_read_raw(acceleration, gyro, &temp);
    float roll = acceleration[0];
    float pitch = acceleration[1];


    int count = 0;
    while (true) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        // calculate pitch and roll from accelerometer o/p
        // this calculation is independent of full scale
        accel_roll = atan2(acceleration[1] , acceleration[2]) * 57.3;
        accel_pitch = atan2((- acceleration[0]) , sqrt(acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2])) * 57.3;

        roll_rate = gyro[0] / 32768.0 * 250.0 - roll_offset;
        pitch_rate = gyro[1] / 32768.0 * 250.0 - pitch_offset;

        // calculate pitch and roll using sensor fusion (complimentary filter)
        roll = (1 - alpha) * (roll + roll_rate * delta_ms/1000) + alpha * accel_roll;
        pitch = (1 - alpha) * (pitch + pitch_rate * delta_ms/1000) + alpha * accel_pitch;
        
        if (count == 100) { // print every second 
            printf("ROLL: %f , PITCH: %f \n", roll, pitch);
            count = 0;
        }

        sleep_ms(delta_ms);
        count++;
    }
        
}