#ifndef TEST_H
#define TEST_H

#include "main.h"

// board
void test_print(void);
void test_led(void);

// adc
void test_adc(void);

// imu
void test_imu(void);

// can
#define TEST_CAN_TX_ID 0x123      // heartbeat: counter + HAL tick
#define TEST_CAN_ADC_ID 0x100     // MCP3561R CH0 code + mV
#define TEST_CAN_IMU_ACC_ID 0x101 // BMI323 raw accel X/Y/Z
#define TEST_CAN_IMU_GYR_ID 0x102 // BMI323 raw gyro X/Y/Z + temp

void test_can(void);

#endif
