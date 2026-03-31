/*
 * File:   GYRO.cpp
 * Author: taku_256
 *
 * Created on March 1, 2026, 9:30 AM
 */

#include "robot.hpp"

double GYRO_kp = 0.8, GYRO_ki = 0.0, GYRO_kd = 0.2;
double GYRO_I = 0.0;
uint16_t GYRO_precnt = 0;

int GYRO_duty = 0, GYRO_E = 0, GYRO_preE = 0, GYRO_AngleOffset = 0;
double dt = 0;

void BNO_init() {
  unsigned char device_address = 0x28;
  uint8_t bno_mode_senddata[] = {0x3d, 0x08};
  HAL_I2C_Master_Transmit(&hi2c1, device_address << 1, bno_mode_senddata, 2,
                          100);
}
