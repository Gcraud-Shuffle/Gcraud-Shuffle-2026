/*
 * File:   GYRO.hpp
 * Author: taku_256
 *
 * Created on March 1, 2026, 9:30 AM
 */

#ifndef GYRO_HPP_
#define GYRO_HPP_

#include <stdint.h>

extern double GYRO_kp, GYRO_ki, GYRO_kd;
extern double GYRO_I;
extern uint16_t GYRO_precnt;

extern int GYRO_duty, GYRO_E, GYRO_preE, GYRO_AngleOffset;
extern double dt;
#endif /*GYRO_HPP_*/

/*******************************************************************************
 End of File
*/
