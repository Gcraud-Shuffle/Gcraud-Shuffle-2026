/*
 * File:   algo.hpp
 * Author: takum
 *
 * Created on March 12, 2026, 10:49 AM
 */

#ifndef ALGO_HPP
#define ALGO_HPP

#include "line_sensor.hpp"

typedef struct {
    int start;
    int end;
} segment;

typedef struct {
    double r;
    double theta;
} circular;

typedef struct {
    double x;
    double y;
} cartesian2;

cartesian2 calc_line();

extern bool line_data[32];
extern segment seg[16];

#endif /* ALGO_HPP */
