/*
 * File:   algo.cpp
 * Author: takum
 *
 * Created on March 12, 2026, 10:49 AM
 */

#include "algo.hpp"

#include <math.h>

#include "common.hpp"

double get_mid(double i, double j) {
    double theta = (i + j) / 2.0 + 16.0 * (16.0 < dabs(j - i));
    theta -= 32.0 * (theta >= 32.0);
    return theta;
}

circular get_param(int i, int j) {
    circular ans;
    ans.theta = get_mid(i, j)*11.25;
    ans.r = dabs(cos((double)(j - i) * pi / 32.0));
    return ans;
}

bool line_data[32];
segment seg[16];

bool debug_pattern[32] = {1,1,1,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,                          
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0};

cartesian2 calc_line() {
    cartesian2 ans = {0.0,0.0}, sum = {0.0, 0.0};
    int number_of_points = 0;

    for (int i = 0; i < 32; i++) {
        line_data[i] = get_Angel(i);
//        line_data[i] = debug_pattern[i];
    }
    
    int n = 0;
    for (int i = 0; i < 32; i++) {
        if (!line_data[i]) continue;

        seg[n].start = i;

        while (i + 1 < 32 && line_data[i + 1]) i++;

        seg[n].end = i;
        n++;
    }

    if (n > 1 && line_data[0] && line_data[31]) {
        seg[0].start = seg[n - 1].start;
        n--;
    }
    
    if (n == 1) {
        int start = seg[0].start;
        int end = seg[0].end;
        if(start == end){
            circular param = get_param(start, end);
            ans.x = param.r*cos(deg(param.theta));
            ans.y = param.r*sin(deg(param.theta));
            number_of_points++;
            return ans;
        }
        for (int i = start; (start < end) ? (i <= end) : (start <= (i & 0x1f) || (i & 0x1f) < end); i++) {
            for (int j = (i + 1) & 0x1f; (start <= end) ? (j <= end) : (start <= (j & 0x1f) || (j & 0x1f) <= end); j++) {
                circular param = get_param(i, j&0x1f);
                sum.x += param.r * cos(deg(param.theta));
                sum.y += param.r * sin(deg(param.theta));
                number_of_points++;
            }
        }
    } else {
        for (int i = 0; i < n; i++) {
            int start = seg[i].start;
            int end = seg[i].end;
            for (int j = start; (start <= end) ? (j <= end) : (start <= (j&0x1f) || (j&0x1f) <= end); j++) {
                for (int k = i + 1; k < n; k++) {
                    for (int l = seg[k].start; (seg[k].start <= seg[k].end) ? (l <= seg[k].end) : (seg[k].start <= (l&0x1f) || (l&0x1f) <= seg[k].end); l++) {
                        circular param = get_param((j&0x1f), (l&0x1f));
                        sum.x += param.r * cos(deg(param.theta));
                        sum.y += param.r * sin(deg(param.theta));
                        number_of_points++;
                    }
                }
            }
        }
    }
    ans.x = ((bool)number_of_points)?sum.x / number_of_points:0;
    ans.y = ((bool)number_of_points)?sum.y / number_of_points:0;
    return ans;
}
