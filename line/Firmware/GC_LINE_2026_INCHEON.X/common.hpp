/*
 * File:   common.hpp
 * Author: taku_256
 *
 * Created on September 11, 2024, 9:30 AM
 */

#ifndef COMMON_HPP
#define COMMON_HPP

#define CCLK (120000000L) // system clock
#define PBCLK (CCLK / 2)  // peripheral bus clock
#define SAMPLE_RATE 5000
#define CCLK_US (PBCLK / 1000 / 1000) // used for micro second delay
#define CCLK_MS (PBCLK / 1000)        // used for milli second delay

#define __delay_us(d)                                       \
    do                                                      \
    {                                                       \
        unsigned int delayCount;                            \
        unsigned int startTime;                             \
        startTime = _CP0_GET_COUNT();                       \
        delayCount = d * CCLK_US;                           \
        while ((_CP0_GET_COUNT() - startTime) < delayCount) \
            ;                                               \
    } while (0)

#define __delay_ms(d)                                       \
    do                                                      \
    {                                                       \
        unsigned int delayCount;                            \
        unsigned int startTime;                             \
        startTime = _CP0_GET_COUNT();                       \
        delayCount = (d) * CCLK_MS;                         \
        while ((_CP0_GET_COUNT() - startTime) < delayCount) \
            ;                                               \
    } while (0)

#define ull unsigned long long int
//
#define pi 3.14159265358979323846264338327950288
#define rd (180.0 / pi) // radian to degree
#define deg(angle) (angle * pi / 180)
#define rad(angle) (angle * 180 / pi)

inline double wrap2pi(double x) { return (((x) > pi) ? wrap2pi((x)-2.0 * pi) : (((x) < -pi) ? wrap2pi((x) + 2.0 * pi) : (x))); }

#define MSB16(B) (B >> 8)
#define LSB16(B) (B & 0x00ff)

#define same_clamp(x, y) ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x))) // max(-y,min(x,y))
#define sign(j) (((j) > 0) - ((j) < 0))
#define dabs(j) ((j) > 0 ? (j) : -(j))
#define add32(add) ((uint32_t *)add)

struct bits
{
    unsigned char b1 : 1;
    unsigned char b2 : 1;
    unsigned char b3 : 1;
    unsigned char b4 : 1;
    unsigned char b5 : 1;
    unsigned char b6 : 1;
    unsigned char b7 : 1;
    unsigned char b8 : 1;
};

typedef union
{
    uint8_t chr;
    struct bits b;
} iro;

#define REF 3.3f
#define DACOUT(rec) (rec * 4095 / REF)

#endif /* COMMON_HPP */
