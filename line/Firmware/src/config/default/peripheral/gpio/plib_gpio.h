/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h UUUUUUUUU

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for L2 pin ***/
#define L2_Set()               (LATGSET = (1U<<15))
#define L2_Clear()             (LATGCLR = (1U<<15))
#define L2_Toggle()            (LATGINV= (1U<<15))
#define L2_OutputEnable()      (TRISGCLR = (1U<<15))
#define L2_InputEnable()       (TRISGSET = (1U<<15))
#define L2_Get()               ((PORTG >> 15) & 0x1U)
#define L2_PIN                  GPIO_PIN_RG15

/*** Macros for L1 pin ***/
#define L1_Set()               (LATASET = (1U<<7))
#define L1_Clear()             (LATACLR = (1U<<7))
#define L1_Toggle()            (LATAINV= (1U<<7))
#define L1_OutputEnable()      (TRISACLR = (1U<<7))
#define L1_InputEnable()       (TRISASET = (1U<<7))
#define L1_Get()               ((PORTA >> 7) & 0x1U)
#define L1_PIN                  GPIO_PIN_RA7

/*** Macros for S21 pin ***/
#define S21_Set()               (LATBSET = (1U<<14))
#define S21_Clear()             (LATBCLR = (1U<<14))
#define S21_Toggle()            (LATBINV= (1U<<14))
#define S21_OutputEnable()      (TRISBCLR = (1U<<14))
#define S21_InputEnable()       (TRISBSET = (1U<<14))
#define S21_Get()               ((PORTB >> 14) & 0x1U)
#define S21_PIN                  GPIO_PIN_RB14

/*** Macros for S22 pin ***/
#define S22_Set()               (LATBSET = (1U<<15))
#define S22_Clear()             (LATBCLR = (1U<<15))
#define S22_Toggle()            (LATBINV= (1U<<15))
#define S22_OutputEnable()      (TRISBCLR = (1U<<15))
#define S22_InputEnable()       (TRISBSET = (1U<<15))
#define S22_Get()               ((PORTB >> 15) & 0x1U)
#define S22_PIN                  GPIO_PIN_RB15

/*** Macros for S23 pin ***/
#define S23_Set()               (LATDSET = (1U<<1))
#define S23_Clear()             (LATDCLR = (1U<<1))
#define S23_Toggle()            (LATDINV= (1U<<1))
#define S23_OutputEnable()      (TRISDCLR = (1U<<1))
#define S23_InputEnable()       (TRISDSET = (1U<<1))
#define S23_Get()               ((PORTD >> 1) & 0x1U)
#define S23_PIN                  GPIO_PIN_RD1

/*** Macros for S24 pin ***/
#define S24_Set()               (LATDSET = (1U<<2))
#define S24_Clear()             (LATDCLR = (1U<<2))
#define S24_Toggle()            (LATDINV= (1U<<2))
#define S24_OutputEnable()      (TRISDCLR = (1U<<2))
#define S24_InputEnable()       (TRISDSET = (1U<<2))
#define S24_Get()               ((PORTD >> 2) & 0x1U)
#define S24_PIN                  GPIO_PIN_RD2

/*** Macros for BB2 pin ***/
#define BB2_Set()               (LATDSET = (1U<<3))
#define BB2_Clear()             (LATDCLR = (1U<<3))
#define BB2_Toggle()            (LATDINV= (1U<<3))
#define BB2_OutputEnable()      (TRISDCLR = (1U<<3))
#define BB2_InputEnable()       (TRISDSET = (1U<<3))
#define BB2_Get()               ((PORTD >> 3) & 0x1U)
#define BB2_PIN                  GPIO_PIN_RD3

/*** Macros for BB1 pin ***/
#define BB1_Set()               (LATDSET = (1U<<4))
#define BB1_Clear()             (LATDCLR = (1U<<4))
#define BB1_Toggle()            (LATDINV= (1U<<4))
#define BB1_OutputEnable()      (TRISDCLR = (1U<<4))
#define BB1_InputEnable()       (TRISDSET = (1U<<4))
#define BB1_Get()               ((PORTD >> 4) & 0x1U)
#define BB1_PIN                  GPIO_PIN_RD4

/*** Macros for S25 pin ***/
#define S25_Set()               (LATGSET = (1U<<7))
#define S25_Clear()             (LATGCLR = (1U<<7))
#define S25_Toggle()            (LATGINV= (1U<<7))
#define S25_OutputEnable()      (TRISGCLR = (1U<<7))
#define S25_InputEnable()       (TRISGSET = (1U<<7))
#define S25_Get()               ((PORTG >> 7) & 0x1U)
#define S25_PIN                  GPIO_PIN_RG7

/*** Macros for S26 pin ***/
#define S26_Set()               (LATGSET = (1U<<8))
#define S26_Clear()             (LATGCLR = (1U<<8))
#define S26_Toggle()            (LATGINV= (1U<<8))
#define S26_OutputEnable()      (TRISGCLR = (1U<<8))
#define S26_InputEnable()       (TRISGSET = (1U<<8))
#define S26_Get()               ((PORTG >> 8) & 0x1U)
#define S26_PIN                  GPIO_PIN_RG8

/*** Macros for B4 pin ***/
#define B4_Set()               (LATESET = (1U<<8))
#define B4_Clear()             (LATECLR = (1U<<8))
#define B4_Toggle()            (LATEINV= (1U<<8))
#define B4_OutputEnable()      (TRISECLR = (1U<<8))
#define B4_InputEnable()       (TRISESET = (1U<<8))
#define B4_Get()               ((PORTE >> 8) & 0x1U)
#define B4_PIN                  GPIO_PIN_RE8

/*** Macros for B3 pin ***/
#define B3_Set()               (LATESET = (1U<<9))
#define B3_Clear()             (LATECLR = (1U<<9))
#define B3_Toggle()            (LATEINV= (1U<<9))
#define B3_OutputEnable()      (TRISECLR = (1U<<9))
#define B3_InputEnable()       (TRISESET = (1U<<9))
#define B3_Get()               ((PORTE >> 9) & 0x1U)
#define B3_PIN                  GPIO_PIN_RE9

/*** Macros for B2 pin ***/
#define B2_Set()               (LATASET = (1U<<12))
#define B2_Clear()             (LATACLR = (1U<<12))
#define B2_Toggle()            (LATAINV= (1U<<12))
#define B2_OutputEnable()      (TRISACLR = (1U<<12))
#define B2_InputEnable()       (TRISASET = (1U<<12))
#define B2_Get()               ((PORTA >> 12) & 0x1U)
#define B2_PIN                  GPIO_PIN_RA12

/*** Macros for B1 pin ***/
#define B1_Set()               (LATASET = (1U<<11))
#define B1_Clear()             (LATACLR = (1U<<11))
#define B1_Toggle()            (LATAINV= (1U<<11))
#define B1_OutputEnable()      (TRISACLR = (1U<<11))
#define B1_InputEnable()       (TRISASET = (1U<<11))
#define B1_Get()               ((PORTA >> 11) & 0x1U)
#define B1_PIN                  GPIO_PIN_RA11

/*** Macros for BB3 pin ***/
#define BB3_Set()               (LATASET = (1U<<0))
#define BB3_Clear()             (LATACLR = (1U<<0))
#define BB3_Toggle()            (LATAINV= (1U<<0))
#define BB3_OutputEnable()      (TRISACLR = (1U<<0))
#define BB3_InputEnable()       (TRISASET = (1U<<0))
#define BB3_Get()               ((PORTA >> 0) & 0x1U)
#define BB3_PIN                  GPIO_PIN_RA0

/*** Macros for S28 pin ***/
#define S28_Set()               (LATASET = (1U<<1))
#define S28_Clear()             (LATACLR = (1U<<1))
#define S28_Toggle()            (LATAINV= (1U<<1))
#define S28_OutputEnable()      (TRISACLR = (1U<<1))
#define S28_InputEnable()       (TRISASET = (1U<<1))
#define S28_Get()               ((PORTA >> 1) & 0x1U)
#define S28_PIN                  GPIO_PIN_RA1

/*** Macros for S27 pin ***/
#define S27_Set()               (LATBSET = (1U<<2))
#define S27_Clear()             (LATBCLR = (1U<<2))
#define S27_Toggle()            (LATBINV= (1U<<2))
#define S27_OutputEnable()      (TRISBCLR = (1U<<2))
#define S27_InputEnable()       (TRISBSET = (1U<<2))
#define S27_Get()               ((PORTB >> 2) & 0x1U)
#define S27_PIN                  GPIO_PIN_RB2

/*** Macros for S29 pin ***/
#define S29_Set()               (LATBSET = (1U<<3))
#define S29_Clear()             (LATBCLR = (1U<<3))
#define S29_Toggle()            (LATBINV= (1U<<3))
#define S29_OutputEnable()      (TRISBCLR = (1U<<3))
#define S29_InputEnable()       (TRISBSET = (1U<<3))
#define S29_Get()               ((PORTB >> 3) & 0x1U)
#define S29_PIN                  GPIO_PIN_RB3

/*** Macros for S30 pin ***/
#define S30_Set()               (LATFSET = (1U<<9))
#define S30_Clear()             (LATFCLR = (1U<<9))
#define S30_Toggle()            (LATFINV= (1U<<9))
#define S30_OutputEnable()      (TRISFCLR = (1U<<9))
#define S30_InputEnable()       (TRISFSET = (1U<<9))
#define S30_Get()               ((PORTF >> 9) & 0x1U)
#define S30_PIN                  GPIO_PIN_RF9

/*** Macros for R3 pin ***/
#define R3_Set()               (LATFSET = (1U<<10))
#define R3_Clear()             (LATFCLR = (1U<<10))
#define R3_Toggle()            (LATFINV= (1U<<10))
#define R3_OutputEnable()      (TRISFCLR = (1U<<10))
#define R3_InputEnable()       (TRISFSET = (1U<<10))
#define R3_Get()               ((PORTF >> 10) & 0x1U)
#define R3_PIN                  GPIO_PIN_RF10

/*** Macros for R4 pin ***/
#define R4_Set()               (LATCSET = (1U<<0))
#define R4_Clear()             (LATCCLR = (1U<<0))
#define R4_Toggle()            (LATCINV= (1U<<0))
#define R4_OutputEnable()      (TRISCCLR = (1U<<0))
#define R4_InputEnable()       (TRISCSET = (1U<<0))
#define R4_Get()               ((PORTC >> 0) & 0x1U)
#define R4_PIN                  GPIO_PIN_RC0

/*** Macros for S31 pin ***/
#define S31_Set()               (LATCSET = (1U<<1))
#define S31_Clear()             (LATCCLR = (1U<<1))
#define S31_Toggle()            (LATCINV= (1U<<1))
#define S31_OutputEnable()      (TRISCCLR = (1U<<1))
#define S31_InputEnable()       (TRISCSET = (1U<<1))
#define S31_Get()               ((PORTC >> 1) & 0x1U)
#define S31_PIN                  GPIO_PIN_RC1

/*** Macros for S32 pin ***/
#define S32_Set()               (LATCSET = (1U<<2))
#define S32_Clear()             (LATCCLR = (1U<<2))
#define S32_Toggle()            (LATCINV= (1U<<2))
#define S32_OutputEnable()      (TRISCCLR = (1U<<2))
#define S32_InputEnable()       (TRISCSET = (1U<<2))
#define S32_Get()               ((PORTC >> 2) & 0x1U)
#define S32_PIN                  GPIO_PIN_RC2

/*** Macros for R1 pin ***/
#define R1_Set()               (LATCSET = (1U<<11))
#define R1_Clear()             (LATCCLR = (1U<<11))
#define R1_Toggle()            (LATCINV= (1U<<11))
#define R1_OutputEnable()      (TRISCCLR = (1U<<11))
#define R1_InputEnable()       (TRISCSET = (1U<<11))
#define R1_Get()               ((PORTC >> 11) & 0x1U)
#define R1_PIN                  GPIO_PIN_RC11

/*** Macros for R2 pin ***/
#define R2_Set()               (LATGSET = (1U<<11))
#define R2_Clear()             (LATGCLR = (1U<<11))
#define R2_Toggle()            (LATGINV= (1U<<11))
#define R2_OutputEnable()      (TRISGCLR = (1U<<11))
#define R2_InputEnable()       (TRISGSET = (1U<<11))
#define R2_Get()               ((PORTG >> 11) & 0x1U)
#define R2_PIN                  GPIO_PIN_RG11

/*** Macros for RR4 pin ***/
#define RR4_Set()               (LATFSET = (1U<<13))
#define RR4_Clear()             (LATFCLR = (1U<<13))
#define RR4_Toggle()            (LATFINV= (1U<<13))
#define RR4_OutputEnable()      (TRISFCLR = (1U<<13))
#define RR4_InputEnable()       (TRISFSET = (1U<<13))
#define RR4_Get()               ((PORTF >> 13) & 0x1U)
#define RR4_PIN                  GPIO_PIN_RF13

/*** Macros for RR2 pin ***/
#define RR2_Set()               (LATFSET = (1U<<12))
#define RR2_Clear()             (LATFCLR = (1U<<12))
#define RR2_Toggle()            (LATFINV= (1U<<12))
#define RR2_OutputEnable()      (TRISFCLR = (1U<<12))
#define RR2_InputEnable()       (TRISFSET = (1U<<12))
#define RR2_Get()               ((PORTF >> 12) & 0x1U)
#define RR2_PIN                  GPIO_PIN_RF12

/*** Macros for S3 pin ***/
#define S3_Set()               (LATDSET = (1U<<14))
#define S3_Clear()             (LATDCLR = (1U<<14))
#define S3_Toggle()            (LATDINV= (1U<<14))
#define S3_OutputEnable()      (TRISDCLR = (1U<<14))
#define S3_InputEnable()       (TRISDSET = (1U<<14))
#define S3_Get()               ((PORTD >> 14) & 0x1U)
#define S3_PIN                  GPIO_PIN_RD14

/*** Macros for S1 pin ***/
#define S1_Set()               (LATDSET = (1U<<15))
#define S1_Clear()             (LATDCLR = (1U<<15))
#define S1_Toggle()            (LATDINV= (1U<<15))
#define S1_OutputEnable()      (TRISDCLR = (1U<<15))
#define S1_InputEnable()       (TRISDSET = (1U<<15))
#define S1_Get()               ((PORTD >> 15) & 0x1U)
#define S1_PIN                  GPIO_PIN_RD15

/*** Macros for S2 pin ***/
#define S2_Set()               (LATASET = (1U<<8))
#define S2_Clear()             (LATACLR = (1U<<8))
#define S2_Toggle()            (LATAINV= (1U<<8))
#define S2_OutputEnable()      (TRISACLR = (1U<<8))
#define S2_InputEnable()       (TRISASET = (1U<<8))
#define S2_Get()               ((PORTA >> 8) & 0x1U)
#define S2_PIN                  GPIO_PIN_RA8

/*** Macros for RR1 pin ***/
#define RR1_Set()               (LATBSET = (1U<<4))
#define RR1_Clear()             (LATBCLR = (1U<<4))
#define RR1_Toggle()            (LATBINV= (1U<<4))
#define RR1_OutputEnable()      (TRISBCLR = (1U<<4))
#define RR1_InputEnable()       (TRISBSET = (1U<<4))
#define RR1_Get()               ((PORTB >> 4) & 0x1U)
#define RR1_PIN                  GPIO_PIN_RB4

/*** Macros for RR3 pin ***/
#define RR3_Set()               (LATASET = (1U<<4))
#define RR3_Clear()             (LATACLR = (1U<<4))
#define RR3_Toggle()            (LATAINV= (1U<<4))
#define RR3_OutputEnable()      (TRISACLR = (1U<<4))
#define RR3_InputEnable()       (TRISASET = (1U<<4))
#define RR3_Get()               ((PORTA >> 4) & 0x1U)
#define RR3_PIN                  GPIO_PIN_RA4

/*** Macros for R5 pin ***/
#define R5_Set()               (LATESET = (1U<<0))
#define R5_Clear()             (LATECLR = (1U<<0))
#define R5_Toggle()            (LATEINV= (1U<<0))
#define R5_OutputEnable()      (TRISECLR = (1U<<0))
#define R5_InputEnable()       (TRISESET = (1U<<0))
#define R5_Get()               ((PORTE >> 0) & 0x1U)
#define R5_PIN                  GPIO_PIN_RE0

/*** Macros for R6 pin ***/
#define R6_Set()               (LATESET = (1U<<1))
#define R6_Clear()             (LATECLR = (1U<<1))
#define R6_Toggle()            (LATEINV= (1U<<1))
#define R6_OutputEnable()      (TRISECLR = (1U<<1))
#define R6_InputEnable()       (TRISESET = (1U<<1))
#define R6_Get()               ((PORTE >> 1) & 0x1U)
#define R6_PIN                  GPIO_PIN_RE1

/*** Macros for S4 pin ***/
#define S4_Set()               (LATFSET = (1U<<5))
#define S4_Clear()             (LATFCLR = (1U<<5))
#define S4_Toggle()            (LATFINV= (1U<<5))
#define S4_OutputEnable()      (TRISFCLR = (1U<<5))
#define S4_InputEnable()       (TRISFSET = (1U<<5))
#define S4_Get()               ((PORTF >> 5) & 0x1U)
#define S4_PIN                  GPIO_PIN_RF5

/*** Macros for S5 pin ***/
#define S5_Set()               (LATASET = (1U<<14))
#define S5_Clear()             (LATACLR = (1U<<14))
#define S5_Toggle()            (LATAINV= (1U<<14))
#define S5_OutputEnable()      (TRISACLR = (1U<<14))
#define S5_InputEnable()       (TRISASET = (1U<<14))
#define S5_Get()               ((PORTA >> 14) & 0x1U)
#define S5_PIN                  GPIO_PIN_RA14

/*** Macros for S6 pin ***/
#define S6_Set()               (LATASET = (1U<<15))
#define S6_Clear()             (LATACLR = (1U<<15))
#define S6_Toggle()            (LATAINV= (1U<<15))
#define S6_OutputEnable()      (TRISACLR = (1U<<15))
#define S6_InputEnable()       (TRISASET = (1U<<15))
#define S6_Get()               ((PORTA >> 15) & 0x1U)
#define S6_PIN                  GPIO_PIN_RA15

/*** Macros for S7 pin ***/
#define S7_Set()               (LATDSET = (1U<<8))
#define S7_Clear()             (LATDCLR = (1U<<8))
#define S7_Toggle()            (LATDINV= (1U<<8))
#define S7_OutputEnable()      (TRISDCLR = (1U<<8))
#define S7_InputEnable()       (TRISDSET = (1U<<8))
#define S7_Get()               ((PORTD >> 8) & 0x1U)
#define S7_PIN                  GPIO_PIN_RD8

/*** Macros for S8 pin ***/
#define S8_Set()               (LATBSET = (1U<<7))
#define S8_Clear()             (LATBCLR = (1U<<7))
#define S8_Toggle()            (LATBINV= (1U<<7))
#define S8_OutputEnable()      (TRISBCLR = (1U<<7))
#define S8_InputEnable()       (TRISBSET = (1U<<7))
#define S8_Get()               ((PORTB >> 7) & 0x1U)
#define S8_PIN                  GPIO_PIN_RB7

/*** Macros for S9 pin ***/
#define S9_Set()               (LATCSET = (1U<<13))
#define S9_Clear()             (LATCCLR = (1U<<13))
#define S9_Toggle()            (LATCINV= (1U<<13))
#define S9_OutputEnable()      (TRISCCLR = (1U<<13))
#define S9_InputEnable()       (TRISCSET = (1U<<13))
#define S9_Get()               ((PORTC >> 13) & 0x1U)
#define S9_PIN                  GPIO_PIN_RC13

/*** Macros for S10 pin ***/
#define S10_Set()               (LATBSET = (1U<<8))
#define S10_Clear()             (LATBCLR = (1U<<8))
#define S10_Toggle()            (LATBINV= (1U<<8))
#define S10_OutputEnable()      (TRISBCLR = (1U<<8))
#define S10_InputEnable()       (TRISBSET = (1U<<8))
#define S10_Get()               ((PORTB >> 8) & 0x1U)
#define S10_PIN                  GPIO_PIN_RB8

/*** Macros for S11 pin ***/
#define S11_Set()               (LATBSET = (1U<<9))
#define S11_Clear()             (LATBCLR = (1U<<9))
#define S11_Toggle()            (LATBINV= (1U<<9))
#define S11_OutputEnable()      (TRISBCLR = (1U<<9))
#define S11_InputEnable()       (TRISBSET = (1U<<9))
#define S11_Get()               ((PORTB >> 9) & 0x1U)
#define S11_PIN                  GPIO_PIN_RB9

/*** Macros for S12 pin ***/
#define S12_Set()               (LATCSET = (1U<<6))
#define S12_Clear()             (LATCCLR = (1U<<6))
#define S12_Toggle()            (LATCINV= (1U<<6))
#define S12_OutputEnable()      (TRISCCLR = (1U<<6))
#define S12_InputEnable()       (TRISCSET = (1U<<6))
#define S12_Get()               ((PORTC >> 6) & 0x1U)
#define S12_PIN                  GPIO_PIN_RC6

/*** Macros for S14 pin ***/
#define S14_Set()               (LATCSET = (1U<<7))
#define S14_Clear()             (LATCCLR = (1U<<7))
#define S14_Toggle()            (LATCINV= (1U<<7))
#define S14_OutputEnable()      (TRISCCLR = (1U<<7))
#define S14_InputEnable()       (TRISCSET = (1U<<7))
#define S14_Get()               ((PORTC >> 7) & 0x1U)
#define S14_PIN                  GPIO_PIN_RC7

/*** Macros for S13 pin ***/
#define S13_Set()               (LATDSET = (1U<<12))
#define S13_Clear()             (LATDCLR = (1U<<12))
#define S13_Toggle()            (LATDINV= (1U<<12))
#define S13_OutputEnable()      (TRISDCLR = (1U<<12))
#define S13_InputEnable()       (TRISDSET = (1U<<12))
#define S13_Get()               ((PORTD >> 12) & 0x1U)
#define S13_PIN                  GPIO_PIN_RD12

/*** Macros for S15 pin ***/
#define S15_Set()               (LATDSET = (1U<<13))
#define S15_Clear()             (LATDCLR = (1U<<13))
#define S15_Toggle()            (LATDINV= (1U<<13))
#define S15_OutputEnable()      (TRISDCLR = (1U<<13))
#define S15_InputEnable()       (TRISDSET = (1U<<13))
#define S15_Get()               ((PORTD >> 13) & 0x1U)
#define S15_PIN                  GPIO_PIN_RD13

/*** Macros for S16 pin ***/
#define S16_Set()               (LATCSET = (1U<<8))
#define S16_Clear()             (LATCCLR = (1U<<8))
#define S16_Toggle()            (LATCINV= (1U<<8))
#define S16_OutputEnable()      (TRISCCLR = (1U<<8))
#define S16_InputEnable()       (TRISCSET = (1U<<8))
#define S16_Get()               ((PORTC >> 8) & 0x1U)
#define S16_PIN                  GPIO_PIN_RC8

/*** Macros for L8 pin ***/
#define L8_Set()               (LATDSET = (1U<<5))
#define L8_Clear()             (LATDCLR = (1U<<5))
#define L8_Toggle()            (LATDINV= (1U<<5))
#define L8_OutputEnable()      (TRISDCLR = (1U<<5))
#define L8_InputEnable()       (TRISDSET = (1U<<5))
#define L8_Get()               ((PORTD >> 5) & 0x1U)
#define L8_PIN                  GPIO_PIN_RD5

/*** Macros for L7 pin ***/
#define L7_Set()               (LATDSET = (1U<<6))
#define L7_Clear()             (LATDCLR = (1U<<6))
#define L7_Toggle()            (LATDINV= (1U<<6))
#define L7_OutputEnable()      (TRISDCLR = (1U<<6))
#define L7_InputEnable()       (TRISDSET = (1U<<6))
#define L7_Get()               ((PORTD >> 6) & 0x1U)
#define L7_PIN                  GPIO_PIN_RD6

/*** Macros for L6 pin ***/
#define L6_Set()               (LATCSET = (1U<<9))
#define L6_Clear()             (LATCCLR = (1U<<9))
#define L6_Toggle()            (LATCINV= (1U<<9))
#define L6_OutputEnable()      (TRISCCLR = (1U<<9))
#define L6_InputEnable()       (TRISCSET = (1U<<9))
#define L6_Get()               ((PORTC >> 9) & 0x1U)
#define L6_PIN                  GPIO_PIN_RC9

/*** Macros for L5 pin ***/
#define L5_Set()               (LATGSET = (1U<<1))
#define L5_Clear()             (LATGCLR = (1U<<1))
#define L5_Toggle()            (LATGINV= (1U<<1))
#define L5_OutputEnable()      (TRISGCLR = (1U<<1))
#define L5_InputEnable()       (TRISGSET = (1U<<1))
#define L5_Get()               ((PORTG >> 1) & 0x1U)
#define L5_PIN                  GPIO_PIN_RG1

/*** Macros for L4 pin ***/
#define L4_Set()               (LATGSET = (1U<<0))
#define L4_Clear()             (LATGCLR = (1U<<0))
#define L4_Toggle()            (LATGINV= (1U<<0))
#define L4_OutputEnable()      (TRISGCLR = (1U<<0))
#define L4_InputEnable()       (TRISGSET = (1U<<0))
#define L4_Get()               ((PORTG >> 0) & 0x1U)
#define L4_PIN                  GPIO_PIN_RG0

/*** Macros for LL2 pin ***/
#define LL2_Set()               (LATFSET = (1U<<7))
#define LL2_Clear()             (LATFCLR = (1U<<7))
#define LL2_Toggle()            (LATFINV= (1U<<7))
#define LL2_OutputEnable()      (TRISFCLR = (1U<<7))
#define LL2_InputEnable()       (TRISFSET = (1U<<7))
#define LL2_Get()               ((PORTF >> 7) & 0x1U)
#define LL2_PIN                  GPIO_PIN_RF7

/*** Macros for S17 pin ***/
#define S17_Set()               (LATBSET = (1U<<10))
#define S17_Clear()             (LATBCLR = (1U<<10))
#define S17_Toggle()            (LATBINV= (1U<<10))
#define S17_OutputEnable()      (TRISBCLR = (1U<<10))
#define S17_InputEnable()       (TRISBSET = (1U<<10))
#define S17_Get()               ((PORTB >> 10) & 0x1U)
#define S17_PIN                  GPIO_PIN_RB10

/*** Macros for S18 pin ***/
#define S18_Set()               (LATBSET = (1U<<11))
#define S18_Clear()             (LATBCLR = (1U<<11))
#define S18_Toggle()            (LATBINV= (1U<<11))
#define S18_OutputEnable()      (TRISBCLR = (1U<<11))
#define S18_InputEnable()       (TRISBSET = (1U<<11))
#define S18_Get()               ((PORTB >> 11) & 0x1U)
#define S18_PIN                  GPIO_PIN_RB11

/*** Macros for LL1 pin ***/
#define LL1_Set()               (LATGSET = (1U<<14))
#define LL1_Clear()             (LATGCLR = (1U<<14))
#define LL1_Toggle()            (LATGINV= (1U<<14))
#define LL1_OutputEnable()      (TRISGCLR = (1U<<14))
#define LL1_InputEnable()       (TRISGSET = (1U<<14))
#define LL1_Get()               ((PORTG >> 14) & 0x1U)
#define LL1_PIN                  GPIO_PIN_RG14

/*** Macros for LL3 pin ***/
#define LL3_Set()               (LATGSET = (1U<<12))
#define LL3_Clear()             (LATGCLR = (1U<<12))
#define LL3_Toggle()            (LATGINV= (1U<<12))
#define LL3_OutputEnable()      (TRISGCLR = (1U<<12))
#define LL3_InputEnable()       (TRISGSET = (1U<<12))
#define LL3_Get()               ((PORTG >> 12) & 0x1U)
#define LL3_PIN                  GPIO_PIN_RG12

/*** Macros for L3 pin ***/
#define L3_Set()               (LATGSET = (1U<<13))
#define L3_Clear()             (LATGCLR = (1U<<13))
#define L3_Toggle()            (LATGINV= (1U<<13))
#define L3_OutputEnable()      (TRISGCLR = (1U<<13))
#define L3_InputEnable()       (TRISGSET = (1U<<13))
#define L3_Get()               ((PORTG >> 13) & 0x1U)
#define L3_PIN                  GPIO_PIN_RG13

/*** Macros for S19 pin ***/
#define S19_Set()               (LATBSET = (1U<<12))
#define S19_Clear()             (LATBCLR = (1U<<12))
#define S19_Toggle()            (LATBINV= (1U<<12))
#define S19_OutputEnable()      (TRISBCLR = (1U<<12))
#define S19_InputEnable()       (TRISBSET = (1U<<12))
#define S19_Get()               ((PORTB >> 12) & 0x1U)
#define S19_PIN                  GPIO_PIN_RB12

/*** Macros for S20 pin ***/
#define S20_Set()               (LATBSET = (1U<<13))
#define S20_Clear()             (LATBCLR = (1U<<13))
#define S20_Toggle()            (LATBINV= (1U<<13))
#define S20_OutputEnable()      (TRISBCLR = (1U<<13))
#define S20_InputEnable()       (TRISBSET = (1U<<13))
#define S20_Get()               ((PORTB >> 13) & 0x1U)
#define S20_PIN                  GPIO_PIN_RB13


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/


#define    GPIO_PORT_A  (0)
#define    GPIO_PORT_B  (1)
#define    GPIO_PORT_C  (2)
#define    GPIO_PORT_D  (3)
#define    GPIO_PORT_E  (4)
#define    GPIO_PORT_F  (5)
#define    GPIO_PORT_G  (6)
typedef uint32_t GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/


#define     GPIO_PIN_RA0  (0U)
#define     GPIO_PIN_RA1  (1U)
#define     GPIO_PIN_RA4  (4U)
#define     GPIO_PIN_RA7  (7U)
#define     GPIO_PIN_RA8  (8U)
#define     GPIO_PIN_RA10  (10U)
#define     GPIO_PIN_RA11  (11U)
#define     GPIO_PIN_RA12  (12U)
#define     GPIO_PIN_RA14  (14U)
#define     GPIO_PIN_RA15  (15U)
#define     GPIO_PIN_RB0  (16U)
#define     GPIO_PIN_RB1  (17U)
#define     GPIO_PIN_RB2  (18U)
#define     GPIO_PIN_RB3  (19U)
#define     GPIO_PIN_RB4  (20U)
#define     GPIO_PIN_RB5  (21U)
#define     GPIO_PIN_RB6  (22U)
#define     GPIO_PIN_RB7  (23U)
#define     GPIO_PIN_RB8  (24U)
#define     GPIO_PIN_RB9  (25U)
#define     GPIO_PIN_RB10  (26U)
#define     GPIO_PIN_RB11  (27U)
#define     GPIO_PIN_RB12  (28U)
#define     GPIO_PIN_RB13  (29U)
#define     GPIO_PIN_RB14  (30U)
#define     GPIO_PIN_RB15  (31U)
#define     GPIO_PIN_RC0  (32U)
#define     GPIO_PIN_RC1  (33U)
#define     GPIO_PIN_RC2  (34U)
#define     GPIO_PIN_RC6  (38U)
#define     GPIO_PIN_RC7  (39U)
#define     GPIO_PIN_RC8  (40U)
#define     GPIO_PIN_RC9  (41U)
#define     GPIO_PIN_RC10  (42U)
#define     GPIO_PIN_RC11  (43U)
#define     GPIO_PIN_RC12  (44U)
#define     GPIO_PIN_RC13  (45U)
#define     GPIO_PIN_RC15  (47U)
#define     GPIO_PIN_RD1  (49U)
#define     GPIO_PIN_RD2  (50U)
#define     GPIO_PIN_RD3  (51U)
#define     GPIO_PIN_RD4  (52U)
#define     GPIO_PIN_RD5  (53U)
#define     GPIO_PIN_RD6  (54U)
#define     GPIO_PIN_RD8  (56U)
#define     GPIO_PIN_RD12  (60U)
#define     GPIO_PIN_RD13  (61U)
#define     GPIO_PIN_RD14  (62U)
#define     GPIO_PIN_RD15  (63U)
#define     GPIO_PIN_RE0  (64U)
#define     GPIO_PIN_RE1  (65U)
#define     GPIO_PIN_RE8  (72U)
#define     GPIO_PIN_RE9  (73U)
#define     GPIO_PIN_RE12  (76U)
#define     GPIO_PIN_RE13  (77U)
#define     GPIO_PIN_RE14  (78U)
#define     GPIO_PIN_RE15  (79U)
#define     GPIO_PIN_RF0  (80U)
#define     GPIO_PIN_RF1  (81U)
#define     GPIO_PIN_RF5  (85U)
#define     GPIO_PIN_RF6  (86U)
#define     GPIO_PIN_RF7  (87U)
#define     GPIO_PIN_RF9  (89U)
#define     GPIO_PIN_RF10  (90U)
#define     GPIO_PIN_RF12  (92U)
#define     GPIO_PIN_RF13  (93U)
#define     GPIO_PIN_RG0  (96U)
#define     GPIO_PIN_RG1  (97U)
#define     GPIO_PIN_RG6  (102U)
#define     GPIO_PIN_RG7  (103U)
#define     GPIO_PIN_RG8  (104U)
#define     GPIO_PIN_RG9  (105U)
#define     GPIO_PIN_RG10  (106U)
#define     GPIO_PIN_RG11  (107U)
#define     GPIO_PIN_RG12  (108U)
#define     GPIO_PIN_RG13  (109U)
#define     GPIO_PIN_RG14  (110U)
#define     GPIO_PIN_RG15  (111U)

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
#define    GPIO_PIN_NONE   (-1)

typedef uint32_t GPIO_PIN;


void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
	 uint32_t xvalue = (uint32_t)value;
    GPIO_PortWrite((pin>>4U), (uint32_t)(0x1U) << (pin & 0xFU), (xvalue) << (pin & 0xFU));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return ((((GPIO_PortRead((GPIO_PORT)(pin>>4U))) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (((GPIO_PortLatchRead((GPIO_PORT)(pin>>4U)) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
