/*
 * File:   interrupt.cpp
 * Author: takum
 *
 * Created on March 13, 2026, 2:46 PM
 */

#include "interrupt.hpp"

uint8_t U1RxData;
uint8_t txData[3] = {0xff,0x80,0x50};

void U1_Callback(uintptr_t) {
    UART1_Read(&U1RxData, 1);
    UART1_Write(UART_Data.data, 3);
}

int a = 0;

void T2_Callback(uint32_t,uintptr_t) {
    UART_Data.right = R;
    UART_Data.back = B;
    UART_Data.left = L;
    UART_Data.Angel_flag = Angel_or;
    cartesian2 vec2;
    vec2 = calc_line();
    UART_Data.data[0] = vec2.x*127.0+128;
    UART_Data.data[1] = vec2.y*127.0+128;
    int j = 0;
    for(int i = 0;i < 32;i++){
        if(get_Angel(i))j++;
    }
    UART_Data.ImDIE_flag = (j > 16);
    LED1=a<100?LED1:~LED1;
    LED2 = UART_Data.Angel_flag;
    LED3 = (bool)(B|L|R);
    a=(a+1)*(a<100);
}