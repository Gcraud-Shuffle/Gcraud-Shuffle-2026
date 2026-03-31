/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.cpp

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************z
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>  // Defines true
#include <stddef.h>   // Defines NULL
#include <stdlib.h>   // Defines EXIT_FAILURE

#include "../GC_LINE_2026_INCHEON.X/algo.hpp"
#include "../GC_LINE_2026_INCHEON.X/common.hpp"
#include "../GC_LINE_2026_INCHEON.X/interrupt.hpp"
#include "../GC_LINE_2026_INCHEON.X/line_sensor.hpp"
#include "../GC_LINE_2026_INCHEON.X/potentiometer.hpp"
#include "../GC_LINE_2026_INCHEON.X/struct.hpp"
#include "../GC_LINE_2026_INCHEON.X/user.hpp"
#include "../GC_LINE_2026_INCHEON.X/algo.hpp"
#include "definitions.h"  // SYS function prototypes
#include "u1.hpp"         // for debug
#include "u5.hpp"         // for communication with main mcu

u_line_data UART_Data;

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);
//    CDAC2_DataWrite(DACOUT(3));
    CDAC2_DataWrite(2000);
    TMR2_CallbackRegister(T2_Callback, (uintptr_t)nullptr);
    TMR2_Start();
    U1.init();
    U5.init();
    UART1_Read(&U1RxData, 1);
    UART1_ReadCallbackRegister(U1_Callback, (uintptr_t)nullptr);
    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        set2potentionmeter(0);
        ADCHS_ChannelConversionStart(ADCHS_CH16);
        while (!ADCHS_ChannelResultIsReady(ADCHS_CH16));
        ANALOG_VALUE = ADCHS_ChannelResultGet(ADCHS_CH16);
        U5.head().send_uint32(Angel_or).tail();
    }

    /* Execution should not come here during normal operation */

    return (EXIT_FAILURE);
}

/*******************************************************************************
 End of File
*/
