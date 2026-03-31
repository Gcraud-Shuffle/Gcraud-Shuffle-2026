/*******************************************************************************
  UART5 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart5.c

  Summary:
    UART5 PLIB Implementation File

  Description:
    None

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

#include "device.h"
#include "plib_uart5.h"
#include "interrupts.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART5 Implementation
// *****************************************************************************
// *****************************************************************************

volatile static UART_OBJECT uart5Obj;

void static UART5_ErrorClear( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint8_t dummyData = 0u;

    errors = (UART_ERROR)(U5STA & (_U5STA_OERR_MASK | _U5STA_FERR_MASK | _U5STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        /* If it's a overrun error then clear it to flush FIFO */
        if((U5STA & _U5STA_OERR_MASK) != 0U)
        {
            U5STACLR = _U5STA_OERR_MASK;
        }

        /* Read existing error bytes from FIFO to clear parity and framing error flags */
        while((U5STA & _U5STA_URXDA_MASK) != 0U)
        {
            dummyData = (uint8_t)U5RXREG;
        }

        /* Clear error interrupt flag */
        IFS2CLR = _IFS2_U5EIF_MASK;

        /* Clear up the receive interrupt flag so that RX interrupt is not
         * triggered for error bytes */
        IFS2CLR = _IFS2_U5RXIF_MASK;
    }

    // Ignore the warning
    (void)dummyData;
}

void UART5_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0*/
    /* PDSEL = 0 */
    /* BRGH = 1 */
    /* RXINV = 0 */
    /* ABAUD = 0 */
    /* LPBACK = 0 */
    /* WAKE = 0 */
    /* SIDL = 0 */
    /* RUNOVF = 0 */
    /* CLKSEL = 0 */
    /* SLPEN = 0 */
    /* UEN = 0 */
    U5MODE = 0x8;

    /* Enable UART5 Receiver, Transmitter and TX Interrupt selection */
    U5STASET = (_U5STA_UTXEN_MASK | _U5STA_URXEN_MASK | _U5STA_UTXISEL1_MASK );

    /* BAUD Rate register Setup */
    U5BRG = 129;

    /* Disable Interrupts */
    IEC2CLR = _IEC2_U5EIE_MASK;

    IEC2CLR = _IEC2_U5RXIE_MASK;

    IEC2CLR = _IEC2_U5TXIE_MASK;

    /* Initialize instance object */
    uart5Obj.rxBuffer = NULL;
    uart5Obj.rxSize = 0;
    uart5Obj.rxProcessedSize = 0;
    uart5Obj.rxBusyStatus = false;
    uart5Obj.rxCallback = NULL;
    uart5Obj.txBuffer = NULL;
    uart5Obj.txSize = 0;
    uart5Obj.txProcessedSize = 0;
    uart5Obj.txBusyStatus = false;
    uart5Obj.txCallback = NULL;
    uart5Obj.errors = UART_ERROR_NONE;

    /* Turn ON UART5 */
    U5MODESET = _U5MODE_ON_MASK;
}

bool UART5_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t status_ctrl;
    uint32_t uxbrg = 0;

    if(uart5Obj.rxBusyStatus == true)
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }
    if (uart5Obj.txBusyStatus == true)
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if ((baud == 0U) || ((setup->dataWidth == UART_DATA_9_BIT) && (setup->parity != UART_PARITY_NONE)))
        {
            return status;
        }

        if(srcClkFreq == 0U)
        {
            srcClkFreq = UART5_FrequencyGet();
        }

        /* Calculate BRG value */
        uxbrg = (((srcClkFreq >> 2) + (baud >> 1)) / baud);

        /* Check if the baud value can be set with low baud settings */
        if (uxbrg < 1U)
        {
            return status;
        }

        uxbrg -= 1U;

        if (uxbrg > UINT16_MAX)
        {
            return status;
        }

        /* Turn OFF UART5. Save UTXEN, URXEN and UTXBRK bits as these are cleared upon disabling UART */

        status_ctrl = U5STA & (_U5STA_UTXEN_MASK | _U5STA_URXEN_MASK | _U5STA_UTXBRK_MASK);

        U5MODECLR = _U5MODE_ON_MASK;

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            /* Configure UART5 mode */
            U5MODE = (U5MODE & (~_U5MODE_PDSEL_MASK)) | setup->dataWidth;
        }
        else
        {
            /* Configure UART5 mode */
            U5MODE = (U5MODE & (~_U5MODE_PDSEL_MASK)) | setup->parity;
        }

        /* Configure UART5 mode */
        U5MODE = (U5MODE & (~_U5MODE_STSEL_MASK)) | setup->stopBits;

        /* Configure UART5 Baud Rate */
        U5BRG = uxbrg;

        U5MODESET = _U5MODE_ON_MASK;

        /* Restore UTXEN, URXEN and UTXBRK bits. */
        U5STASET = status_ctrl;

        status = true;
    }

    return status;
}

bool UART5_Read(void* buffer, const size_t size )
{
    bool status = false;

    if(buffer != NULL)
    {
        /* Check if receive request is in progress */
        if(uart5Obj.rxBusyStatus == false)
        {
            /* Clear error flags and flush out error data that may have been received when no active request was pending */
            UART5_ErrorClear();

            uart5Obj.rxBuffer = buffer;
            uart5Obj.rxSize = size;
            uart5Obj.rxProcessedSize = 0;
            uart5Obj.rxBusyStatus = true;
            uart5Obj.errors = UART_ERROR_NONE;
            status = true;

            /* Enable UART5_FAULT Interrupt */
            IEC2SET = _IEC2_U5EIE_MASK;

            /* Enable UART5_RX Interrupt */
            IEC2SET = _IEC2_U5RXIE_MASK;
        }
    }

    return status;
}

bool UART5_Write( void* buffer, const size_t size )
{
    bool status = false;

    if(buffer != NULL)
    {
        /* Check if transmit request is in progress */
        if(uart5Obj.txBusyStatus == false)
        {
            uart5Obj.txBuffer = buffer;
            uart5Obj.txSize = size;
            uart5Obj.txProcessedSize = 0;
            uart5Obj.txBusyStatus = true;
            status = true;

            size_t txProcessedSize = uart5Obj.txProcessedSize;
            size_t txSize = uart5Obj.txSize;

            /* Initiate the transfer by writing as many bytes as we can */
             while(((U5STA & _U5STA_UTXBF_MASK) == 0U) && (txSize > txProcessedSize) )
            {
                if (( U5MODE & (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK)) == (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK))
                {
                    /* 9-bit mode */
                    U5TXREG = ((uint16_t*)uart5Obj.txBuffer)[txProcessedSize];
                    txProcessedSize++;
                }
                else
                {
                    /* 8-bit mode */
                    U5TXREG = ((uint8_t*)uart5Obj.txBuffer)[txProcessedSize];
                    txProcessedSize++;
                }
            }

            uart5Obj.txProcessedSize = txProcessedSize;

            IEC2SET = _IEC2_U5TXIE_MASK;
        }
    }

    return status;
}

UART_ERROR UART5_ErrorGet( void )
{
    UART_ERROR errors = uart5Obj.errors;

    uart5Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}

bool UART5_AutoBaudQuery( void )
{
    bool autobaudcheck = false;
    if((U5MODE & _U5MODE_ABAUD_MASK) != 0U)
    {

      autobaudcheck = true;

    }
    return autobaudcheck;
}

void UART5_AutoBaudSet( bool enable )
{
    if( enable == true )
    {
        U5MODESET = _U5MODE_ABAUD_MASK;
    }

    /* Turning off ABAUD if it was on can lead to unpredictable behavior, so that
       direction of control is not allowed in this function.                      */
}

void UART5_ReadCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart5Obj.rxCallback = callback;

    uart5Obj.rxContext = context;
}

bool UART5_ReadIsBusy( void )
{
    return uart5Obj.rxBusyStatus;
}

size_t UART5_ReadCountGet( void )
{
    return uart5Obj.rxProcessedSize;
}

bool UART5_ReadAbort(void)
{
    if (uart5Obj.rxBusyStatus == true)
    {
        /* Disable the fault interrupt */
        IEC2CLR = _IEC2_U5EIE_MASK;

        /* Disable the receive interrupt */
        IEC2CLR = _IEC2_U5RXIE_MASK;

        uart5Obj.rxBusyStatus = false;

        /* If required application should read the num bytes processed prior to calling the read abort API */
        uart5Obj.rxSize = 0U;
        uart5Obj.rxProcessedSize = 0U;
    }

    return true;
}

void UART5_WriteCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart5Obj.txCallback = callback;

    uart5Obj.txContext = context;
}

bool UART5_WriteIsBusy( void )
{
    return uart5Obj.txBusyStatus;
}

size_t UART5_WriteCountGet( void )
{
    return uart5Obj.txProcessedSize;
}

void __attribute__((used)) UART5_FAULT_InterruptHandler (void)
{
    /* Save the error to be reported later */
    uart5Obj.errors = (U5STA & (_U5STA_OERR_MASK | _U5STA_FERR_MASK | _U5STA_PERR_MASK));

    /* Disable the fault interrupt */
    IEC2CLR = _IEC2_U5EIE_MASK;

    /* Disable the receive interrupt */
    IEC2CLR = _IEC2_U5RXIE_MASK;

    /* Clear size and rx status */
    uart5Obj.rxBusyStatus = false;

    UART5_ErrorClear();

    /* Client must call UARTx_ErrorGet() function to get the errors */
    if( uart5Obj.rxCallback != NULL )
    {
        uintptr_t rxContext = uart5Obj.rxContext;

        uart5Obj.rxCallback(rxContext);
    }
}

void __attribute__((used)) UART5_RX_InterruptHandler (void)
{
    if(uart5Obj.rxBusyStatus == true)
    {
        size_t rxProcessedSize = uart5Obj.rxProcessedSize;
        size_t rxSize = uart5Obj.rxSize;

        while((_U5STA_URXDA_MASK == (U5STA & _U5STA_URXDA_MASK)) && (rxSize > rxProcessedSize) )
        {
            if (( U5MODE & (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK)) == (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                ((uint16_t*)uart5Obj.rxBuffer)[rxProcessedSize] = (uint16_t )(U5RXREG);
            }
            else
            {
                /* 8-bit mode */
                ((uint8_t*)uart5Obj.rxBuffer)[rxProcessedSize] = (uint8_t )(U5RXREG);
            }
            rxProcessedSize++;
        }

        uart5Obj.rxProcessedSize = rxProcessedSize;

        /* Clear UART5 RX Interrupt flag */
        IFS2CLR = _IFS2_U5RXIF_MASK;

        /* Check if the buffer is done */
        if(rxProcessedSize >= uart5Obj.rxSize)
        {
            uart5Obj.rxBusyStatus = false;

            /* Disable the fault interrupt */
            IEC2CLR = _IEC2_U5EIE_MASK;

            /* Disable the receive interrupt */
            IEC2CLR = _IEC2_U5RXIE_MASK;

            if(uart5Obj.rxCallback != NULL)
            {
                uintptr_t rxContext = uart5Obj.rxContext;

                uart5Obj.rxCallback(rxContext);
            }
        }
    }
    else
    {
        /* Nothing to process */
    }
}

void __attribute__((used)) UART5_TX_InterruptHandler (void)
{
    if(uart5Obj.txBusyStatus == true)
    {
        size_t txSize = uart5Obj.txSize;
        size_t txProcessedSize = uart5Obj.txProcessedSize;

        while(((U5STA & _U5STA_UTXBF_MASK) == 0U) && (txSize > txProcessedSize) )
        {
            if (( U5MODE & (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK)) == (_U5MODE_PDSEL0_MASK | _U5MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                U5TXREG = ((uint16_t*)uart5Obj.txBuffer)[txProcessedSize];
                txProcessedSize++;
            }
            else
            {
                /* 8-bit mode */
                U5TXREG = ((uint8_t*)uart5Obj.txBuffer)[txProcessedSize];
                txProcessedSize++;
            }
        }

        uart5Obj.txProcessedSize = txProcessedSize;

        /* Clear UART5TX Interrupt flag */
        IFS2CLR = _IFS2_U5TXIF_MASK;

        /* Check if the buffer is done */
        if(txProcessedSize >= uart5Obj.txSize)
        {
            uart5Obj.txBusyStatus = false;

            /* Disable the transmit interrupt, to avoid calling ISR continuously */
            IEC2CLR = _IEC2_U5TXIE_MASK;

            if(uart5Obj.txCallback != NULL)
            {
                uintptr_t txContext = uart5Obj.txContext;

                uart5Obj.txCallback(txContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}



bool UART5_TransmitComplete( void )
{
    bool transmitComplete = false;

    if((U5STA & _U5STA_TRMT_MASK) != 0U)
    {
        transmitComplete = true;
    }

    return transmitComplete;
}