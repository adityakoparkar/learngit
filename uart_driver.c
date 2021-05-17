/*
 * uart_driver.c
 *
 *  Created on: Jan 3, 2020
 
 Oh this is another change but to branch2.
 haha.
 
 *      Author: adityakoparkar
 */

#include "Board.h"
#include <ti/drivers/UART.h>

#include "uart_driver.h"
#include <string.h>

//UART
UART_Handle uart;
UART_Params uartParams;


void UartDriverInit(void) {


    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


//Example of calling UART
//char arr1[] = "BeforePOwer";
//UART_write(uart, arr1, sizeof(arr1));

void WriteToUart(uint8_t * pData, uint8_t dataSize)
{

    UART_write(uart, pData, dataSize);
}


void PrintUart(uint8_t *pData)
{
    uint8_t size = strlen((char *)pData);

    WriteToUart(pData,size);

}

