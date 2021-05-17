
#This is another file 
/*
 * uart_driver.h
 *
 *  Created on: Jan 3, 2020
 
 branch2 in header !!!
 
 *      Author: adityakoparkar
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include <stdint.h>

void WriteToUart(uint8_t * pData, uint8_t dataSize);
void UartDriverInit(void);
void PrintUart(uint8_t *pData);



#endif /* UART_DRIVER_H_ */
