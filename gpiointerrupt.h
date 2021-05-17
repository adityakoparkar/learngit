/*
 * gpiointerrupt.h
 
 This is another file to change. Checking the staging part.
 
 *
 *  Created on: Jan 1, 2020
 *      Author: adityakoparkar
 */

#ifndef GPIOINTERRUPT_H_
#define GPIOINTERRUPT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define BOX_ONE     1
#define BOX_TWO     2

#define LOCK            1
#define UNLOCK          0


#define DOOR_OPEN       1
#define DOOR_CLOSE      0

void EnableDoorInterrupt(void);
void EnableShutDownWakeUp(void);
void DebounceTimerInit(void);

#define DOOR1_STATUS      0x01                  // 1: Door Closed     0: Door open
#define DOOR2_STATUS      0x02
#define SHELF_STATUS      0x04


#define PACKAGE1_STATUS         0x01            // 1: Package present 0: package absent
#define PACKAGE2_STATUS         0x02


#define PACKAGE_OCCUPIED        1
#define PACKAGE_UNOCCUPIED      0

void MarkDoorClosed(uint8_t status);
void MarkDoorOpen(uint8_t status);
void UpdateDoorsState(void);
bool IsDoorOpen(uint8_t doorNum);


void SystemShutDown(void);



#endif /* GPIOINTERRUPT_H_ */
