/*
 
 Changes for test branch.
 
 This is fun.
 
 and this is second change to stage. Lets see how this works.

 
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>

/* Example/Board Header files */
#include "Board.h"

#include <ti/drivers/timer/GPTimerCC26XX.h>
#include "CC1310_LAUNCHXL.h"
#include "gpiointerrupt.h"

//GLOBAL VARIABLES

volatile uint8_t doorStatus;        // 1 : Open. 0 : Close
volatile uint8_t interruptedPin = CC1310_LAUNCHXL_GPIOCOUNT;            // The count is invalid value
volatile uint8_t packageStatus;


GPTimerCC26XX_Handle hTimer1A;
GPTimerCC26XX_Params paramsTimer1A;

/* Wake-up Button pin table */
PIN_Config ButtonTableWakeUp[] = {
    IOID_15 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    IOID_17 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE                                 /* Terminate list */
};

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON1.
 *  This may not be used for all boards.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    interruptedPin = index;
    GPTimerCC26XX_start(hTimer1A);
}


void gpioTOFIntHandler(uint_least8_t index)
{
    switch(index)
    {
        case GPIO_TOF1:
            if(GPIO_read(GPIO_TOF1) == 1)
            {
                //packet placed
                packageStatus |= PACKAGE1_STATUS;
            }
            else
            {
                //packet removed
                packageStatus &= ~PACKAGE1_STATUS;
            }
            break;

        case GPIO_TOF2:
            if(GPIO_read(GPIO_TOF1) == 1)
            {
                //packet placed
                packageStatus |= PACKAGE2_STATUS;
            }
            else
            {
                //packet removed
                packageStatus &= ~PACKAGE2_STATUS;
            }
            break;

        default:
            break;
    }
}


void timerCallback1A(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
//    // interrupt callback code goes here. Minimize processing in interrupt.
//    if(GPIO_read(DOOR1) == 0 && interruptedPin == GPIO_DOOR_REED1)
//    {
//        //proximitySensorStatus |= DOOR1_STATUS;      // closed status
//        MarkDoorClosed(DOOR1_STATUS);
//        reed1 = 1;
//        interruptedPin = 0;
//    }

    switch(interruptedPin)
    {
        case GPIO_LOCK_DETECT_CLOSE_1:
            if(GPIO_read(GPIO_LOCK_DETECT_CLOSE_1) == 0)
            {
                GPIO_write(GPIO_LOCK_UNLOCK_1,1);             // LOCK THE DOOR by writing HIGH to LOCK_UNLOCK
                MarkDoorClosed(DOOR1_STATUS);
                interruptedPin = CC1310_LAUNCHXL_GPIOCOUNT;                 // The count is invalid value so we are reseting the pin here.
            }
            break;
        case GPIO_LOCK_DETECT_OPEN_1:
            break;
        case GPIO_LOCK_DETECT_CLOSE_2:
            if(GPIO_read(GPIO_LOCK_DETECT_CLOSE_1) == 0)
            {
                GPIO_write(GPIO_LOCK_UNLOCK_2,1);           // LOCK THE DOOR by writing HIGH to LOCK_UNLOCK
                MarkDoorClosed(DOOR2_STATUS);
                interruptedPin = CC1310_LAUNCHXL_GPIOCOUNT;                 // The count is invalid value so we are reseting the pin here.
            }
            break;
        case GPIO_LOCK_DETECT_OPEN_2:
            break;

        default:
            break;

    }
}

void DebounceTimerInit(void)
{
    GPTimerCC26XX_Params_init(&paramsTimer1A);
    paramsTimer1A.width          = GPT_CONFIG_32BIT;
    paramsTimer1A.mode           = GPT_MODE_ONESHOT;
    paramsTimer1A.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer1A = GPTimerCC26XX_open(CC1310_LAUNCHXL_GPTIMER1A, &paramsTimer1A);
    if(hTimer1A == NULL) {
       while(1);
    }


    GPTimerCC26XX_Value loadVal = 719999; //15 mS delay for button debounce //47999 is for 1mS. So (48000 * 15) - 1 for 15 mS.
    GPTimerCC26XX_setLoadValue(hTimer1A, loadVal);
    GPTimerCC26XX_registerInterrupt(hTimer1A, timerCallback1A, GPT_INT_TIMEOUT);

}

void EnableShutDownWakeUp(void)
{
    PINCC26XX_setWakeup(ButtonTableWakeUp);
}

/*
 *  ======== mainThread ========
 */
void EnableDoorInterrupt(void)
{

    GPIO_setConfig(GPIO_LOCK_DETECT_CLOSE_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(GPIO_LOCK_DETECT_OPEN_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(GPIO_LOCK_DETECT_CLOSE_2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(GPIO_LOCK_DETECT_OPEN_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(GPIO_TOF1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_BOTH_EDGES);
    GPIO_setConfig(GPIO_TOF2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_BOTH_EDGES);


    /* install Button callback */
    GPIO_setCallback(GPIO_LOCK_DETECT_CLOSE_1, gpioButtonFxn1);
    GPIO_setCallback(GPIO_LOCK_DETECT_OPEN_1, gpioButtonFxn1);
    GPIO_setCallback(GPIO_LOCK_DETECT_CLOSE_2, gpioButtonFxn1);
    GPIO_setCallback(GPIO_LOCK_DETECT_OPEN_1, gpioButtonFxn1);
    GPIO_setCallback(GPIO_TOF1, gpioTOFIntHandler);
    GPIO_setCallback(GPIO_TOF2, gpioTOFIntHandler);


    /* Enable interrupts */
    GPIO_enableInt(GPIO_LOCK_DETECT_CLOSE_1);
    GPIO_enableInt(GPIO_LOCK_DETECT_OPEN_1);
    GPIO_enableInt(GPIO_LOCK_DETECT_CLOSE_2);
    GPIO_enableInt(GPIO_LOCK_DETECT_OPEN_2);
    GPIO_enableInt(GPIO_TOF1);
    GPIO_enableInt(GPIO_TOF2);

    DebounceTimerInit();

}


//Aggressive Power saving mode.
void SystemShutDown(void)
{
    EnableShutDownWakeUp();
    Power_shutdown(0,0);
}

//Less Aggressive mode. The interrupts can wake the system.
void SystemSleep(void)
{
    Power_idleFunc();
}

bool IsPackageInside(uint8_t boxNum)
{
    bool result = false;

    switch (boxNum)
    {
        case BOX_ONE:
            if(GPIO_read(GPIO_TOF1) == 1)
                result = true;
            else
                result = false;
            break;

        case BOX_TWO:
            if(GPIO_read(GPIO_TOF2) == 1)
                result = true;
            else
                result = false;
            break;

        default:
            break;
    }

    return result;
}

void MarkDoorClosed(uint8_t status)
{
    doorStatus |= status;
}

void MarkDoorOpen(uint8_t status)
{
    doorStatus &= ~status;
}

bool IsDoorOpen(uint8_t boxNum)
{
    bool result = false;
    switch(boxNum)
    {
        case BOX_ONE:
            if(GPIO_read(GPIO_LOCK_DETECT_OPEN_1) == 0)
            {
                result = true;
            }
            else
            {
                result = false;
            }
        break;

        case BOX_TWO:
            if(GPIO_read(GPIO_LOCK_DETECT_OPEN_2) == 0)
            {
                result = true;
            }
            else
            {
                result = false;
            }
        break;

//        case SHELF:
//            if(proximitySensorStatus & SHELF_STATUS)
//            {
//                result = true;
//            }
//
//        break;

        default:
        break;


    }

    return result;
}

//This function is only required during the start when the interrupts might not
// have triggered. So during the startup we actually call this function where
// the GPIOs are read individually.

void UpdateDoorsState(void)
{
    doorStatus |= ((GPIO_read(GPIO_LOCK_DETECT_OPEN_1) == 0) ? DOOR1_STATUS : 0);
    doorStatus |= ((GPIO_read(GPIO_LOCK_DETECT_OPEN_2) == 0) ? DOOR1_STATUS : 0);
}


void UpdatePackageState(void)
{
    packageStatus |= ((GPIO_read(GPIO_TOF1) == 1) ? PACKAGE1_STATUS : 0);
    packageStatus |= ((GPIO_read(GPIO_TOF2) == 1) ? PACKAGE2_STATUS : 0);

}
