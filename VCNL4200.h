/*
 * VCNL4200.h
 *
 *  Created on: Jan 7, 2020
 *      Author: adityakoparkar
 */

#ifndef VCNL4200_H_
#define VCNL4200_H_

#include <stdint.h>

//Register declarations
#define VCNL4200_I2CADDR 0x51
#define VCNL4200_ALS_CONF_REG 0x00
#define VCNL4200_ALS_THDH_REG 0x01 //Ambient Light Sensor Threshold Data High
#define VCNL4200_ALS_THDL_REG 0x02 //Ambient Light Sensor Threshold Data Low
#define VCNL4200_PS_CONF1_CONF2_REG 0x03
#define VCNL4200_PS_CONF3_MS_REG 0x04 //Conf3 and Mode Select
#define VCNL4200_PS_CANC_REG 0x05
#define VCNL4200_PS_THDL_REG 0x06 //Proximity Sensor Threshold Data Low
#define VCNL4200_PS_THDH_REG 0x07 //Proximity Sensor Threshold Data High
#define VCNL4200_PROXIMITY_REG 0x08
#define VCNL4200_AMBIENT_REG 0x09
#define VCNL4200_WHITE_REG 0x0A
#define VCNL4200_INT_FLAG_REG 0x0D
#define VCNL4200_DeviceID_REG 0x0E


//Ambient Light Sensor Shut Down
typedef enum {VCNL4200_ALS_Shutdown_on = 0, VCNL4200_ALS_Shutdown_off = 1} VCNL4200_ALS_Shutdown;
//ALS Sensor Interrupt
typedef enum {VCNL4200_ALS_Interrupt_disable = 0, VCNL4200_ALS_Interrupt_enable = 2} VCNL4200_ALS_Interrupt;
//ALS Persistence Setting
typedef enum {VCNL4200_ALS_Pers_one = 0, VCNL4200_ALS_Pers_two = 4, VCNL4200_ALS_Pers_four = 8, VCNL4200_ALS_Pers_eight = 12} VCNL4200_ALS_Pers;
//ALS Interrupt
typedef enum {VCNL4200_ALS_INT_SWITCH_als = 0, VCNL4200_ALS_INT_SWITCH_white = 32} VCNL4200_ALS_INT_SWITCH;
//ALS Integration Time in Milliseconds
typedef enum {VCNL4200_ALS_IT_ms50 = 0, VCNL4200_ALS_IT_ms100 = 64, VCNL4200_ALS_IT_ms200 = 128, VCNL4200_ALS_IT_ms400 = 192} VCNL4200_ALS_IT;

void VerifyVCNL(void);
void ReadProximityReading(uint16_t *pData);
void ProxLowInterrupt();
void ProxHighInterrupt();
void ReadInterruptFlag();
void VCNLInit(void);


#endif /* VCNL4200_H_ */
