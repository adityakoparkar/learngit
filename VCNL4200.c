/*
 * VCNL4200.c
 *
 *  Created on: Jan 7, 2020
 *      Author: adityakoparkar
 */
#include <stdint.h>
#include "VCNL4200.h"
#include "i2c_driver.h"


void VerifyVCNL(void)
{
    uint16_t readDeviceId;

    I2CRead16BitReg(VCNL4200_I2CADDR, VCNL4200_DeviceID_REG, (uint8_t *)&readDeviceId );

    uint16_t temp;

    temp = readDeviceId;
}


void VCNLInit(void)
{
    //set_ALS_CONF();
    uint16_t data;
    data = 0x0040;
    I2CWrite16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_ALS_CONF_REG, data);

    //set_PS_CONF1_CONF2();
    //data = 0x092A;              // original value 0x0B2A  : 0x0A2A higher 09 indicative of  trigger by closing instead of trigger by closing and away
    data = 0x0B2A;
    I2CWrite16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_CONF1_CONF2_REG, data);

    //set_PS_CONF3_MS();
    data = 0x2070;      // original 0x0770  : higher 20 byte indicates lower current and INT pin output enable.
    I2CWrite16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_CONF3_MS_REG, data);

    //Interrupt Level
    //write16_LowHigh(VCNL4200_PS_THDL_REG, B10001000, B00010011);
    //data = 0x1388;
    //data = 0x01FF;
    data = 0x007F;   //Smaller the value greater the distance. Make sure THDL and THDH are both same.

    I2CWrite16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_THDL_REG, data);

    //write16_LowHigh(VCNL4200_PS_THDH_REG, B11100000, B00101110);
    //data = 0xE2E0;
    //data = 0x01FF;
    data = 0x007F;  // Smaller the value greater the distance detected.

    I2CWrite16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_THDH_REG, data);
}

void ReadProximityReading(uint16_t * pData)
{

    I2CRead16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PROXIMITY_REG, (uint8_t *)pData );
}


void ProxLowInterrupt()
{
    uint16_t reading;
    I2CRead16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_THDL_REG, (uint8_t *)&reading );
}

void ProxHighInterrupt()
{
    uint16_t reading;
    I2CRead16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_PS_THDH_REG, (uint8_t *)&reading );
}

void ReadInterruptFlag()
{
    uint16_t reading;
    I2CRead16BitRegLowFirst(VCNL4200_I2CADDR, VCNL4200_INT_FLAG_REG, (uint8_t *)&reading );


}

