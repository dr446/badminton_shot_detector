#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
//#include "devINA219.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	
	deviceStatePointer->i2cAddress	= i2cAddress;
	//deviceStatePointer->signalType	= (	kWarpTypeMaskHumidity);
					
	return;
}



uint16_t readSensorRegisterINA219(uint8_t deviceRegister)
{
    
    uint8_t cmdBuf[1]	= {0xFF};
	i2c_status_t returnValue;
    
    i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};
    
    cmdBuf[0]	= deviceRegister;
    uint16_t receive_1;
    uint16_t receive_2;
    uint16_t receive_3;
    uint16_t receive_buf;
    
    returnValue = I2C_DRV_MasterReceiveDataBlocking(0,
                                                    &slave,
                                                    cmdBuf,
                                                    1,
                                                    &receive_1,
                                                    2,
                                                    1000);
                                              
    receive_2 = receive_1>>8;
    receive_3 = receive_1<<8;
                                              
    receive_buf = receive_2 | receive_3;
                                           
    //SEGGER_RTT_printf(0,"i2c should have completed\n");
    if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceINA219State.i2cBuffer[0]);
	}
	else
	{
		SEGGER_RTT_printf(0, 0, 0x01, returnValue);

		//return kWarpStatusDeviceCommunicationFailed;
	}
    
    return receive_buf;
    
}

int16_t measure_current_INA219()
{
    
    int16_t config, shunt, bus, calibration, current, power;
    
    //read config reg
    config = readSensorRegisterINA219(0x00);
    //read shunt reg
    shunt = readSensorRegisterINA219(0x01);
    //read bus reg
    bus = readSensorRegisterINA219(0x02);
    //read calibration reg
    calibration = readSensorRegisterINA219(0x05);
    //read current reg
    current = readSensorRegisterINA219(0x04);
    //read power reg
    power = readSensorRegisterINA219(0x03);
    
    //SEGGER_RTT_printf(0, "\rconfig [0x%04x]\n", config);
    SEGGER_RTT_printf(0, "\rshunt %d\n", shunt);
    /*SEGGER_RTT_printf(0, "\rbus [0x%04x]\n", bus);
    SEGGER_RTT_printf(0, "\rcalibration [0x%04x]\n", calibration);
    SEGGER_RTT_printf(0, "\rcurrent [0x%04x]\n", current);*/
    
    float shunt_float = (float)shunt;
    
    int16_t load_current = shunt*320/(4096*0.1*8);
    //SEGGER_RTT_printf(0, "\rcalc [0x%04x]\n", load_current);
    
    float load_current_float= shunt_float*320/(4096*0.1*8);
    
    
    return load_current;

}


