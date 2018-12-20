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

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceMPU6050State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ);
					
	return;
}


uint8_t readSensorRegisterMPU6050(uint8_t deviceRegister)
{


    uint8_t cmdBuf[1]	= {0xFF};
    uint8_t 	sendBuf[2]	= {0x6B, 0x0};
	i2c_status_t		returnValue;
	
	i2c_device_t slave =
	{
		.address = deviceMPU6050State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};
	
	cmdBuf[0] = deviceRegister;
	uint32_t receive_1;
    
    returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							NULL,
							0,
							sendBuf,
							2,//TODO: for now, we fix command code as two byte 'read firmware version' command
							500 /* timeout in milliseconds */);
    
	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							&receive_1,
							1,
							500 /* timeout in milliseconds */);
	
	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceINA219State.i2cBuffer[0]);
	}
	else
	{
		SEGGER_RTT_printf(0, 0, 0x01, returnValue);

		//return kWarpStatusDeviceCommunicationFailed;
	}
	
	return receive_1;
	
}

void print_accelerations()
{
    
    uint16_t x_acc, y_acc, z_acc;
    int minVal=265; int maxVal=402;
    int x; int y; int z;
    
    x_acc = readSensorRegisterMPU6050(0x3B);
    x_acc = x_acc <<8| readSensorRegisterMPU6050(0x3C);
    y_acc = readSensorRegisterMPU6050(0x3D);
    y_acc = y_acc <<8| readSensorRegisterMPU6050(0x3E);
    z_acc = readSensorRegisterMPU6050(0x3F);
    z_acc = z_acc <<8| readSensorRegisterMPU6050(0x40);
    uint8_t who = readSensorRegisterMPU6050(0x6B);
   
    SEGGER_RTT_printf(0, "\rx_acc= %d y_acc= %d z_acc= %d\n",x_acc, y_acc, z_acc);
        
}



