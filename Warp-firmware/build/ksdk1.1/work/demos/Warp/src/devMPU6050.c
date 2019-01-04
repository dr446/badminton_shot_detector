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
#include "fsl_lptmr_driver.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "fsl_hwtimer.h"

#define LPTMR_INSTANCE 0U

extern volatile WarpI2CDeviceState	deviceMPU6050State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

uint16_t acceleration_circular_buffer[20][3];
uint8_t head;
bool buffer_full;

extern uint16_t waveform_buffer[20][3];



#define HWTIMER_LL_DEVIF    kSystickDevif
#define HWTIMER_LL_ID       0

#define HWTIMER_ISR_PRIOR       5
#define HWTIMER_PERIOD          100000
#define HWTIMER_DOTS_PER_LINE   40
#define HWTIMER_LINES_COUNT     2

extern const hwtimer_devif_t kSystickDevif;
extern const hwtimer_devif_t kPitDevif;
hwtimer_t hwtimer;


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

uint16_t get_acc_x()
{
    uint16_t x_acc;
    x_acc = readSensorRegisterMPU6050(0x3B);
    x_acc = x_acc <<8| readSensorRegisterMPU6050(0x3C);
    return x_acc;
}

uint16_t get_acc_y()
{
    uint16_t y_acc;
    y_acc = readSensorRegisterMPU6050(0x3D);
    y_acc = y_acc <<8| readSensorRegisterMPU6050(0x3E);
    return y_acc;
}

uint16_t get_acc_z()
{
    uint16_t z_acc;
    z_acc = readSensorRegisterMPU6050(0x3F);
    z_acc = z_acc <<8| readSensorRegisterMPU6050(0x40);
    return z_acc;
}


//interrupt that reads MPU6050 every 100ms and stores in circular buffer.
void MPU6050_ISR()
{/*
    uint16_t x_acc = acceleration_circular_buffer[head][0] = get_acc_x();
    uint16_t y_acc = acceleration_circular_buffer[head][1] = get_acc_y();
    uint16_t z_acc = acceleration_circular_buffer[head][2] = get_acc_z();
    
    head++;
    
    if(head>19){
        head = 0;    
    }*/
    
   // SEGGER_RTT_printf(0, "\rx_acc= %d y_acc= %d z_acc= %d\n",x_acc, y_acc, z_acc);
    SEGGER_RTT_printf(0, "acc ISR entered! WHoo!\n"); 
}


void update_shot_buffer()
{
    for(int i = 0; i<20; i++)
    {
        waveform_buffer[i][0] =  acceleration_circular_buffer[head][0];
        waveform_buffer[i][1] =  acceleration_circular_buffer[head][1];
        waveform_buffer[i][2] =  acceleration_circular_buffer[head][2];
        head++;
        if(head>20)
        {
            head = 0;
        }
    }
}

void LPTMR0_IRQHandler(void)
{
    LPTMR_DRV_IRQHandler(LPTMR_INSTANCE); 
}


void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{

	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ);
	

   
	return;
	
}

