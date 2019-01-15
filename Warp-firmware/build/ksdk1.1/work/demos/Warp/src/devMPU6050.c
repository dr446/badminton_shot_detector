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
#include "devINMP401.h"

#define LPTMR_INSTANCE 0U

extern volatile WarpI2CDeviceState	deviceMPU6050State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

uint16_t acceleration_circular_buffer[10][3];
uint8_t head;
bool buffer_full;

uint16_t waveform_buffer[10][3];
bool read_acceleration_flag;
extern shot_detected_flag;


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
	i2c_status_t		returnValue;
	
	i2c_device_t slave =
	{
		.address = deviceMPU6050State.i2cAddress,
		.baudRate_kbps = 1
	};
	
	cmdBuf[0] = deviceRegister;
	uint32_t receive_1;
    
     uint8_t 	sendBuf[2]	= {0x6B, 0x0};
    returnValue = I2C_DRV_MasterSendDataBlocking(
							0,
							&slave,
							NULL,
							0,
							sendBuf,
							2,
							500);
	OSA_TimeDelay(15);
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
		//SEGGER_RTT_printf(0, "\rgood\n");
	}
	else
	{
		SEGGER_RTT_printf(0, 0, 0x01, returnValue);
        SEGGER_RTT_printf(0, "\rfail\n");
		//return kWarpStatusDeviceCommunicationFailed;
	}
	
	return receive_1;
	
}

void print_accelerations()
{
    
    uint16_t x_high_acc, y_high_acc, z_high_acc;
    uint16_t x_low_acc, y_low_acc, z_low_acc;
    uint16_t x_acc, y_acc, z_acc;
    uint16_t minVal=265; uint16_t maxVal=402;
    
    x_high_acc = readSensorRegisterMPU6050(0x3B);//b
    OSA_TimeDelay(15);
    x_low_acc = readSensorRegisterMPU6050(0x3C);
    OSA_TimeDelay(15);
    y_high_acc = readSensorRegisterMPU6050(0x3D);//d
    OSA_TimeDelay(15);
    y_low_acc = readSensorRegisterMPU6050(0x3E);//e
    OSA_TimeDelay(15);
    z_high_acc = readSensorRegisterMPU6050(0x3F);//f
    OSA_TimeDelay(15);
    z_low_acc = readSensorRegisterMPU6050(0x40);//40
    
    x_acc = ((x_high_acc << 8) & 0xFF00) | (x_low_acc & 0xFF);
    y_acc = ((y_high_acc << 8) & 0xFF00) | (y_low_acc & 0xFF);
    z_acc = ((z_high_acc << 8) & 0xFF00) | (z_low_acc & 0xFF);
       
    SEGGER_RTT_printf(0, "\rx_acc= %d y_acc= %d z_acc= %d\n", x_acc, y_high_acc, z_high_acc);
        
}

uint16_t get_acc_x()
{
    uint16_t x_acc;
    OSA_TimeDelay(15);
    x_acc = readSensorRegisterMPU6050(0x3B);
    OSA_TimeDelay(15);
    x_acc = x_acc <<8| readSensorRegisterMPU6050(0x3C);
    return x_acc;
}

uint16_t get_acc_y()
{
    uint16_t y_acc;
    OSA_TimeDelay(15);
    y_acc = readSensorRegisterMPU6050(0x3D);
    OSA_TimeDelay(15);
    y_acc = y_acc <<8| readSensorRegisterMPU6050(0x3E);
    return y_acc;
}

uint16_t get_acc_z()
{
    uint16_t z_acc;
    OSA_TimeDelay(15);
    z_acc = readSensorRegisterMPU6050(0x3F);
    OSA_TimeDelay(15);
    z_acc = z_acc <<8| readSensorRegisterMPU6050(0x40);
    return z_acc;
}




void update_circular_buffer()
{

    acceleration_circular_buffer[head][0] = get_acc_x();
    acceleration_circular_buffer[head][1] = get_acc_y();
    acceleration_circular_buffer[head][2] = get_acc_z();
   //SEGGER_RTT_printf(0, "\rx_acc= %d y_acc= %d z_acc= %d\n",acceleration_circular_buffer[head][0],acceleration_circular_buffer[head][1], acceleration_circular_buffer[head][2]);
    head++;
    
    if(head>9){
        head = 0;    
    }

    read_acceleration_flag = false;
    
}



void update_shot_buffer()
{
    for(int i = 0; i<10; i++)
    {
        waveform_buffer[i][0] =  acceleration_circular_buffer[head][0];
        waveform_buffer[i][1] =  acceleration_circular_buffer[head][1];
        waveform_buffer[i][2] =  acceleration_circular_buffer[head][2];
        
      //  SEGGER_RTT_printf(0, "\rx_acc= %d y_acc= %d z_acc= %d\n",waveform_buffer[i][0],waveform_buffer[i][1], waveform_buffer[i][2]);
        
        head++;
        if(head>9)
        {
            head = 0;
        }
    }
}



void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{

	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ);
	
    uint8_t 	sendBuf[2]	= {0x6B, 0x0};
	i2c_status_t		returnValue;
		
	i2c_device_t slave =
	{
		.address = deviceMPU6050State.i2cAddress,
		.baudRate_kbps = 1
	};
	
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0,
							&slave,
							NULL,
							0,
							sendBuf,
							2,
							500);
   
	return;
	
}

