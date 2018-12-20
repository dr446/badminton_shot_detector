#include <stdint.h>


#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"

#include "fsl_port_hal.h"
#include "fsl_adc16_driver.h"
#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
#include "fsl_adc16_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"

//#include "devINMP401.h"


//microphone connected to PTB0, which is ADC0_SE9, channel: AD9, 01001

#define ADC_0                   (0U)
#define CHANNEL_0               (9U)

/*Required Procedures: (Obtained from page 416 of reference manual: https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
1) Calibrate ADC
2) Select input clock source and divide ratio, select sample time and low-power config. This is done by updating configuration register, CFG.
3) Select conversion trigger, SC2 reg
4) Select whether conversions are continuous or singular (SC3 reg).
5) enable or disable conversion complete interrupts and select input channel to perform conversions (SC1:SC1n).
*/


enum
{
	kSSD1331PinAO		= GPIO_MAKE_PIN(HW_GPIOB, 0)
};


int32_t init_adc(uint32_t instance)
{
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger enabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.intEnable = true;
    adcUserConfig.resolutionMode = 6U;
    adcUserConfig.hwTriggerEnable = false;
    adcUserConfig.continuousConvEnable = true;
    adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(instance, &adcUserConfig);

    // Install Callback function into ISR
    //ADC_TEST_InstallCallback(instance, CHANNEL_0, ADC1IRQHandler);

    adcChnConfig.chnNum = CHANNEL_0;
    adcChnConfig.diffEnable = false;
    adcChnConfig.intEnable = true;
    adcChnConfig.chnMux = kAdcChnMuxOfA;

    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);

    return 0;
}



int devINMP401init(void)
{
    SEGGER_RTT_printf(0, "init mic reached\n");
   //initialise
   int32_t err = init_adc(ADC_0);
   //set up trigger source
   //init_trigger_source(ADC_0); 
    
   while(1){
      int mic_output = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
      SEGGER_RTT_printf(0, "\rmic output = %d, %d\n", mic_output, err);  
       OSA_TimeDelay(100);
   }
}






