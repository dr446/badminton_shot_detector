
#include <stdint.h>


#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"

#include "fsl_port_hal.h"
#include "fsl_adc16_driver.h"
#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
#include "fsl_adc16_hal.h"
#include "fsl_lptmr_driver.h"


#include "SEGGER_RTT.h"
#include "gpio_pins.h"

//#include "devINMP401.h"


//microphone connected to PTB0, which is ADC0_SE9, channel: AD9, 01001

#define ADC_0                   (0U)
#define CHANNEL_0               (0U)
#define CHANNEL_MIC             (9U)

//#define LPTMR_COMPARE_VALUE     (500000U)   // Low Power Timer interrupt time in microseconds

const uint32_t gSimBaseAddr[] = SIM_BASE_ADDRS;
static smc_power_mode_config_t smcConfig;
//static lptmr_state_t gLPTMRState;





bool shot_detected_flag;



void init_trigger_source(uint32_t adcInstance)
{
    SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBaseAddr[0], adcInstance, true);

}


void Microphone_ISR()
{
    shot_detected_flag = true; 
}



// Define array to keep run-time callback set by application
void (* volatile g_AdcTestCallback[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT])(void);
volatile uint16_t g_AdcValueInt[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT];

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/* User-defined function to install callback. */
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) )
{
    g_AdcTestCallback[instance][chnGroup] = callbackFunc;
}

/* User-defined function to read conversion value in ADC ISR. */
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup)
{
    return g_AdcValueInt[instance][chnGroup];
}



static void ADC16_TEST_IRQHandler(uint32_t instance)
{
    uint32_t chnGroup;
    for (chnGroup = 0U; chnGroup < HW_ADC_SC1n_COUNT; chnGroup++)
    {
        if (   ADC16_DRV_GetChnFlag(instance, chnGroup, kAdcChnConvCompleteFlag) )
        {
            g_AdcValueInt[instance][chnGroup] = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
            if ( g_AdcTestCallback[instance][chnGroup] )
            {
                (void)(*(g_AdcTestCallback[instance][chnGroup]))();
            }
        }
    }
}

/* ADC IRQ handler that would cover the same name's APIs in startup code */
void ADC0_IRQHandler(void)
{
    // Add user-defined ISR for ADC0
    ADC16_TEST_IRQHandler(0U);
}




int32_t init_adc(uint32_t instance)
{

    adc16_calibration_param_t auto_params;
    
    
    ADC16_DRV_GetAutoCalibrationParam(instance, &auto_params);
    
    SEGGER_RTT_printf(0, "\rautores = %d\n", auto_params.plusSideGainValue);  
    
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
    adcUserConfig.resolutionMode = 0U;
    adcUserConfig.hwTriggerEnable = false;
    adcUserConfig.continuousConvEnable = true;
    adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(instance, &adcUserConfig);

    // Install Callback function into ISR
    adc16_hw_cmp_config_t hardware_compare;
    
    hardware_compare.cmpValue1 =100;
    hardware_compare.cmpValue2 = 145;
    hardware_compare.cmpRangeMode = 2;
    
    ADC16_DRV_EnableHwCmp(ADC_0, &hardware_compare);
    
    
    //ADC16_HAL_SetHwTriggerCmd(ADC_0, 1);
    
    // Install Callback function into ISR
    ADC_TEST_InstallCallback(ADC_0, CHANNEL_0, Microphone_ISR);
    
    
    adcChnConfig.chnNum = CHANNEL_MIC;
    adcChnConfig.diffEnable = true;
    adcChnConfig.intEnable = true;
    adcChnConfig.chnMux = kAdcChnMuxOfA;
    
    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);
    
    
    
    init_trigger_source(ADC_0);
    
    return 0;
}





int devINMP401init(void)
{
   
   //initialise
   
   int32_t err = init_adc(ADC_0);
    
    uint8_t flag = ADC16_DRV_GetChnFlag(ADC_0, CHANNEL_MIC, 0);

}






