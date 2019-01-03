
#pragma once


void Microphone_ISR();

void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) );

uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup);

static void ADC16_TEST_IRQHandler(uint32_t instance);


int32_t init_adc(uint32_t instance);

calibrateParams(void);

int devINMP401init(void);

