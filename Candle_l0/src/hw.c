/*
 * hw.c
 *
 *  Created on: 22. 1. 2020
 *  Author:     Priesol Vladimir
 */

#include "hw.h"
#include "common_L0.h"
#include "timer.h"

#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_iwdg.h"

#define LED                 PA1

#define LED_ON              (GET_PORT(LED)->BRR = GET_PIN(LED))
#define LED_OFF             (GET_PORT(LED)->BSRR = GET_PIN(LED))

#define HW_ADC_VOLTAGE_MV          3000
#define HW_ADC_RESISTOR_DIVIDER_OPTO    1000/327
#define HW_ADC_RESISTOR_DIVIDER_BATT    1000/327

#define HW_ADC_SAMPLES      10
#define HW_BATT_CH          LL_ADC_CHANNEL_4
#define BATT_CTRL           PA7

#define BATT_CTRL_EN        (GET_PORT(BATT_CTRL)->BRR = GET_PIN(BATT_CTRL))
#define BATT_CTRL_DIS       (GET_PORT(BATT_CTRL)->BSRR = GET_PIN(BATT_CTRL))

#define HW_OPTO_CH          LL_ADC_CHANNEL_0

#define TIM_PWM             TIM2  // (PA1 = TIM2/CH2)
#define PWM_STEPS           16       // pocet kroku PWM

typedef enum
{
  hw_adc_opto = 0,
  hw_adc_batt,
//  hw_adc_ref,
  hw_adc_sizeof,
} hw_adc_channel_e;

static uint32_t                    g_nActualAdcChannel;          // converted channel
static uint8_t                     g_nAdcValues[hw_adc_sizeof];   //
static bool                        g_bAdcConverted;

static PtrTimIntCb                 g_pTimCb = 0;                 // callback from TIM_PWM interrupt
static uint32_t                    g_nBatVoltage_mV = 4000;      // battery voltage (mV)
static uint32_t                    g_nOptoVoltage_mV = 3300;     // opto transistor voltage (mV)


void _AD_Init(void);
void _Gpio_Init(void);
void _PwmInit(void);
void _CalculateAdcVoltage(void);

void HW_Init(void)
{
  // Change MSI frequency to 1 MHz
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_4);   // 1 MHz

  // set voltage range 3
  while (LL_PWR_IsActiveFlag_VOS());
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  while (LL_PWR_IsActiveFlag_VOS());

//  Timer_Init();
  _Gpio_Init();
  _AD_Init();
  _PwmInit();

  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);
  LL_IWDG_Enable(IWDG);
  LL_IWDG_SetReloadCounter(IWDG, 0xFFF);  // cca 32s

  while(!LL_IWDG_IsReady(IWDG))
  {
  /* add time out here for a robust application */
  }

  LL_IWDG_ReloadCounter(IWDG);

}

void _AD_Init(void)
{
  // Configure ADC INPUT pins as analog input
   // asi neni treba konfigurovat, po resetu jsou vstupy v analog input
   RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
   //  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | GPIO_MODER_MODE4;

   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

   // ADC clock PCLK/2 (Synchronous clock mode) (ADC clock = 1MHz)
   LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);

   LL_ADC_SetCommonFrequencyMode(ADC1_COMMON, LL_ADC_CLOCK_FREQ_MODE_LOW);
   LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_8B);
   LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_AUTOPOWEROFF);

   // set oversampling, ! bity CKMODE registru CFGR2 musi byt nastaveny pred jakymkoliv nastaveni ADC - viz datasheet !
   LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_RIGHT_4);
   LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);

   LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_160CYCLES_5);

//   // Calibrate ADC
//   LL_ADC_Disable(ADC1);
//   LL_ADC_StartCalibration(ADC1);
//   while (LL_ADC_IsCalibrationOnGoing(ADC1))
//   {
//     /* For robust implementation, add here time-out management */
//   }
//
//   LL_ADC_ClearFlag_EOC(ADC1);   // clear calibration flag


//   // Calibrate ADC DataLogger
//   if ((ADC1->CR & ADC_CR_ADEN) != 0) // Ensure that ADEN = 0
//   {
//     ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  // Clear ADEN
//   }
//
//   ADC1->CR |= ADC_CR_ADCAL; // Set ADCAL=1
//   while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) // Clear EOCAL
//   {
//     /* For robust implementation, add here time-out management */
//   }
//

   LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);
   LL_ADC_REG_SetSequencerChannels(ADC1, HW_OPTO_CH | HW_BATT_CH);

   NVIC_SetPriority(ADC1_COMP_IRQn, 2);
   NVIC_EnableIRQ(ADC1_COMP_IRQn);

   LL_ADC_EnableIT_EOC(ADC1);

   LL_ADC_Enable(ADC1);
}

void HW_StartAdc(void)
{
  g_nActualAdcChannel = hw_adc_opto;
  LL_ADC_REG_StartConversion(ADC1);
}

void _Gpio_Init(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_SetPinMode(GET_PORT(LED), GET_PIN(LED), LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(GET_PORT(LED), GET_PIN(LED), LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GET_PORT(LED), GET_PIN(LED), LL_GPIO_PULL_NO);
  GPIO_SetAFpin(LED, LL_GPIO_AF_2);
}

uint32_t HW_GetTrueRandomNumber(void)
{
  //enable ADC1 clock
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  // Enable ADCperipheral
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsEnabled(ADC1));

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  for (uint8_t i = 0; i < 8; i++)
  {
    //Start ADC1 Software Conversion
    LL_ADC_REG_StartConversion(ADC1);

    //wait for conversion complete
    while (!LL_ADC_IsActiveFlag_EOC(ADC1));

    CRC->DR = LL_ADC_REG_ReadConversionData12(ADC1);
  }

  LL_ADC_Disable(ADC1);

  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);

  CRC->DR = 0xBADA55E5;
  uint32_t nValue = CRC->DR;

  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CRC);

  return nValue;
}

void _PwmInit(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_RCC_ClocksTypeDef  RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  LL_TIM_SetPrescaler(TIM_PWM, RCC_Clocks.PCLK1_Frequency / 6600);  // frekvence PWM 6600 Hz
  LL_TIM_SetAutoReload(TIM_PWM, PWM_STEPS);

  LL_TIM_OC_SetMode(TIM_PWM, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_EnablePreload(TIM_PWM, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetPolarity(TIM_PWM, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_LOW);
  LL_TIM_CC_EnableChannel(TIM_PWM, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableCounter(TIM_PWM);
  LL_TIM_GenerateEvent_UPDATE(TIM_PWM);

  LL_TIM_EnableIT_UPDATE(TIM_PWM);

  // povolit preruseni od TIM3
  NVIC_SetPriority(TIM2_IRQn, 1);    // Set priority
  NVIC_EnableIRQ(TIM2_IRQn);         // Enable _IRQn

#ifdef DEBUG
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_TIM2_STOP);
#endif
}

void HW_PwmSet(uint16_t nValue)
{
  LL_TIM_OC_SetCompareCH2(TIM_PWM, nValue);
}

void HW_LedOnOff(bool bEnable)
{
  bEnable ? LED_ON : LED_OFF;
}

void HW_BatVoltageCtrl(bool bEnable)
{
  bEnable ? BATT_CTRL_EN : BATT_CTRL_DIS;
}

void HW_SetTimCallback(PtrTimIntCb pTimCb)
{
  g_pTimCb = pTimCb;
}

uint32_t HW_GetBatVoltage(void)
{
  return g_nBatVoltage_mV;
}

uint32_t HW_GetOptoVoltage(void)
{
  return g_nOptoVoltage_mV;
}

void _CalculateAdcVoltage(void)
{
  // calculate opto voltage
  g_nOptoVoltage_mV = (uint32_t)g_nAdcValues[hw_adc_opto] * HW_ADC_VOLTAGE_MV / 256;
  g_nOptoVoltage_mV *= HW_ADC_RESISTOR_DIVIDER_OPTO;

  // calculate batt voltage
  g_nBatVoltage_mV = (uint32_t)g_nAdcValues[hw_adc_batt] * HW_ADC_VOLTAGE_MV / 256;
  g_nBatVoltage_mV *= HW_ADC_RESISTOR_DIVIDER_BATT;
}

bool HW_IsAdcConverted(void)
{
  return g_bAdcConverted;
}

void HW_ResetAdcConverted(void)
{
  g_bAdcConverted = false;
}

void ADC1_COMP_IRQHandler(void)
{
  g_nAdcValues[g_nActualAdcChannel] = LL_ADC_REG_ReadConversionData8(ADC1);
  g_nActualAdcChannel++;
  if (g_nActualAdcChannel == hw_adc_sizeof)
  {
    g_bAdcConverted = true;
    HW_BatVoltageCtrl(false);
    _CalculateAdcVoltage();
  }
}

void TIM2_IRQHandler(void)
{
  if (!(TIM_PWM->SR & TIM_SR_UIF))
  {
    return;
  }

  TIM_PWM->SR &= ~TIM_SR_UIF;

  if (g_pTimCb)
  {
    g_pTimCb();
  }

}
