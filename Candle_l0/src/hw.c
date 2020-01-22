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

#define LED                 PA1

#define LED_ON              (GET_PORT(LED)->BRR = GET_PIN(LED))
#define LED_OFF             (GET_PORT(LED)->BSRR = GET_PIN(LED))

#define BATT_IN             PA4
#define BATT_CTRL           PA7

#define BATT_CTRL_EN        (GET_PORT(BATT_CTRL)->BRR = GET_PIN(BATT_CTRL))
#define BATT_CTRL_DIS       (GET_PORT(BATT_CTRL)->BSRR = GET_PIN(BATT_CTRL))

#define TIM_PWM              TIM2  // (PA1 = TIM2/CH2)
#define PWM_STEPS            16       // pocet kroku PWM

PtrTimIntCb           g_pTimCb = 0;         // callback from TIM_PWM interrupt

void _AD_Init(void);
void _Gpio_Init(void);
void _PwmInit(void);


void HW_Init(void)
{
  Timer_Init();
  _Gpio_Init();
  _AD_Init();
}

void HW_LedOnOff(bool bEnable)
{
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

 #ifdef OVERSAMPLING
   // set oversampling, ! bity CKMODE registru CFGR2 musi byt nastaveny pred jakymkoliv nastaveni ADC - viz datasheet !
   LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_RIGHT_4);
   LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
 #endif

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
   LL_ADC_Enable(ADC1);
}

void _Gpio_Init(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

#ifdef HW
  LL_GPIO_SetPinMode(GET_PORT(LED), GET_PIN(LED), LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(GET_PORT(LED), GET_PIN(LED), LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GET_PORT(LED), GET_PIN(LED), LL_GPIO_PULL_NO);
  GPIO_SetAFpin(LED, LL_GPIO_AF_6);
#else
  LL_GPIO_SetPinMode(GET_PORT(LED), GET_PIN(LED), LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GET_PORT(LED), GET_PIN(LED), LL_GPIO_OUTPUT_PUSHPULL);
#endif
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

  // nastavit 6kHz
  TIM_PWM->PSC = RCC_Clocks.PCLK1_Frequency / 5000;  // cca 6 kHz
  TIM_PWM->ARR = PWM_STEPS;

  /* (3) Set CCRx = 4, , the signal will be high during 4 us */
  /* (4) Select PWM mode 1 on OC1 (OC1M = 110),
  enable preload register on OC1 (OC1PE = 1) */
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
  enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Enable counter (CEN = 1)
  select edge aligned mode (CMS = 00, reset value)
  select direction as upcounter (DIR = 0, reset value) */
  /* (8) Force update generation (UG = 1) */
  TIM_PWM->CCR2 = 0; /* (3)  PWM value */
  TIM_PWM->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* (4) */ // PWM mode1 + Preload register on TIMx_CCR4 enabled.
  TIM_PWM->CCER |= TIM_CCER_CC2E; /* (5) */
  TIM_PWM->CR1 |= TIM_CR1_CEN;
  TIM_PWM->EGR |= TIM_EGR_UG; /* (8) */

  TIM_PWM->DIER |= TIM_DIER_UIE;

  // povolit preruseni od TIM3
  NVIC_SetPriority(TIM2_IRQn, 1);    // Set priority
  NVIC_EnableIRQ(TIM2_IRQn);         // Enable _IRQn

#ifdef DEBUG
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_TIM2_STOP);
#endif
}

void HW_PwmSet(uint16_t nValue)
{
  TIM_PWM->CCR4 = nValue;
}

void HW_SetTimCallback(PtrTimIntCb pTimCb)
{
  g_pTimCb = pTimCb;
}

void TIM3_IRQHandler(void)
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
