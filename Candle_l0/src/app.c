/*
 * app.c
 *
 *  Created on: 22. 1. 2020
 *  Author:     Priesol Vladimir
 */

#include "app.h"
#include "common_L0.h"
#include "timer.h"
#include "hw.h"

#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rcc.h"

#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

#define APP_CYCLE_DURATION_MS         160
#define APP_MEASURE_BAT_CTRL_MS     (9900 / APP_CYCLE_DURATION_MS)
#define APP_MEASURE_MS             (10000 / APP_CYCLE_DURATION_MS)

#define APP_BATT_MIN_MV             3200
#define APP_OPTO_MIN_MV             1000

#include "data.inc"

//static uint8_t        g_nFrameCtrl = 0;   // 5 bit-Counter

//static uint8_t        g_nPwmValue = 0;    // 4 bit-Register
//static uint8_t        g_nNextBright = 0;  // 4 bit-Register
//static uint8_t        g_nRand = 0;        // 5 bit Signal
//static uint8_t        g_nRandFlag = 0;    // 1 bit Signal

static uint32_t       g_nMeasureIntervalCounter;


static bool           g_bInitialization;    // wait for first ADC conversion


void _FrameControl(void);
void _Sleep(void);

uint32_t _GetTrueRandomNumber(void);

void App_Init(void)
{
#ifdef DEBUG
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif

  HW_Init();
  HW_SetTimCallback(App_TimCallback);

  g_nMeasureIntervalCounter = APP_MEASURE_BAT_CTRL_MS - 1;
  g_bInitialization = true;
}

void App_Exec(void)
{
  // SLEPP mod, nez dobehne PWM cyklus
  _Sleep();

  g_nMeasureIntervalCounter++;
  if (g_nMeasureIntervalCounter == APP_MEASURE_BAT_CTRL_MS)
  {
    HW_BatVoltageCtrl(true);
  }
  else if (g_nMeasureIntervalCounter == APP_MEASURE_MS)
  {
    g_nMeasureIntervalCounter = 0;
    HW_StartAdc();
  }

  if (HW_IsAdcConverted())
  {
//    if (HW_GetBatVoltage() < APP_BATT_MIN_MV || HW_GetOptoVoltage() > APP_OPTO_MIN_MV)
//    {
//      HW_PwmOff();
//      // standby/stop
//      while(1);
//      LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
//    }

    HW_ResetAdcConverted();
    if (g_bInitialization)
    {
      g_bInitialization = false;
      HW_PwmOn();
    }
  }

  LL_IWDG_ReloadCounter(IWDG);
}

void _FrameControl(void)
{
  static uint16_t nDataIndex = 0;        // index to data array
  static uint32_t nNibble = 0;

  uint8_t nValue;
  if (nNibble & 0x01)
  {
    nValue = g_arrData[nDataIndex] & 0x0F;
    nDataIndex++;
    nDataIndex %= sizeof (g_arrData);
  }
  else
  {
    nValue = g_arrData[nDataIndex] >> 4;
  }

  nNibble++;
  HW_PwmSet(nValue);
}

void _Sleep(void)
{
//  APP_SYSTICK_ISR_OFF;
  __WFI();
//  APP_SYSTICK_ISR_ON;
}

void StopMode(void)
{
//  RTC_WriteAccess(true);
  RTC->ISR = RTC_ISR_INIT; // Enable init phase

  // vynulovat time registr
  RTC->TR = 0;
  RTC->CR |= (RTC_CR_ALRAE | RTC_CR_ALRAIE);
  RTC->ISR =~ RTC_ISR_INIT;
//  RTC_WriteAccess(false);

  APP_SYSTICK_ISR_OFF;

  LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

  APP_SYSTICK_ISR_ON;
}

void App_TimCallback(void)
{
  _FrameControl();
}
