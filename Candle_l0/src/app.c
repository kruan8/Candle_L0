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
#define APP_MEASURE_BAT_CTRL_MS     10000
#define APP_MEASURE_OFFSET_MS         100

#define APP_BATT_MIN_MV             3200
#define APP_OPTO_MIN_MV             1000

#include "data.inc"


static uint32_t       g_nBatCtrlTime;
static uint32_t       g_nMeasureTime;

static bool           g_bInitializated;    // wait for first ADC conversion
static uint8_t        g_FrameCounter;


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

  g_nBatCtrlTime = 0;
  g_bInitializated = false;
}

void App_Exec(void)
{
   // SLEPP mod, wake up from PWM or Systick
  _Sleep();

  if (Timer_GetTicks_ms() >= g_nBatCtrlTime)
  {
    g_nBatCtrlTime = Timer_GetTicks_ms() + APP_MEASURE_BAT_CTRL_MS;
    g_nMeasureTime = Timer_GetTicks_ms() + APP_MEASURE_OFFSET_MS;
    HW_BatVoltageCtrl(true);
  }
  else if (Timer_GetTicks_ms() >= g_nMeasureTime)
  {
    g_nMeasureTime = Timer_GetTicks_ms() + APP_MEASURE_BAT_CTRL_MS;
    HW_StartAdc();
  }

  if (HW_IsAdcConverted())
  {
    if (HW_GetBatVoltage() < APP_BATT_MIN_MV || HW_GetOptoVoltage() > APP_OPTO_MIN_MV)
    {
      HW_PwmOff();
      // standby/stop
      while(1);
      LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
    }

    HW_ResetAdcConverted();
    if (!g_bInitializated)
    {
      g_bInitializated = true;
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

  HW_PwmSet(nValue);
  nNibble++;
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
  g_FrameCounter++;
  g_FrameCounter %= 0xF;
  if (g_FrameCounter == 0)
  {
    _FrameControl();
  }

}
