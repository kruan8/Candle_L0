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


#define APP_MEASURE_BAT_CTRL_MS     9900
#define APP_MEASURE_MS             10000

#define APP_BATT_MIN_MV             3200
#define APP_OPTO_MIN_MV             1000

#include "data.inc"

static uint8_t        g_nFrameCtrl = 0;   // 5 bit-Counter

//static uint8_t        g_nPwmValue = 0;    // 4 bit-Register
//static uint8_t        g_nNextBright = 0;  // 4 bit-Register
//static uint8_t        g_nRand = 0;        // 5 bit Signal
//static uint8_t        g_nRandFlag = 0;    // 1 bit Signal

static uint32_t       g_nModeCounter;

static uint16_t       g_nDataCounter;


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

  g_nDataCounter = 0;
  g_nModeCounter = APP_MEASURE_BAT_CTRL_MS - 1;
}

void App_Exec(void)
{

  // WWDG reset?
//  if (LL_RCC_IsActiveFlag_WWDGRST())
//  {
//
//  }

  // SLEPP mod, nez dobehne PWM cyklus
  while (1)
  {
    _Sleep();
    g_nModeCounter++;
    if (g_nModeCounter == APP_MEASURE_BAT_CTRL_MS)
    {
      HW_BatVoltageCtrl(true);
    }
    else if (g_nModeCounter == APP_MEASURE_MS)
    {
      g_nModeCounter = 0;
      HW_StartAdc();
    }

    LL_IWDG_ReloadCounter(IWDG);

    if (HW_IsAdcConverted())
    {
      if (HW_GetBatVoltage() < APP_BATT_MIN_MV)
      {
        // standby/stop
        while(1);
        LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
      }

      if (HW_GetOptoVoltage() < APP_OPTO_MIN_MV)
      {
        // standby/stop
        while(1);
        LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
      }

      HW_ResetAdcConverted();
    }

  }
}

void _FrameControl(void)
{
  g_nFrameCtrl++;
  g_nFrameCtrl &= 0x1f;

  // NEW FRAME
  if (g_nFrameCtrl == 0)
  {
    uint8_t nValue;
    if (g_nDataCounter & 0x01)
    {
      nValue = g_arrData[g_nDataCounter >> 1] & 0x0F;
    }
    else
    {
      nValue = g_arrData[g_nDataCounter >> 1] >> 4;
    }

    g_nDataCounter++;
    if (g_nDataCounter == sizeof (g_arrData) * 2)
    {
      g_nDataCounter = 0;
    }

    HW_PwmSet(nValue);
  }
}

void _Sleep(void)
{
  APP_SYSTICK_ISR_OFF;
  __WFI();
  APP_SYSTICK_ISR_ON;
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
