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

#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

uint8_t g_nFrameCtrl = 0;   // 5 bit-Counter

uint8_t g_nPwmValue = 0;    // 4 bit-Register
uint8_t g_nNextBright = 0;  // 4 bit-Register
uint8_t g_nRand = 0;        // 5 bit Signal
uint8_t g_nRandFlag = 0;    // 1 bit Signal


void _FrameControl(void);

uint32_t _GetTrueRandomNumber(void);

void App_Init(void)
{
#ifdef DEBUG
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif

  HW_Init();
  HW_SetTimCallback(App_TimCallback);

#ifdef HW
  App_PwmInit(nHCLKFrequency);
#endif
}

void App_Exec(void)
{

#ifdef HW
  // SLEPP mod, nez dobehne PWM cyklus
  while (1)
  {
    SleepMode();
  }
#endif

  uint8_t nPwmCtrl = 0;   // 4 bit-Counter

  while(1)
  {
    TimerUs_delay(150);
//     StopMode();

    // PWM led
    nPwmCtrl++;
    nPwmCtrl &= 0xf;    // only 4 bit
    if (nPwmCtrl <= g_nPwmValue)
    {
      HW_LedOnOff(true);
    }
    else
    {
      HW_LedOnOff(false);
    }

    // FRAME
    if (nPwmCtrl == 0)
    {
      _FrameControl();
    }
  }

}

void _FrameControl(void)
{
  g_nFrameCtrl++;
  g_nFrameCtrl &= 0x1f;

  // generate a new random number every 8 cycles. In reality this is most likely bit serial
  if ((g_nFrameCtrl & 0x07) == 0)
  {
    g_nRand = HW_GetTrueRandomNumber() & 0x1f;
    if ((g_nRand & 0x0c) != 0)
    {
      g_nRandFlag = 1;
    }
    else
    {
      g_nRandFlag = 0; // only update if valid
    }
  }

  // NEW FRAME
  if (g_nFrameCtrl == 0)
  {
    // reload PWM
    g_nPwmValue = g_nNextBright;

    // force update at beginning of frame
    g_nRandFlag = 1;
  }

  if (g_nRandFlag)
  {
    g_nNextBright = g_nRand > 15 ? 15 : g_nRand;
  }
}

void SleepMode(void)
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

//  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

  APP_SYSTICK_ISR_ON;
}

void App_TimCallback(void)
{
  _FrameControl();

  HW_PwmSet(g_nPwmValue);
}
