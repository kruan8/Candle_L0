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
#include "stm32l0xx_ll_cortex.h"


#define APP_SYSTICK_ISR_OFF     SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk  // vypnout preruseni od Systick
#define APP_SYSTICK_ISR_ON      SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk   // zapnout preruseni od Systick

#define APP_MEASURE_BAT_CTRL_MS     10000
#define APP_MEASURE_OFFSET_MS         100

#define APP_BATT_MIN_MV             3200
#define APP_OPTO_MIN_MV              300

#define APP_BUTTON_DEB_INTERVAL_MS    10  // debounce interval for button

#define APP_OFF_INTERVAL_MS         1000*3600  // 1 hour

#define APP_MODE_MODULATION            0
#define APP_MODE_ON                    1
#define APP_MODE_SHOUTDOWN             2

#include "data.inc"


static uint32_t       g_nBatCtrlTime;
static uint32_t       g_nMeasureTime;

static bool           g_bInitializated;    // wait for first ADC conversion
static uint8_t        g_FrameCounter;

TimerDebounce_t       g_ButtonDeb;
uint32_t              g_nButtonState;

uint32_t              g_nOffInterval;
uint32_t              g_nMode;

void _FrameControl(void);
void _Sleep(void);
uint32_t _GetTrueRandomNumber(void);
void _App_SysTimerCallback(void);

void App_Init(void)
{
#ifdef DEBUG
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif

  HW_Init();

  Timer_DebounceInit(&g_ButtonDeb, true, APP_BUTTON_DEB_INTERVAL_MS);
  Timer_SetSysTickCallback(_App_SysTimerCallback);
  HW_SetTimCallback(App_TimCallback);

  g_nBatCtrlTime = 0;
  g_bInitializated = false;
  g_nOffInterval = APP_OFF_INTERVAL_MS;
  g_nButtonState = true;
  g_nMode = APP_MODE_MODULATION;

  while (g_nButtonState == true);
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
    if (HW_GetBatVoltage() < APP_BATT_MIN_MV)
    {
      g_nOffInterval = 0;
    }

    HW_ResetAdcConverted();
    if (!g_bInitializated)
    {
      g_bInitializated = true;
      HW_PwmOn();
    }
  }

//  LL_IWDG_ReloadCounter(IWDG);

  if (g_nButtonState == true)
  {
    g_nMode++;
    while (g_nButtonState == true);
    if (g_nMode == APP_MODE_ON)
    {
      HW_SetTimCallback(0);
      HW_PwmSet(0xFFFF);
    }
  }

  // shoutdown
  if (g_nOffInterval == 0 || g_nMode == APP_MODE_SHOUTDOWN)
  {
    // standby/stop
    HW_PwmOff();
    HW_SetWakeUpPin();
    HW_SetLowPowerMode(true);
    while(1);
  }

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

void HW_SetLowPowerMode(bool bStandby)
{
  // Adc_Disable();
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableUltraLowPower();
  if (bStandby)
  {
    PWR->CR |= PWR_CR_PDDS;  // rozlisuje mody STANDBY | STOP
  }

  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  LL_PWR_ClearFlag_WU();  // Clear Wakeup flag
  LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);  //PWR->CR |= PWR_CR_LPSDSR;
  LL_LPM_EnableDeepSleep();

  __asm volatile ("wfi");

  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
  LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
  SysTick->VAL = 0;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

//  Adc_Enable();
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

void _App_SysTimerCallback(void)
{
  g_nButtonState = Timer_Debounce(&g_ButtonDeb, HW_IsButtonOn());

  if (g_nOffInterval > 0)
  {
    g_nOffInterval--;
  }
}
