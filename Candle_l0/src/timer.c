/*
 * timer.c
 *
 *  Created on: 24. 6. 2016
 *      Author: Priesol Vladimir
 */

#include "timer.h"

#define TIMER_US      TIM21         // prirazeni casovace pro mereni us intervalu
#define TIMER_US_CLK  RCC_APB2ENR_TIM21EN


typedef void(*Ptr_OnTxDataPacketResponse)(void);

static volatile uint32_t nDelayTimer;
static volatile timer_tick_t g_nTicks = 0;

PtrSysTickCallback pSysTickCallback = 0;

void Timer_Init()
{
  SystemCoreClockUpdate();
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  TimerUs_init();
}

void Timer_Delay_ms(uint32_t delay_ms)
{
  nDelayTimer = delay_ms;
  while (nDelayTimer);
}

timer_tick_t Timer_GetTicks_ms()
{
  return g_nTicks;
}

void Timer_SetSysTickCallback(PtrSysTickCallback pFunction)
{
  pSysTickCallback = pFunction;
}

void SysTick_Handler(void)
{
  g_nTicks++;
  if (nDelayTimer)
  {
    nDelayTimer--;
  }

  if (pSysTickCallback)
  {
    pSysTickCallback();
  }
}


// timer for us counting
void TimerUs_init(void)
{
    // Enable clock for TIM22
    RCC->APB2ENR |= TIMER_US_CLK;
}

void TimerUs_start(void)
{
  TIMER_US->PSC = SystemCoreClock / 1000000; // 7 instructions
  TIMER_US->CNT = 0;
  TIMER_US->EGR = TIM_EGR_UG;
  TIMER_US->CR1 |= TIM_CR1_CEN;
}

uint16_t TimerUs_get_microseconds(void)
{
    return TIMER_US->CNT;
}

void TimerUs_delay(uint16_t microseconds)
{
    uint16_t t = TimerUs_get_microseconds() + microseconds;
    while (TimerUs_get_microseconds() < t)
    {
        continue;
    }
}

void TimerUs_clear(void)
{
  TIMER_US->CNT = 0;
}

void TimerUs_stop(void)
{
  TIMER_US->CR1 &= ~TIM_CR1_CEN;
}

void Timer_DebounceInit(TimerDebounce_t* deb, uint32_t nInitState, uint32_t nDebounceInterval_ms)
{
  deb->nPreviousDebounced = nInitState;
  deb->nDebounceInterval_ms = nDebounceInterval_ms;
  deb->nLastTime_ms = Timer_GetTicks_ms();
}

uint32_t Timer_Debounce(TimerDebounce_t* deb, uint32_t nCurrentState)
{
  if (deb->nPreviousNotDebounced == nCurrentState)
  {
    if ((Timer_GetTicks_ms() - deb->nLastTime_ms) > deb->nDebounceInterval_ms)
    {
      deb->nPreviousDebounced = nCurrentState;
    }
  }
  else
  {
    deb->nPreviousNotDebounced = nCurrentState;
    deb->nLastTime_ms = Timer_GetTicks_ms();
  }

  return deb->nPreviousDebounced;
}

//void _DebounceSwitch(bool bSwitchState)
//{
//  static uint32_t Count = HW_PRESS_MSEC / HW_CHECK_MSEC;
//
//  if (bSwitchState == g_bDebouncedKeyPress)
//  {
//    // Set the timer which will allow a change from the current state.
//    if (g_bDebouncedKeyPress)
//    {
//      g_nButtonStateDuration++;
//      Count = HW_RELEASE_MSEC / HW_CHECK_MSEC;
//    }
//    else
//    {
//      Count = HW_PRESS_MSEC / HW_CHECK_MSEC;
//    }
//  }
//  else
//  {
//    // Key has changed - wait for new state to become stable.
//    g_nButtonStateDuration = 0;
//    if (--Count == 0)
//    {
//      // Timer expired - accept the change.
//      g_bDebouncedKeyPress = bSwitchState;
//
//      // And reset the timer.
//      if (g_bDebouncedKeyPress)
//      {
//        Count = HW_RELEASE_MSEC / HW_CHECK_MSEC;
//      }
//      else
//      {
//        Count = HW_PRESS_MSEC / HW_CHECK_MSEC;
//      }
//    }
//  }
//}
