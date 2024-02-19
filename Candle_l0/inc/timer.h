/*
 * timer.h
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32l0xx.h"

typedef struct
{
  uint32_t nLastTime_ms;
  uint32_t nPreviousDebounced;
  uint32_t nPreviousNotDebounced;
  uint32_t nDebounceInterval_ms;
} TimerDebounce_t;

typedef void(*PtrSysTickCallback) (void);

typedef uint64_t timer_tick_t;

void Timer_Init();
void Timer_Delay_ms(uint32_t delay_ms);
timer_tick_t Timer_GetTicks_ms();
void Timer_SetSysTickCallback(PtrSysTickCallback pFunction);


void TimerUs_init(void);
void TimerUs_start(void);
uint16_t TimerUs_get_microseconds(void);
void TimerUs_delay(uint16_t microseconds);
void TimerUs_clear(void);
void TimerUs_stop(void);

void Timer_DebounceInit(TimerDebounce_t* deb, uint32_t nInitState, uint32_t nDebounceInterval_ms);
uint32_t Timer_Debounce(TimerDebounce_t* deb, uint32_t nNewState);

#endif /* TIMER_H_ */
