/*
 * hw.h
 *
 *  Created on: 22. 1. 2020
 *  Author:     Priesol Vladimir
 */

#ifndef HW_H_
#define HW_H_

#include "stm32l0xx.h"
#include <stdbool.h>

typedef void(*PtrTimIntCb) (void);

void HW_Init(void);
void HW_PwmSet(uint16_t nValue);
uint32_t HW_GetTrueRandomNumber(void);
void HW_LedOnOff(bool bEnable);
void HW_SetTimCallback(PtrTimIntCb pTimCb);

#endif /* HW_H_ */
