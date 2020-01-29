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
void HW_PwmOff(void);

uint32_t HW_GetTrueRandomNumber(void);
void HW_LedOnOff(bool bEnable);
void HW_PwmOn(void);
void HW_SetTimCallback(PtrTimIntCb pTimCb);

void HW_BatVoltageCtrl(bool bEnable);
uint32_t HW_GetBatVoltage(void);
uint32_t HW_GetOptoVoltage(void);

void HW_StartAdc(void);
bool HW_IsAdcConverted(void);
void HW_ResetAdcConverted(void);

#endif /* HW_H_ */
