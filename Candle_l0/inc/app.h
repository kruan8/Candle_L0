/*
 * app.h
 *
 *  Created on: 22. 1. 2020
 *  Author:     Priesol Vladimir
 */

#ifndef APP_H_
#define APP_H_

#include "stm32l0xx.h"
#include "stdbool.h"

void App_Init(void);
void App_Exec(void);

void HW_SetLowPowerMode(bool bStandby);
void App_PwmSet(uint16_t nValue);
void App_TimCallback(void);


#endif /* APP_H_ */
