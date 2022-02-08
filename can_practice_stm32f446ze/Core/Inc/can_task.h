#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <stdio.h>

#ifndef CAN_TASK_H_
#define CAN_TASK_H_

#ifdef __cplusplus
extern "C"
{
#endif

extern CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
extern uint8_t RxData0[8];
extern uint8_t RxData1[8];
extern int rxfifo;

void CAN_write(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t length, uint8_t TxData[]);
void CAN_read(CAN_HandleTypeDef *hcan, uint32_t RxFIFO, uint8_t RxData[]);
void CAN_error_handler(void);

#ifdef __cplusplus
}
#endif

#endif

