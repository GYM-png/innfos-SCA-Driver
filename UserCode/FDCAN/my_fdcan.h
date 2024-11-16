#ifndef __MY_FDCAN_H
#define __MY_FDCAN_H

#include "fdcan.h"
#include "system.h"

typedef struct
{
    FDCAN_HandleTypeDef * hcan_t;
    FDCAN_RxHeaderTypeDef * rx_header_t;
    uint8_t rx_data[8];
    SemaphoreHandle_t mutex;
}Can_t;

uint8_t can_init(Can_t *can, FDCAN_HandleTypeDef* hcan);
uint8_t can_send(Can_t *can, uint32_t id, uint8_t * data, uint8_t dlc);
uint8_t can_receive(Can_t *can);





#endif
