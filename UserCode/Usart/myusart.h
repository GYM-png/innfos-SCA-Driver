/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-13 11:05:20
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-25 20:53:29
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\Usart\myusart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYUSART_H
#define __MYUSART_H

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "global.h"
#include "config.h"
#include "system.h"




typedef struct
{
    UART_HandleTypeDef * huart;    //串口句柄
    DMA_HandleTypeDef* dma_t;       //dma
    uint16_t rx_len;                //接收数据长度
    uint8_t  rx_buffer[UART_RX_LEN_MAX];            //接收缓冲区
    uint8_t tx_data[UART_TX_LEN_MAX];              //
    uint8_t rx_flag;                //接收标志
    SemaphoreHandle_t mutex;//互斥
}uart_t;


uint8_t uart_dma_init(uart_t* uart_dma, UART_HandleTypeDef* huart, DMA_HandleTypeDef* dma);
void uart_idle_callback(uart_t *uart_dma);
uint8_t uart_transmit(uart_t *uart, uint8_t* data, uint16_t len);


#endif
