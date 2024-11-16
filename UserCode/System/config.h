/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-17 22:21:06
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-25 21:11:13
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\System\config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CONFIG_H
#define __CONFIG_H


/*FreeRTOS CLI组件命令查找定义*/
#define PARAMETER_SEPARATOR '-'


/*串口调试相关定义*/
#define DEBUG_UART (huart1)
#define DEBUG_UART_DMA_RX (hdma_usart1_rx)

/*串口缓冲区*/
#define UART_RX_LEN_MAX 30//最大接收长度
#define UART_TX_LEN_MAX 50//最大发送长度

#endif
