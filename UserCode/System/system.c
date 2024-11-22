#include "system.h"
#include "myusart.h"
#include "dma.h"
#include "config.h"
#include "FreeRTOS_CLI.h"
#include <stdarg.h> // 引入可变参数头文件

extern void vRegisterSampleCLICommands( void );
extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );

uart_t debug_uart = {0};      //调试串口
extern DMA_HandleTypeDef DEBUG_UART_DMA_RX;

/**
 * @brief 调试相关初始化 cmd和log
 * @param  
 */
void debug_init(void)
{
    uart_dma_init(&debug_uart, &DEBUG_UART, &DEBUG_UART_DMA_RX);
    vRegisterSampleCLICommands();
    cmd_init();
    vUARTCommandConsoleStart(256, 4);
    log_init();
}

static char tag_buffer[128] = {0}; // 定义一个128字节的字符数组来存储tag的输出

/**
 * @brief 软件重启
 * @param tag 重启前输出的tag
 */
void system_reset_soft(const char* tag, ...)
{
    
    va_list args; // 创建一个va_list类型的变量，用来存储可变参数
    va_start(args, tag); // 使用va_start宏初始化args，使之指向第一个可选参数
    vsnprintf(tag_buffer, sizeof(tag_buffer), tag, args); // 使用vsnprintf处理可变参数列表
    va_end(args); // 使args不再指向可变参数列表中的任何参数
    log_e("系统即将重启 tag:%s", tag_buffer);
    vTaskDelay(10);
    NVIC_SystemReset(); // 重启
}
