/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-12 22:14:37
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-13 15:30:41
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\Cmd\cmd.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CDM_H
#define __CDM_H

#include "main.h"





typedef struct
{
    uint16_t index;         //
    char * cmd;             //命令字符串
    void (*callback)(void); //回调函数
}cmd_t;


// void myprintf(const char *format, ...); 
// void mylog(const char*__format, ...);

// uint8_t cmd_init(void);
// void find_cmd(char * cmd);
// void cmd_install(char* cmd, void(*callback)(void));

void cmd_init(void);

#endif
