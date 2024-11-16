/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-13 00:23:37
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-15 23:04:54
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\global\global.h
 * @Description: ����Ĭ������,������`customMade`, ��koroFileHeader�鿴���� ��������: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "main.h"
//#include "rtc.h"
#include "system.h"
#include "innfos_ne30.h"
#include "my_fdcan.h"

#define ERROR   1
#define OK      0

typedef struct
{
	uint16_t Hours;
	uint16_t Minutes;
	uint16_t Seconds;
}Rtc_Time_t;

typedef struct
{
	uint16_t Year;
	uint16_t Month;
	uint16_t Date;
}Rtc_Date_t;

typedef struct
{
	float velocity;
	float current;
	float position;
}Sca_Targte_t;


#define MOTOR_NUM  2
extern Sca_t scaMotor[MOTOR_NUM];
extern Can_t fdcan1;
extern Sca_Targte_t sca_target;




void start_task_init(void);



extern uint32_t systern_run_time;
extern uint16_t rtc_ms;//ϵͳʱ��ms rtcֻ���ṩ��ȷ��s

extern Rtc_Time_t rtc_time;
extern Rtc_Date_t rtc_date;


#endif
