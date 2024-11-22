#ifndef __SCA_PROTOCOL_H
#define __SCA_PROTOCOL_H

#include "my_fdcan.h"

#define Actr_Enable     0x01
#define Actr_Disable    0x00



/* ！！！以下宏定义信息参数请勿修改！！！ */

//INNFOS CAN 通信协议指令
//第一类读取指令
#define R1_Heartbeat			0x00        //握手        
#define R1_Mode					0x55        //查询模式
#define R1_PositionLimitState	0x8B        //查询位置限位使能/使能
#define R1_PowerState			0x2B        //查询执行器使能/使能

//第二类读取指令
#define R2_Voltage				0x45        //查询执行器电压(数据为真实值的2^10倍)
#define R2_Current_Max			0x53        //查询最大电流量程
#define R2_MotorTemp			0x5F        //查询电机温度
#define R2_MotorProtectTemp		0x6C        //查询电机保护温度
#define R2_MotorRecoverTemp		0x6E        //查询电机恢复温度
#define R2_Error				0xFF        //查询执行器的报警信息

//第三类读取指令
#define R3_Current				0x04        //查询执行器电流值(数据乘以满量程为真实电流值)
#define R3_Velocity				0x05        //查询执行器速度值(单位：RPM)
#define R3_Position				0x06        //查询执行器位置值(单位：R)
#define R3_PPMaxVelocity		0x1C        //查询位置梯形曲线最大速度
#define R3_PPMaxAcceleration	0x1D        //查询位置梯形曲线最大加速度
#define R3_PPMaxDeceleration	0x1E        //查询位置梯形曲线最大减速度
#define R3_PVMaxVelocity		0x22        //查询速度梯形曲线的最大速度
#define R3_PVMaxAcceleration	0x23        //查询速度梯形曲线的最加大速度
#define R3_PVMaxDeceleration	0x24        //查询速度梯形曲线的最减大速度
#define R3_CurrentLimit			0x59        //查询电流限制值(数据乘以满量程为真实电流值)
#define R3_VelocityLimit		0x5B        //查询查询速度限制值
#define R3_PositionLimitH		0x85        //查询位置的上限值
#define R3_PositionLimitL		0x86        //查询位置的下限值
#define R3_PositionOffset		0x8A        //查询查询位置偏置值
#define R3_BlockEngy			0x7F        //查询堵转能量

//第四类读取指令
#define R4_CVP					0x94        //读取执行器电路速度位置

//第五类读取指令
#define R5_ShakeHands			0x02        //查询执行器的序列号


//第一类写入命令
#define W1_Mode					0x07
#define W1_CurrentFilterState	0X70
#define W1_VelocityFilterState	0x74
#define W1_PositionFilterState	0x78
#define W1_PositionLimitState	0x8C
#define W1_PowerState			0x2A




//第三类写入命令
#define W3_Current				0x08
#define W3_Velocity				0x09
#define W3_Position				0x0A
#define W3_CurrentFilterP		0x0E
#define W3_CurrentFilterI		0x0F
#define W3_VelocityFilterP		0x10
#define W3_VelocityFilterI		0x11
#define W3_PositionFilterP		0x12
#define W3_PositionFilterI		0x13
#define W3_PositionFilterD		0X14
#define W3_PPMaxVelocity		0x1F
#define W3_PPMaxAcceleration	0x20
#define W3_PPMaxDeceleration	0x21
#define W3_PVMaxVelocity		0x25
#define W3_PVMaxAcceleration	0x26
#define W3_PVMaxDeceleration	0x27
#define W3_CurrentFilterLimitL	0x2E
#define W3_CurrentFilterLimitH	0x2F
#define W3_VelocityFilterLimitL	0x30
#define W3_VelocityFilterLimitH	0x31
#define W3_PositionFilterLimitL	0x32
#define W3_PositionFilterLimitH	0x33
#define W3_CurrentLimit			0x58
#define W3_VelocityLimit		0x5A
#define W3_PositionLimitH		0x83
#define W3_PositionLimitL		0x84
#define W3_HomingValue			0x87
#define W3_PositionOffset		0x89
#define W3_HomingCurrentLimitL	0x90
#define W3_HomingCurrentLimitH	0x91
#define W3_BlockEngy			0x7E

//第四类写入命令
#define W4_ClearError			0xFE
#define W4_ClearHome			0x88
#define W4_Save					0x0D

//第五类写入命令
#define W5_ChangeID				0x3D


//变量缩放值定义
// #define Velocity_Max	6000.0f			//速度最大值，固定为6000RPM（仅作为换算用）
#define BlkEngy_Scal	75.225f			//堵转能量缩放值
#define Profile_Scal	960.0f			//梯形参数缩放值
#define IQ8				256.0f			//2^8
#define IQ10			1024.0f			//2^10
#define IQ24			16777216.0f		//2^24
#define IQ30			1073741824.0f	//2^30



uint8_t SCA_Write_1(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData);
uint8_t SCA_Write_4(Can_t * can, uint32_t id, uint8_t cmd);
uint8_t SCA_Write_5(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData, uint8_t * serial_num);
uint8_t SCA_Read(Can_t * can, uint32_t id, uint8_t cmd);



#endif
