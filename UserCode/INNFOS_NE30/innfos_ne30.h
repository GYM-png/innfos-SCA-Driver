#ifndef INNFOS_NE30_INNFOS_NE30_H
#define INNFOS_NE30_INNFOS_NE30_H

#include "my_fdcan.h"


/**
 * @brief SCA报警信息
 */
typedef struct
{
    uint16_t Error_Code;			//错误代码

    /* 具体报警信息，0：正常，1：出错 */
    uint8_t WARN_OVER_VOLT;  		//过压异常
    uint8_t WARN_UNDER_VOLT;  		//欠压异常
    uint8_t WARN_LOCK_ROTOR;  		//堵转异常
    uint8_t WARN_OVER_TEMP;  		//过热异常
    uint8_t WARN_RW_PARA;  			//读写参数异常
    uint8_t WARN_MUL_CIRCLE;  		//多圈计数异常
    uint8_t WARN_TEMP_SENSOR_INV; 	//逆变器温度传感器异常
    uint8_t WARN_CAN_BUS;  			//CAN通讯异常
    uint8_t WARN_TEMP_SENSOR_MTR;	//电机温度传感器异常
    uint8_t WARN_OVER_STEP;			//位置模式阶跃大于1
    uint8_t WARN_DRV_PROTEC;  		//DRV保护
    uint8_t WARN_DEVICE;  			//设备异常

}sca_errcode_t;

/**
 * @defgroup ne30WorkMode
 */
#define  SCA_Current_Mode			    0x01
#define  SCA_Velocity_Mode			    0x02
#define  SCA_Position_Mode			    0x03
#define  SCA_Profile_Position_Mode	    0X06
#define  SCA_Profile_Velocity_Mode	    0X07
#define  SCA_Homing_Mode				0X08

/**
 * @brief SCA 电机型号
 */
typedef enum
{
	NE30 = 0, 			//
	Lite_NE30_36 = 1, 	//
}sca_model_e;

/**
 * @brief 执行器的各种参数信息
 */
typedef struct 
{
	/* 通信相关信息 */
    uint8_t Serial_Num[6];			//序列号
    Can_t * can;                    //can驱动
    uint32_t id;                    //id

	/* 不同电机的参数 */
	sca_model_e model;				//电机型号
	float ReductionRatio; 			//减速比					
	float Velocity_MaxRange; 		//转速满量程 (RPM) 速度环使用(初始化赋值，只和电机有关，不能改变)

	/* 目标值 */
	float Current_Target;			//目标电流 (单位：A）
	float Velocity_Target;			//目标速度 (单位：RPM）
	float Position_Target;			//目标位置 (单位：R）
    float Angle_Target;	            //目标角度 (单位：度)
	float Mode_Target;				//目标工作模式

	/* 第一类数据变量 */
	uint8_t Mode;					//当前操作模式 @brief ne30WorkMode
	uint8_t Position_Limit_State;	//位置限位状态
	uint8_t Power_State;			//开关机状态
	uint8_t Online_State;			//在线状态

	/* 第二类数据变量 */
	float Voltage;					//当前电压（单位：V）
	float Motor_Temp;				//电机温度
	float Motor_Protect_Temp;		//电机保护温度
	float Motor_Recover_Temp;		//电机恢复温度
	float Current_MaxRange;			//电流满量程 (A) 电流环使用(初始化获取，只和电机有关，不能改变)
	sca_errcode_t * ptErrcode;	    //电机报警信息

	/* 第三类数据变量 */
	float Current_Real;				//当前电流 (单位：A）
	float Velocity_Real;			//当前速度 (单位：RPM）
	float Position_Real;			//当前位置 (单位：R）
    float Angle_Real;	            //当前角度 (单位：度)
	float PP_Max_Velocity;			//位置梯形速度最大值
	float PP_Max_Acceleration;		//位置梯形加速度最大值
	float PP_Max_Deceleration;		//位置梯形减速度最大值
	float PV_Max_Velocity;			//速度梯形速度最大值
	float PV_Max_Acceleration;		//速度梯形加速度最大值
	float PV_Max_Deceleration;		//速度梯形减速度最大值
	float Position_Filter_Limit_L;	//位置环输出下限
	float Position_Filter_Limit_H;	//位置环输出上限
	float Position_Limit_H;			//执行器的位置上限
	float Position_Limit_L;			//执行器的位置下限
	float Current_Limit;			//电流输入限幅
	float Velocity_Limit;			//速度输入限幅
	float Homing_Value;				//执行器的Homing值
	float Position_Offset;			//执行器的位置偏置
	float Blocked_Energy;			//堵转锁定能量

}Sca_t;

uint8_t ScaInit(Sca_t* sca, sca_model_e model, uint8_t id, uint8_t workmode, Can_t* can);
uint8_t SCA_Enable(Sca_t * sca);
uint8_t SCA_Disable(Sca_t * sca);
uint8_t SCA_GetState(Sca_t * sca);
uint8_t SCA_HeartBeat(Sca_t * sca);
uint8_t SCA_SetMode(Sca_t * sca, uint8_t mod);
int8_t SCA_GetMode(Sca_t * sca);
uint8_t SCA_GetSerialNumbe(Sca_t * sca);
uint8_t SCA_SetID(Sca_t * sca, uint8_t id);

uint8_t SCA_SetPosition(Sca_t * sca);
uint8_t SCA_SetAngle(Sca_t * sca);
uint8_t SCA_GetPosition(Sca_t * sca);
uint8_t SCA_SetZeroAngle(Sca_t * sca, float angle);
uint8_t SCA_GetPPAcceleration(Sca_t * sca);
uint8_t SCA_GetPPDeceleration(Sca_t * sca);
uint8_t SCA_SetPPAcceleration(Sca_t * sca, float acce);
uint8_t SCa_SetPPDeceleration(Sca_t * sca, float dece);
uint8_t SCA_GetPPMaxVelocity(Sca_t * sca);
uint8_t SCA_SetPPMaxcVelocity(Sca_t * sca, float max_velocity);

uint8_t SCA_SetVelocity(Sca_t * sca);
uint8_t SCA_GetVelocity(Sca_t * sca);
uint8_t SCA_GetPVAcceleration(Sca_t * sca);
uint8_t SCA_GetPVDeceleration(Sca_t * sca);
uint8_t SCA_SetPVAcceleration(Sca_t * sca, float acceleration);
uint8_t SCA_SetPVDeceleration(Sca_t * sca, float deceleration);
uint8_t SCA_GetVelocityLimit(Sca_t * sca);
uint8_t SCA_GetPVMaxVelocity(Sca_t * sca);
uint8_t SCA_SetPVMaxVelocity(Sca_t * sca, float maxVelocity);

uint8_t SCA_SetCurrent(Sca_t* sca);
uint8_t SCA_GetCurrentMaxRange(Sca_t* sca);
uint8_t SCA_GetCurrentLimit(Sca_t* sca);

uint8_t SCA_GetCVP(Sca_t * sca);
uint8_t SCA_GetVoltage(Sca_t * sca);
uint8_t SCA_GetTemperature(Sca_t * sca);
uint8_t SCA_GetAllparameters(Sca_t * sca);
uint8_t SCA_SaveAllParamters(Sca_t * sca);

uint8_t SCA_DataProcess(Sca_t * sca);


/************************************user code************************************************************* */
char* find_name_of_mode(uint8_t index);


#endif //INNFOS_NE30_INNFOS_NE30_H
