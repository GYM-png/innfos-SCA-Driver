/*
 * @Author: GYM 480609450@qq.com
 * @Date: 2024-08-04 22:17:44
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-09-04 15:45:40
 * @Description: SCA通信协议层
 */


#include "sca_protocol.h"
#include "string.h"
#include "innfos_ne30.h"

static void warnBitAnaly(sca_errcode_t* ptErr);

/**
 * @brief 第一类写入命令 发送2字节，返回2字节
 * @param sca 
 * @param cmd 
 * @param TxData 
 * @return 
 */
uint8_t SCA_Write_1(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData)
{
    static uint8_t tx_buf[2];

    /* 数据打包格式：
        tx_buf[0]-操作命令 	tx_buf[1]-数据（高位）至 tx_buf[7]-数据（低位） */
    tx_buf[0] = cmd;
    tx_buf[1] = TxData;

    /* 调用底层通信函数传输数据，若出现通信错误则返回错误值 */
	return can_send(can, id, tx_buf, 2);
}

/**
 * @brief 第三类写入命令
 * @param can 
 * @param id 
 * @param cmd 
 * @param TxData 
 * @return 
 */					
uint8_t SCA_Write_3(Sca_t * sca, uint8_t cmd, float TxData)
{
    static uint8_t tx_buf[5];
    int32_t temp;

    /*	速度与电流在设定时，要采用标值，
        即设定值除以该参数的最大值，再转换为IQ24格式	*/
    if((cmd == W3_Velocity)||(cmd == W3_VelocityLimit))
        temp = TxData / sca->Velocity_MaxRange * IQ24;
    else if((cmd == W3_Current)||(cmd == W3_CurrentLimit))
        temp = TxData / sca->Current_MaxRange * IQ24;
    else if(cmd == W3_BlockEngy)
        temp = TxData * BlkEngy_Scal;	//堵转能量为真实值的75.225倍
    else
        temp = TxData * IQ24;

    tx_buf[0] = cmd;
    tx_buf[1] = (uint8_t)(temp>>24);
    tx_buf[2] = (uint8_t)(temp>>16);
    tx_buf[3] = (uint8_t)(temp>>8);
    tx_buf[4] = (uint8_t)(temp>>0);

    return can_send(sca->can, sca->id, tx_buf, 5);
}


/**
  * @功	能	第4类写入命令，发送1byte，接收2byte
  * @参	数	pSCA：要操作的执行器句柄指针或地址
  *			cmd：操作指令
  * @返	回	SCA_NoError：发送成功
  *			其他通信错误参见 SCA_Error 错误列表
  */
uint8_t SCA_Write_4(Can_t * can, uint32_t id, uint8_t cmd)
{
	static uint8_t tx_buf[8];

	tx_buf[0] = cmd;

    return can_send(can, id, tx_buf, 1);
}


/**
  * @功	能	第5类写入命令，发送8byte，接收2byte
  * @参	数	sca：要操作的执行器句柄指针或地址
  *			cmd：操作指令
  *			TxData：发送数据
  * @返	回	SCA_NoError：发送成功
  *			其他通信错误参见 SCA_Error 错误列表
  */
uint8_t SCA_Write_5(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData, uint8_t * serial_num)
{
	static uint8_t tx_buf[8];

	tx_buf[0] = cmd;
	memcpy(tx_buf + 1, serial_num, 6);
	tx_buf[7] = TxData;
    return can_send(can, id, tx_buf, 8);
}

/**
  * @功	能	读取命令接口，发送1byte
  * @参	数	sca：要操作的执行器句柄指针或地址
  *			cmd：操作指令
  * @返	回	SCA_NoError：操作成功
  *			其他通信错误参见 SCA_Error 错误列表
  */
uint8_t SCA_Read(Can_t * can, uint32_t id, uint8_t cmd)
{
	static uint8_t tx_buf[1];
	tx_buf[0] = cmd;
	return can_send(can, id, tx_buf, 1);
}

/**
  * @功	能	第1类读取命令返回数据解析，发送1byte，接收2byte
  * @参	数	sca：目标执行器句柄指针或地址
  *			RxMsg：接收到的数据包
  * @返	回	无
  */			
void R1dataProcess(Sca_t * sca)
{
	/* 将读取结果装载到接收地址中 */
	switch(sca->can->rx_data[0])
	{
		case R1_Heartbeat:
			sca->Online_State = Actr_Enable;
			break;
			
		case R1_Mode:
			sca->Mode = sca->can->rx_data[1];
			break;
			
		case R1_PositionLimitState:
			sca->Position_Limit_State = sca->can->rx_data[1];
			break;
		
		case R1_PowerState: 
			sca->Power_State = sca->can->rx_data[1];	
			break;
			
		default:
			break;
	}
}

/**
  * @功	能	第2类读取命令返回数据解析，发送1byte，接收3byte
  * @参	数	sca：目标执行器句柄指针或地址
  *			RxMsg：接收到的数据包
  * @返	回	无
  */
void R2dataProcess(Sca_t * sca)
{
	int16_t temp;
	float RxData;
	
	/* 第二类读写命令为IQ8格式 */
	temp  = ((int16_t)sca->can->rx_data[1])<<8;
	temp |= ((int16_t)sca->can->rx_data[2])<<0;
	
	/* 在第二类读写命令中，电压数据为IQ10格式 */
	if(sca->can->rx_data[0] == R2_Voltage)					
		RxData = (uint16_t)temp / IQ10;
	else
		RxData = (float)temp / IQ8;

	switch(sca->can->rx_data[0])
	{
		case R2_Voltage:
			sca->Voltage = RxData;
			break;
				
		case R2_MotorTemp:
			sca->Motor_Temp  = RxData;
			break;
				
		case R2_MotorProtectTemp:
			sca->Motor_Protect_Temp = RxData;
			break;
		
		case R2_MotorRecoverTemp:
			sca->Motor_Recover_Temp = RxData;
			break;
		
		case R2_Current_Max:
		    sca->Current_MaxRange = RxData;
		
		case R2_Error:
			sca->Error_Code = (uint16_t)RxData;
			// sca->ptErrcode->Error_Code = (uint16_t)RxData;
			// warnBitAnaly(sca->ptErrcode);
			break;

		default:
			break;
	}
}

/**
  * @功	能	第3类读取命令返回数据解析，发送1byte，接收5byte
  * @参	数	sca：目标执行器句柄指针或地址
  *			RxMsg：接收到的数据包
  * @返	回	无
  */
void R3dataProcess(Sca_t * sca)
{
	int32_t temp;
	float RxData;

	/* 第三类读写命令以IQ24格式传输 */
	temp  = ((int32_t)sca->can->rx_data[1])<<24;
	temp |= ((int32_t)sca->can->rx_data[2])<<16;
	temp |= ((int32_t)sca->can->rx_data[3])<<8;
	temp |= ((int32_t)sca->can->rx_data[4])<<0;

	/* 速度和电流使用标值，需要将转换值乘以该参数的最大值得到实际值 */
	if((sca->can->rx_data[0] == R3_Velocity)||(sca->can->rx_data[0] == R3_VelocityLimit))
		RxData = (float)temp / IQ24 * sca->Velocity_MaxRange; 
	
	else if((sca->can->rx_data[0] == R3_Current)||(sca->can->rx_data[0] == R3_CurrentLimit))
		RxData = (float)temp / IQ24 * sca->Current_MaxRange; 
	
	else if(sca->can->rx_data[0] == R3_BlockEngy)
		RxData = (float)temp / BlkEngy_Scal; 	//堵转能量为真实的75.225倍
		
	else
		RxData = (float)temp / IQ24; 

	switch(sca->can->rx_data[0])
	{
		case R3_Current:
			sca->Current_Real = RxData;
			break;
		
		case R3_Velocity:	
			sca->Velocity_Real = RxData / sca->ReductionRatio;
			break;
		
		case R3_Position:	
			sca->Position_Real = RxData / sca->ReductionRatio;
			break;
				
		case R3_PPMaxVelocity:
			sca->PP_Max_Velocity = RxData * Profile_Scal / sca->ReductionRatio;
			break;
		
		case R3_PPMaxAcceleration:
			sca->PP_Max_Acceleration = RxData * Profile_Scal / sca->ReductionRatio;
			break;
		
		case R3_PPMaxDeceleration:
			sca->PP_Max_Deceleration = RxData * Profile_Scal / sca->ReductionRatio;
			break;
		
		case R3_PVMaxVelocity:
			sca->PV_Max_Velocity = RxData * Profile_Scal / sca->ReductionRatio;
			break;
		
		case R3_PVMaxAcceleration:
			sca->PV_Max_Acceleration = RxData * Profile_Scal / sca->ReductionRatio;
			break;
		
		case R3_PVMaxDeceleration:
			sca->PV_Max_Deceleration = RxData * Profile_Scal / sca->ReductionRatio;
			break;
				
		case R3_CurrentLimit:	
			sca->Current_Limit = RxData;
			break;
		
		case R3_VelocityLimit:
			sca->Velocity_Limit = RxData / sca->ReductionRatio;
			break;
		
		case R3_PositionLimitH:
			sca->Position_Limit_H = RxData / sca->ReductionRatio;
			break;
		
		case R3_PositionLimitL:
			sca->Position_Limit_L = RxData / sca->ReductionRatio;
			break;
		
		case R3_PositionOffset:
			sca->Position_Offset = RxData / sca->ReductionRatio;
			break;
				
		case R3_BlockEngy:
			sca->Blocked_Energy = RxData;
			break;

		default:
			break;
	}
}

/**
  * @功	能	第4类读取命令返回数据解析，发送1byte，接收8byte
  * @参	数	sca：目标执行器句柄指针或地址
  *			RxMsg：接收到的数据包
  * @返	回	无
  */
void R4dataProcess(Sca_t * sca)
{
	int32_t temp;	

	/*	在三环读取协议中，为了使速度、电流、位置数据在同一数据帧中表示出
		将电流和速度值以IQ14格式传输，将位置值以IQ16格式传输。为了方便符
		号位的计算，将位置值向左移8位对齐符号位，转而除以IQ24得到真实值；
		同理，将电流和速度值左移16位对齐符号位，转而除以IQ30得到真实值	。	*/
	
	temp  = ((int32_t)sca->can->rx_data[1])<<24;
	temp |= ((int32_t)sca->can->rx_data[2])<<16;
	temp |= ((int32_t)sca->can->rx_data[3])<<8;
	sca->Position_Real = (float)temp / IQ24  / sca->ReductionRatio;

	temp  = ((int32_t)sca->can->rx_data[4])<<24;
	temp |= ((int32_t)sca->can->rx_data[5])<<16;
	sca->Velocity_Real = (float)temp / IQ30 * sca->Velocity_MaxRange  / sca->ReductionRatio;

	temp  = ((int32_t)sca->can->rx_data[6])<<24;
	temp |= ((int32_t)sca->can->rx_data[7])<<16;
	sca->Current_Real  = (float)temp / IQ30 * sca->Current_MaxRange; 
	
}

/**
  * @功	能	第5类读取命令返回数据解析，发送1byte，接收7byte
  *			用于查询指定执行器的序列号
  * @参	数	sca：目标执行器句柄指针或地址
  *			RxMsg：接收到的数据包
  * @返	回	无
  */
void R5dataProcess(Sca_t * sca)
{
	/* 装填序列号 */
	memcpy(sca->Serial_Num, sca->can->rx_data + 1, 6);
}

/**
  * @功	能	识别错误代码中的具体错误信息
  * @参	数	pSCA：要操作的执行器句柄地址或指针
  * @返	回	无
  */
static void warnBitAnaly(sca_errcode_t* ptErr)
{
	if(ptErr->Error_Code & 0x0001)
		ptErr->WARN_OVER_VOLT = Actr_Enable;
	else
		ptErr->WARN_OVER_VOLT = Actr_Disable;

	if(ptErr->Error_Code & 0x0002)
		ptErr->WARN_UNDER_VOLT = Actr_Enable;
	else
		ptErr->WARN_UNDER_VOLT = Actr_Disable;

	if(ptErr->Error_Code & 0x0004)
		ptErr->WARN_LOCK_ROTOR = Actr_Enable;
	else
		ptErr->WARN_LOCK_ROTOR = Actr_Disable;

	if(ptErr->Error_Code & 0x0008)
		ptErr->WARN_OVER_TEMP = Actr_Enable;
	else
		ptErr->WARN_OVER_TEMP = Actr_Disable;

	if(ptErr->Error_Code & 0x0010)
		ptErr->WARN_RW_PARA = Actr_Enable;
	else
		ptErr->WARN_RW_PARA = Actr_Disable;

	if(ptErr->Error_Code & 0x0020)
		ptErr->WARN_MUL_CIRCLE = Actr_Enable;
	else
		ptErr->WARN_MUL_CIRCLE = Actr_Disable;

	if(ptErr->Error_Code & 0x0040)
		ptErr->WARN_TEMP_SENSOR_INV = Actr_Enable;
	else
		ptErr->WARN_TEMP_SENSOR_INV = Actr_Disable;

	if(ptErr->Error_Code & 0x0080)
		ptErr->WARN_CAN_BUS = Actr_Enable;
	else
		ptErr->WARN_CAN_BUS = Actr_Disable;

	if(ptErr->Error_Code & 0x0100)
		ptErr->WARN_TEMP_SENSOR_MTR= Actr_Enable;
	else
		ptErr->WARN_TEMP_SENSOR_MTR = Actr_Disable;

	if(ptErr->Error_Code & 0x0200)
		ptErr->WARN_OVER_STEP= Actr_Enable;
	else
		ptErr->WARN_OVER_STEP = Actr_Disable;

	if(ptErr->Error_Code & 0x0400)
		ptErr->WARN_DRV_PROTEC= Actr_Enable;
	else
		ptErr->WARN_DRV_PROTEC = Actr_Disable;

	if(ptErr->Error_Code & 0xF800)
		ptErr->WARN_DEVICE= Actr_Enable;
	else
		ptErr->WARN_DEVICE = Actr_Disable;
}
