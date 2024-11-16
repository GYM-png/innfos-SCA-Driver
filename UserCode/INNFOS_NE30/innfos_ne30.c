#include "innfos_ne30.h"
#include "string.h"
#include "sca_protocol.h"
#include "global.h"

void R1dataProcess(Sca_t * sca);
void R2dataProcess(Sca_t * sca);
void R3dataProcess(Sca_t * sca);
void R4dataProcess(Sca_t * sca);
void R5dataProcess(Sca_t * sca);

/**
 * @brief enable sca controller
 * @param sca 
 * @return 
 */
uint8_t SCA_Enable(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Write_1(sca->can, sca->id, W1_PowerState, Actr_Enable);
}

/**
 * @brief disable sca controler
 * @param ptSca 
 * @return 
 */
uint8_t SCA_Disable(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Write_1(sca->can, sca->id, W1_PowerState, Actr_Disable);
}

/**
 * @brief get state of sca controller
 *        Check if the controller is enabled or disabled
 * @param sca 
 * @return 
 */
uint8_t SCA_GetState(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Read(sca->can, sca->id, R1_PowerState);
}

/**
 * @brief check heartbeat of sca controller
 *        the function can check the sca controller wheather online
 * @param sca 
 * @return 
 */
uint8_t SCA_HeartBeat(Sca_t * sca)
{
    if (sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R1_Heartbeat);
}

/**
 * @brief set work mode of sca controller
 * @param ptSca 
 * @param mod sca mode  @ref ne30WorkMode
 * @return 
 */
uint8_t SCA_SetMode(Sca_t * sca, uint8_t mode)
{
    if(sca == NULL)
        return ERROR;
    if (sca->Mode == mode)
        return OK;

    uint8_t result = ERROR;
    
    uint8_t err_t= 0;
    while(sca->Mode != mode)
    {
        err_t++;
        SCA_Write_1(sca->can, sca->id, W1_Mode, mode);
        SCA_GetMode(sca);
        if(err_t >= 30)
        {
            log_e("SCA 模式设置失败 ");
            return ERROR;
        }
        vTaskDelay(100);
    }
    return OK;
}

int8_t SCA_GetMode(Sca_t * sca)
{
    if (sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R1_Mode);
}

/**
 * @brief get the serial number of sca controler
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetSerialNumbe(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R5_ShakeHands);
}


/**
 * @brief set sca controler id
 *        this function must be called when you hand get the serial number 
 *        and only one motor is on the can bus
 * @param ptSca 
 * @param id new id
 * @return 
 */
uint8_t SCA_SetID(Sca_t * sca, uint8_t id)
{
    if(sca == NULL)
        return ERROR;
    
    /* 判断是否查询到序列号 */
    uint16_t num = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        
        num += sca->Serial_Num[i];
    }
    if (num == 0)
    {
        log_e("sca%d 序列号为空 ", sca->id);
        return ERROR;
    }
    

    uint8_t result = ERROR;
    uint8_t * serial_num = (uint8_t*)pvPortMalloc(sizeof(uint8_t) * 6);
    if (serial_num == NULL)
    {
        log_e("内存分配失败 ");
        vPortFree(serial_num);
    }
    memcpy(serial_num, sca->Serial_Num, 6);
    result = SCA_Write_5(sca->can, sca->id, W5_ChangeID, id, serial_num);
    vPortFree(serial_num);
    if (result == OK)
        sca->id = id;
    else
        log_w("SCA ID 设置失败 ");
    return result;
}

/*********************************Position******************************************** */

/**
 * @brief set the motor position as the Position_Target
 * @param ptSca 
 * @return 
 */
uint8_t SCA_SetPosition(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    if(sca->Position_Target < -127.0f || sca->Position_Target > 127.0f)
        log_w("SCA 参数超范围 ");
    if (sca->Mode == SCA_Position_Mode || sca->Mode == SCA_Profile_Position_Mode)
    {
        return SCA_Write_3(sca->can, sca->id, W3_Position, sca->Position_Target);
    }
    log_w("SCA 模式不匹配 ");
    return ERROR;
}

/**
 * @brief set the motor position as the Angle_Target
 * @param ptSca 
 * @return 
 */
uint8_t SCA_SetAngle(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    float position = sca->Angle_Target / 360.0f;
    if(position < -127.0f || position > 127.0f)
        log_w("SCA 参数超范围 ");
    if (sca->Mode == SCA_Position_Mode || sca->Mode == SCA_Profile_Position_Mode)
    {
        return SCA_Write_3(sca->can, sca->id, W3_Position, position);
    }
    log_w("SCA 模式不匹配 ");
    return ERROR;
}


/**
 * @brief get the motor position
 *        the information will update the "Position_Real" and "Angle_Real"
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetPosition(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Read(sca->can, sca->id, R3_Position);
}

/**
 * @brief set the motor homing angle
 * @param ptSca 
 * @param angle 
 * @return 
 */
uint8_t SCA_SetZeroAngle(Sca_t * sca, float angle)
{

    if(sca == NULL)
        return ERROR;
    if (angle == 0)
    {
        return SCA_Write_3(sca->can, sca->id, W3_HomingValue, 0);
    }
    else
    {
        float position = angle / 360.0f;
        return SCA_Write_3(sca->can, sca->id, W3_HomingValue, position);
    }
}


/*********************************Velocity******************************************** */

/**
 * @brief set the motor velocity as Velocity_Target
 * @param ptSca 
 * @return 
 */
uint8_t SCA_SetVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Write_3(sca->can, sca->id, W3_Velocity, sca->Velocity_Target);
}

/**
 * @brief get the motor velocity
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
  
    return SCA_Read(sca->can, sca->id, R3_Velocity);
}

/**
 * @brief set the acceleration and deceleration of the ProfileVelocityMode
 * @param ptSca 
 * @param acce acceleration
 * @param dece deceleration
 * @return 
 */
uint8_t SCA_SetAcceDece(Sca_t * sca, float acce, float dece)
{
    if(sca == NULL)
        return ERROR;

    uint8_t result = ERROR;
    
    result = SCA_Write_3(sca->can, sca->id, W3_PVMaxAcceleration, acce);
    if (result == ERROR)
    {
        return result;
    }
    return SCA_Write_3(sca->can, sca->id, W3_PVMaxDeceleration, dece);
}

/**
 * @brief get the limit of the velocity
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetVelocityLimit(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
   
    return SCA_Read(sca->can, sca->id, R3_VelocityLimit);
}

/**
 * @brief set the max velocity of ProfileVelocityMode
 * @param sca 
 * @param maxVelocity max velocity of ProfileVelocityMode
 * @return 
 */
uint8_t SCA_SetProfileVelocityMaxVelocity(Sca_t * sca, float maxVelocity)
{
    if(sca == NULL)
        return ERROR;

    maxVelocity /= Profile_Scal;
    return  SCA_Write_3(sca->can, sca->id, W3_PVMaxVelocity, maxVelocity);
}

/**
 * @brief get the max velocity of ProfileVelocityMode
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetProfileVelocityMaxVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return  SCA_Read(sca->can, sca->id, R3_PVMaxVelocity);
}

/*********************************currents******************************************** */

/**
 * @brief set sca motor current as Current_Target
 * @param sca 
 * @return 
 */
uint8_t SCA_SetCurrent(Sca_t* sca)
{
    if (sca == NULL)
    {
        return ERROR;
    }
    
	return SCA_Write_3(sca->can, sca->id, W3_Current, sca->Current_Target);
}

/*********************************others******************************************** */

/**
 * @brief get current, velocity and position of the motor
 * @param ptSca 
 * @return 
 */
uint8_t SCA_GetCVP(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    
    return SCA_Read(sca->can, sca->id, R4_CVP);
}

/**
 * @brief get the pwoer supply of the motor
 * @param sca 
 * @return 
 */
uint8_t SCA_GetVoltage(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Read(sca->can, sca->id, R2_Voltage);
}

/**
 * @brief get the motor temperature
 * @param sca 
 * @return 
 */
uint8_t SCA_GetTemperature(Sca_t * sca)
{
    if (sca == NULL)
    {
        return ERROR;
    }
	return  SCA_Read(sca->can, sca->id, R2_MotorTemp);
}

/**
 * @brief get the all information of sca controler
 * @param sca 
 * @return 
 */
uint8_t SCA_GetAllInfomation(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    SCA_GetSerialNumbe(sca);
    SCA_GetProfileVelocityMaxVelocity(sca);
    vTaskDelay(10);
    SCA_GetTemperature(sca);
    vTaskDelay(10);
    SCA_GetVelocityLimit(sca);
    vTaskDelay(10);
    SCA_GetVoltage(sca);
    vTaskDelay(10);
    SCA_GetMode(sca);
    vTaskDelay(10);
    return OK;
}
/****************************data process*******************************/


/**
 * @brief process information returned by the controller
 * @param ptSca 
 * @return 
 */
uint8_t SCA_DataProcess(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    
	switch(sca->can->rx_data[0])
	{
		case R1_Heartbeat:
		case R1_Mode:
		case R1_PositionLimitState:
		case R1_PowerState:
			R1dataProcess(sca);
		    break;
		
		case R2_Voltage:
		case R2_MotorTemp:
		case R2_MotorProtectTemp:
		case R2_MotorRecoverTemp:
		case R2_Error:
            R2dataProcess(sca);
		    break;
		
		case R3_Current:
		case R3_Velocity:	
		case R3_Position:	
		case R3_PPMaxVelocity:
		case R3_PPMaxAcceleration:
		case R3_PPMaxDeceleration:
		case R3_PVMaxVelocity:
		case R3_PVMaxAcceleration:
		case R3_PVMaxDeceleration:
		case R3_CurrentLimit:	
		case R3_VelocityLimit:
		case R3_PositionLimitH:
		case R3_PositionLimitL:
		case R3_PositionOffset:
		case R3_BlockEngy:
			R3dataProcess(sca);
            sca->Angle_Real = (sca->Position_Real) * 360.0f;
		    break;
		
		case R4_CVP:
			R4dataProcess(sca);
            sca->Angle_Real = (sca->Position_Real) * 360.0f;
		    break;
		
		case R5_ShakeHands:
			R5dataProcess(sca);
		    break;
        
		default:
			break;
	}
    return 0;
}


/**
 * @brief initialize the sca controller and set the work mode
 * @param sca sca handle
 * @param id sca controler id
 * @param mod work mode you want to set
 * @param can can bus handle of sca controler 
 * @return 
 */
uint8_t ScaInit(Sca_t* sca, uint8_t id, uint8_t mode, Can_t* can)
{

    if(sca == NULL || can == NULL)
        return ERROR;

    sca->id = id;
    sca->Mode_Target = mode;
    sca->Mode = 0x00;
    sca->can = can;
    sca->Online_State = 1;
    sca->Power_State = 0;

    SCA_Enable(sca);
    SCA_GetState(sca);
    vTaskDelay(10);
    SCA_SetMode(sca, sca->Mode_Target);
    return OK;
}

/************************************user code************************************************************* */

typedef struct
{
    uint8_t id;
    char *name;
}index_name_t;

index_name_t index_name[] = {
    {.id = 0x01, .name = "Current_Mode"},
    {.id = 0x02, .name = "Velocity_Mode"},
    {.id = 0x03, .name = "Position_Mode"},
    {.id = 0x06, .name = "Profile_Position_Mode"},
    {.id = 0x07, .name = "Profile_Velocity_Mode"},
    {.id = 0x08, .name = "SCA_Homing_Mode"},
};

char* find_name_of_mode(uint8_t index)
{
    for (uint8_t i = 0; i < sizeof(index_name) / sizeof(index_name[0]); i++)
    {
        if (index_name[i].id == index)
        {
            return index_name[i].name;
        }
    }
    return NULL;
}

void show_sca_t(void)
{
    SCA_GetAllInfomation(&scaMotor[0]);
    vTaskDelay(5);
    log_v("work mode:\t\t%s", find_name_of_mode(scaMotor[0].Mode));
    log_v("target current:\t\t%.2f", scaMotor[0].Current_Target);
    log_v("target velocity:\t%.2fRPM", scaMotor[0].Velocity_Target);
    log_v("target position:\t%.2fR", scaMotor[0].Position_Target);
    log_v("real current:\t\t%.2f", scaMotor[0].Current_Real);
    log_v("real velocity:\t\t%.2fRPM", scaMotor[0].Velocity_Real);
    log_v("real position:\t\t%.2fR", scaMotor[0].Position_Real);
    log_v("serial numbe:\t\t%02x %02x %02x %02x %02x %02x", scaMotor[0].Serial_Num[0], scaMotor[0].Serial_Num[1],scaMotor[0].Serial_Num[2],scaMotor[0].Serial_Num[3],scaMotor[0].Serial_Num[4], scaMotor[0].Serial_Num[5]);
    log_v("power voltage:\t\t%.2fV", scaMotor[0].Voltage);
    log_v("velocity limit:\t\t%.2fRPM", scaMotor[0].Velocity_Limit);
    log_v("motor temperture:\t%.2f°C", scaMotor[0].Motor_Temp);

}
