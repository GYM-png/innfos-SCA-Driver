#include "innfos_ne30.h"
#include "string.h"
#include "sca_protocol.h"
#include "global.h"

uint8_t SCA_Write_3(Sca_t * sca, uint8_t cmd, float TxData);
void R1dataProcess(Sca_t * sca);
void R2dataProcess(Sca_t * sca);
void R3dataProcess(Sca_t * sca);
void R4dataProcess(Sca_t * sca);
void R5dataProcess(Sca_t * sca);


static void SCA_InitModelParameters(Sca_t * sca, sca_model_e model)
{
    sca->model = model;
    switch (model)
    {
    case NE30:
        sca->ReductionRatio = 1.0f;
        sca->Velocity_MaxRange = 6000.0f;
        break;
    
    case Lite_NE30_36:
        sca->ReductionRatio = 36.0f;
        sca->Velocity_MaxRange = 6000.0f;
        break;

    default:
        break;
    }
}


/**
 * @brief Enables the SCA motor.
 *
 * This function sends a command to the SCA motor to enable its operation.
 * The motor will start responding to control commands after it is enabled.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the enable command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_Enable(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Write_1(sca->can, sca->id, W1_PowerState, Actr_Enable);
}


/**
 * @brief Disables the SCA motor.
 *
 * This function sends a command to the SCA motor to disable it.
 * The motor will stop responding to control commands and enter a low-power state.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_Disable(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Write_1(sca->can, sca->id, W1_PowerState, Actr_Disable);
}


/**
 * @brief Retrieves the state of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its power state.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the heartbeat is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetState(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R1_PowerState);
}


/**
 * @brief Sends a heartbeat command to the SCA motor to check its status.
 *
 * This function sends a command to shake hands with the motor to check its connection status
 * The motor will reply with a bit 0x00
 * 
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the heartbeat is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_HeartBeat(Sca_t * sca)
{
    if (sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R1_Heartbeat);
}


/**
 * @brief Sets the mode of the SCA motor.
 *
 * This function sends a command to the SCA motor to set its mode.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param mode Desired mode of the SCA motor. @ref ne30WorkMode in innfos_ne30.h
 *
 * @return Returns OK if the mode is successfully set.
 *         Returns ERROR if the sca pointer is NULL or if the mode setting fails.
 *         The function will retry setting the mode up to 3 times with a delay of 1 second between attempts.
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
        if(err_t >= 3)
        {
            log_e("sca%d 模式设置失败 ", sca->id);
            return ERROR;
        }
        vTaskDelay(1000);
    }
    log_i("sca %d is working in %s", sca->id, find_name_of_mode(sca->Mode));
    return OK;
}



/**
 * @brief Retrieves the mode of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its mode.
 * The mode is represented by a single byte value. @ref ne30WorkMode in innfos_ne30.h
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the mode value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
int8_t SCA_GetMode(Sca_t * sca)
{
    if (sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R1_Mode);
}


/**
 * @brief Retrieves the serial number of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its serial number.
 * The serial number is a 6 bytes value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetSerialNumbe(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R5_ShakeHands);
}



/**
 * @brief Sets the ID of the SCA motor.
 *
 * This function sends a command to the SCA motor to change its ID.
 * Before changing the ID, it checks if the serial number of the SCA motor is available.
 * If the serial number is not available, it logs an error message and returns ERROR.
 * After changing the ID, it updates the ID value in the SCA_t structure.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param id The new ID to be set for the SCA motor.
 *
 * @return Returns OK if the ID is successfully set.
 *         Returns ERROR if the sca pointer is NULL or the serial number is not available.
 *         If the ID setting fails, it logs a warning message.
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
        log_w("sca%d ID 设置失败 ", sca->id);
    return result;
}


/*********************************Position******************************************** */


/**
 * @brief Sets the target position for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target position.
 * The target position is a float value representing the desired position in
 * encoder counts. The function checks if the target position is within the
 * valid range and logs a warning if it is not. It also checks if the SCA motor
 * is in the correct mode to set the position and logs a warning if the mode is
 * not appropriate.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL or if the mode is not
 *         appropriate.
 */
uint8_t SCA_SetPosition(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    if((sca->Position_Target * sca->ReductionRatio) < -127.0f || 
       (sca->Position_Target * sca->ReductionRatio) > 127.0f)
        log_w("sca%d 参数超范围 当前参数%.2fR", sca->id, sca->Position_Target);

    if (sca->Mode == SCA_Position_Mode || sca->Mode == SCA_Profile_Position_Mode)
    {
        return SCA_Write_3(sca, W3_Position, sca->Position_Target * sca->ReductionRatio);
    }

    log_w("sca%d 模式不匹配 ", sca->id);
    return ERROR;
}


/**
 * @brief Sets the target angle for the SCA motor.
 *
 * This function converts the target angle from degrees to revolutions,
 * checks if the angle is within the valid range for the SCA motor,
 * and then sends a command to the SCA motor to set its target position.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL or the angle is out of range.
 *         If the SCA motor is not in position mode, it logs a warning message and returns ERROR.
 */
uint8_t SCA_SetAngle(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    float position = sca->Angle_Target / 360.0f * sca->ReductionRatio;

    if(position < -127.0f || position > 127.0f)
    {
        log_w("sca%d 参数超范围 当前参数:%.2f°" ,sca->id, sca->Angle_Target);
    }
    if (sca->Mode == SCA_Position_Mode || sca->Mode == SCA_Profile_Position_Mode)
    {
        return SCA_Write_3(sca, W3_Position, position);
    }

    log_w("sca%d  模式不匹配 ", sca->id);
    return ERROR;
}


/**
 * @brief Retrieves the position of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its position.
 * The position is a float value representing the current
 * position of the motor in encoder counts.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPosition(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Read(sca->can, sca->id, R3_Position);
}


/**
 * @brief Sets the zero angle for the SCA motor.
 *
 * This function is used to set the zero angle for the SCA motor. It writes the homing value to the motor,
 * which is used to determine the zero position. If the angle provided is 0, the homing value is set to 0.
 * Otherwise, the angle is converted to a position value (0 to 1) and written to the motor.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param angle The angle to set as the zero angle for the motor.
 *
 * @return Returns OK if the homing value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetZeroAngle(Sca_t * sca, float angle)
{

    if(sca == NULL)
        return ERROR;
    if (angle == 0)
    {
        return SCA_Write_3(sca, W3_HomingValue, 0);
    }
    else
    {
        float position = angle / 360.0f * sca->ReductionRatio;
        return SCA_Write_3(sca, W3_HomingValue, position);
    }
}

/**
 * @brief Gets the acceleration for the Profile Position Mode.
 *
 * This function reads the acceleration value from the SCA motor's Profile Position Mode
 * and returns it.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPPAcceleration(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R3_PPMaxAcceleration);
}



/**
 * @brief Retrieves the deceleration limit for the Profile Position Mode of the SCA motor.
 *
 * This function reads the deceleration limit value from the SCA motor s Profile Position Mode.
 * The deceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPPDeceleration(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R3_PPMaxDeceleration);
}


/**
 * @brief Sets the acceleration for the Profile Position Mode.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param acce Desired acceleration value in the unit of Profile_Scal.
 *
 * @return Returns OK if the acceleration is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetPPAcceleration(Sca_t * sca, float acce)
{
    if(sca == NULL)
        return ERROR;
    
    uint8_t result = ERROR;
    acce = acce / Profile_Scal * sca->ReductionRatio;

    result = SCA_Write_3(sca, W3_PPMaxAcceleration, acce);
    if (result != OK)   
    {
        log_e("sca%d PPAcceleration 写入失败 ", sca->id);
        return result;
    }

    SCA_GetPPAcceleration(sca);
    return OK;
    
}


/**
 * @brief Sets the deceleration for the Profile Position Mode.
 *
 * This function sets the deceleration value for the Profile Position Mode of the SCA motor.
 * The deceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param dece Desired deceleration value in the unit of Profile_Scal.
 *
 * @return Returns OK if the deceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCa_SetPPDeceleration(Sca_t * sca, float dece)
{
    if(sca == NULL)
        return ERROR;

    uint8_t result = ERROR;
    dece = dece / Profile_Scal * sca->ReductionRatio;
    result = SCA_Write_3(sca, W3_PPMaxDeceleration, dece);
    if (result != OK)
    {
        log_e("sca%d PPDeceleration 写入失败 ",sca->id);
        return ERROR;
    }
    SCA_GetPPDeceleration(sca);
    return OK; 
}



/**
 * @brief Gets the maximum velocity for the Profile Position Mode.
 *
 * This function reads the maximum velocity value from the SCA motor's Profile Position Mode
 * and returns it. The value is scaled by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPPMaxVelocity(Sca_t * sca)
{
    if (sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R3_PPMaxVelocity);
}


/**
 * @brief Sets the maximum velocity for the Profile Position Mode.
 *
 * This function sets the maximum velocity for the Profile Position Mode of the SCA motor.
 * The maximum velocity is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param max_velocity Desired maximum velocity value in the unit of Profile_Scal.
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetPPMaxcVelocity(Sca_t * sca, float max_velocity)
{
    if(sca == NULL)
        return ERROR;
    
    uint8_t result = ERROR;
    uint8_t err_t = 0;
    max_velocity = max_velocity / Profile_Scal * sca->ReductionRatio;

    result = SCA_Write_3(sca, W3_PPMaxVelocity, max_velocity);
    if (result != OK)
    {
        log_e("sca%d PPMaxcVelocity 写速度失败 ", sca->id);
        return result;
    }

    SCA_GetPPMaxVelocity(sca);

    return OK;
}



/*********************************Velocity******************************************** */

/**
 * @brief Sets the target velocity for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target velocity.
 * The target velocity is a float value representing the desired
 * velocity in revolutions per minute (rpm).
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param velocity_target Desired target velocity in revolutions per minute (rpm).
 *
 * @return Returns OK if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Write_3(sca, W3_Velocity, sca->Velocity_Target * sca->ReductionRatio);
}

/**
 * @brief Retrieves the velocity of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its velocity.
 * The velocity is a float value 
 * velocity in revolutions per minute (rpm).
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
  
    return SCA_Read(sca->can, sca->id, R3_Velocity);
}

/**
 * @brief Retrieves the acceleration limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the acceleration limit value from the SCA motor's Profile Velocity Mode.
 * The acceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the acceleration limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPVAcceleration(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R3_PVMaxAcceleration);
}



/**
 * @brief Retrieves the deceleration limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the deceleration limit value from the SCA motor's Profile Velocity Mode.
 * The deceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the deceleration limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPVDeceleration(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R3_PVMaxDeceleration);
}

/**
 * @brief Sets the acceleration for the Profile Velocity Mode.
 *
 * This function sets the acceleration value for the Profile Velocity Mode of the SCA motor.
 * The acceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param acceleration Desired acceleration value in the unit of Profile_Scal.
 *
 * @return Returns OK if the acceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetPVAcceleration(Sca_t * sca, float acceleration)
{
    if(sca == NULL)
        return ERROR;

    uint8_t result = ERROR;
    acceleration = acceleration / Profile_Scal * sca->ReductionRatio;
    result = SCA_Write_3(sca, W3_PVMaxAcceleration, acceleration);
    if (result!= OK)
    {
        log_e("sac%d PVAcceleration 写入失败 ", sca->id);
        return result;
    }
    SCA_GetPVAcceleration(sca);
    return OK;
}


/**
 * @brief Sets the deceleration for the Profile Velocity Mode.
 *
 * This function sets the deceleration value for the Profile Velocity Mode of the SCA motor.
 * The deceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param deceleration Desired deceleration value in the unit of Profile_Scal.
 *
 * @return Returns OK if the deceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetPVDeceleration(Sca_t * sca, float deceleration)
{
    if (sca == NULL)
        return ERROR;

    uint8_t result = ERROR;
    deceleration = deceleration / Profile_Scal * sca->ReductionRatio;
    result = SCA_Write_3(sca, W3_PVMaxDeceleration, deceleration); 
    if (result!= OK)
    {   
        log_e("sca%d PVDeceleration 写入失败 ", sca->id);
        return result;
    } 
    SCA_GetPVDeceleration(sca);
    return OK;
}


/**
 * @brief Retrieves the velocity limit of the SCA motor.
 *
 * This function reads the velocity limit value from the SCA motor's
 * internal registers and returns it.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the velocity limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetVelocityLimit(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R3_VelocityLimit);
}

/**
 * @brief Retrieves the maximum velocity limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the maximum velocity limit value from the SCA motor's Profile Velocity Mode.
 * The maximum velocity limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the maximum velocity limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetPVMaxVelocity(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return  SCA_Read(sca->can, sca->id, R3_PVMaxVelocity);
}


/**
 * @brief Sets the maximum velocity for the Profile Velocity Mode of the SCA motor.
 *
 * This function sets the maximum velocity for the Profile Velocity Mode of the SCA motor.
 * The maximum velocity is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param maxVelocity Desired maximum velocity value in the unit of Profile_Scal.
 *
 * @return Returns OK if the maximum velocity is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 *         If the write operation fails, it logs an error message and returns the error code.
 */
uint8_t SCA_SetPVMaxVelocity(Sca_t * sca, float maxVelocity)
{
    if(sca == NULL)
        return ERROR;

    maxVelocity = maxVelocity / Profile_Scal * sca->ReductionRatio;
    uint8_t result = ERROR;
    result = SCA_Write_3(sca, W3_PVMaxVelocity, maxVelocity);
    if (result!= OK)
    {
        log_e("sca%d PVMaxVelocity 写入失败 ", sca->id);
        return result;
    }
    SCA_GetPVMaxVelocity(sca);
    return OK;
}



/*********************************currents******************************************** */


/**
 * @brief Sets the target current for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target current.
 * The target current is a float value representing the desired
 * current in amperes. The actual current output of the motor may not match the
 * target current exactly due to internal control algorithms.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SetCurrent(Sca_t* sca)
{
    if (sca == NULL)
    {
        return ERROR;
    }
    
    return SCA_Write_3(sca, W3_Current, sca->Current_Target);
}


/**
 * @brief Retrieves the maximum current maximum range of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its maximum current range.
 * The maximum current limit is a float value representing the maximum current in amperes.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetCurrentMaxRange(Sca_t* sca)
{
    if (sca == NULL)
    {
        return ERROR;
    }
    
    return SCA_Read(sca->can, sca->id, R2_Current_Max);
}

uint8_t SCA_GetCurrentLimit(Sca_t* sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R3_CurrentLimit);
    
}

/*********************************others******************************************** */


/**
 * @brief Retrieves the error code from the SCA motor.
 *
 * This function sends a command to the SCA motor to read its error code.
 * The error code is a 16-bit value that represents the status of the motor.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_GetErrorCode(Sca_t* sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Read(sca->can, sca->id, R2_Error);
}


/**
 * @brief Retrieves the CVP value from the SCA motor.
 *
 * This function sends a command to the SCA motor to read its CVP value.
 * The CVP value represents the current, velocity and position value of the motor
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t SCA_GetCVP(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    return SCA_Read(sca->can, sca->id, R4_CVP);
}


/**
 * @brief Retrieves the voltage of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its voltage.
 * The voltage is a float value representing the voltage in volts.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t SCA_GetVoltage(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    return SCA_Read(sca->can, sca->id, R2_Voltage);
}


/**
 * @brief Retrieves the temperature of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its temperature.
 * The temperature is a float value representing the temperature in degrees Celsius.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
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
 * @brief Retrieves all parameters from the SCA motor.
 *
 * This function sends commands to the SCA motor to retrieve various parameters
 * such as serial number, temperature, voltage, mode, and profile velocity
 * parameters. The function then waits for a short period before sending the
 * next command to ensure that the data is ready.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if all commands are successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t SCA_GetAllparameters(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;

    SCA_GetSerialNumbe(sca);
    vTaskDelay(10);
    SCA_GetTemperature(sca);
    vTaskDelay(10);
    SCA_GetVoltage(sca);
    vTaskDelay(10);
    SCA_GetMode(sca);
    vTaskDelay(10);
    SCA_GetPPAcceleration(sca);
    vTaskDelay(10);
    SCA_GetPPDeceleration(sca);
    vTaskDelay(10);
    SCA_GetPPMaxVelocity(sca);
    vTaskDelay(10);
    SCA_GetPVAcceleration(sca);
    vTaskDelay(10);
    SCA_GetPVDeceleration(sca);
    vTaskDelay(10);
    SCA_GetPVMaxVelocity(sca);
    vTaskDelay(10);
    SCA_GetVelocityLimit(sca);
    vTaskDelay(10);
    SCA_GetCurrentMaxRange(sca);
    vTaskDelay(10);
    SCA_GetCurrentLimit(sca);
    return OK;
}




/**
 * @brief Saves all parameters of the SCA motor.
 *
 * This function sends a command to the SCA motor to save all its parameters.
 * The parameters are saved to non-volatile memory, so they will remain after
 * a power cycle.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the save command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t SCA_SaveAllParamters(Sca_t * sca)
{
    if(sca == NULL)
        return ERROR;
    
    return SCA_Write_4(sca->can, sca->id, W4_Save);
}


/****************************data process*******************************/


/**
 * @brief Processes the received data from the SCA motor.
 *
 * This function checks the first byte of the received data to determine the type of data,
 * and then calls the appropriate data processing function to handle the data.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns OK if the function completes successfully.
 *         Returns ERROR if the sca pointer is NULL.
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
        case R2_Current_Max:
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
 * @brief Initializes the SCA motor.
 *
 * This function initializes the SCA motor with the given parameters.
 * It sets the motor's ID, target mode, CAN interface, and other necessary variables.
 * It then enables the motor, gets its state, sets the target mode, and initializes the model parameters.
 * If the current maximum range cannot be obtained within 20 attempts, it logs an error message and returns an error.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param model The model of the SCA motor.
 * @param id The ID of the SCA motor.
 * @param workmode The target mode of the SCA motor.
 * @param can Pointer to the CAN interface used for communication with the SCA motor.
 *
 * @return Returns OK if the initialization is successful.
 *         Returns ERROR if the sca pointer or can pointer is NULL, or if the current maximum range cannot be obtained within 20 attempts.
 */
uint8_t ScaInit(Sca_t* sca, sca_model_e model, uint8_t id, uint8_t workmode, Can_t* can)
{

    if(sca == NULL || can == NULL)
        return ERROR;

    uint8_t err_t = 0;
    sca->id = id;
    sca->Mode_Target = workmode;
    sca->Mode = 0x00;
    sca->can = can;
    sca->Online_State = 1;
    sca->Power_State = 0;
    sca->Current_MaxRange = 0;
    // sca->ptErrcode->Error_Code = 0x0000;
    sca->Error_Code = 0x00;

    SCA_Enable(sca);
    SCA_GetState(sca);
    vTaskDelay(10);
    SCA_SetMode(sca, sca->Mode_Target);
    SCA_InitModelParameters(sca, model);
    while (sca->Current_MaxRange == 0)
    {
        err_t++;
        SCA_GetCurrentMaxRange(sca);
        if (err_t >= 20)
        {
            log_e("sca%d电流量程获取失败 ", sca->id);
            return ERROR;
        }
        vTaskDelay(100);
    }
    return OK;
}


/************************************user code************************************************************* */

typedef struct
{
    uint8_t id;
    char * const name;
}index_name_t;

index_name_t mode_name[] = {
    {.id = 0x01, .name = "Current_Mode"},
    {.id = 0x02, .name = "Velocity_Mode"},
    {.id = 0x03, .name = "Position_Mode"},
    {.id = 0x06, .name = "Profile_Position_Mode"},
    {.id = 0x07, .name = "Profile_Velocity_Mode"},
    {.id = 0x08, .name = "SCA_Homing_Mode"},
};

index_name_t error_name[] = {
    {.id = 0x00, .name = "over voltage fault"},
    {.id = 0x01, .name = "under voltage fault"},
    {.id = 0x02, .name = "lock rotor fault"},
    {.id = 0x03, .name = "over temperature fault"},
    {.id = 0x04, .name = "r/w parameter fault"},
    {.id = 0x05, .name = "mul circle count fault"},
    {.id = 0x06, .name = "inv temperature sensor fault"},
    {.id = 0x07, .name = "can bus fault"},
    {.id = 0x08, .name = "motor temperature sensor fault"},
    {.id = 0x09, .name = "over step fault"},
    {.id = 0x0A, .name = "drv protect fault"},
    {.id = 0x0B, .name = "device fault"}
};

char* find_name_of_mode(uint8_t index)
{
    for (uint8_t i = 0; i < sizeof(mode_name) / sizeof(mode_name[0]); i++)
    {
        if (mode_name[i].id == index)
        {
            return mode_name[i].name;
        }
    }
    return NULL;
}

char * find_name_of_error(uint8_t index)
{
    for (uint8_t i = 0; i < sizeof(error_name) / sizeof(error_name[0]); i++)
    {
        if (error_name[i].id == index)
        {
            return error_name[i].name;
        }
    }
    return NULL;
}


void show_sca_t(uint8_t index)
{
    SCA_GetAllparameters(&motor[index]);
    vTaskDelay(5);
    log_v("----------motor %d----------", index + 1);
    log_v("work mode:\t\t%s", find_name_of_mode(motor[index].Mode));
    log_v("reduction ratio rate:\t%.1f", motor[index].ReductionRatio);
    log_v("target current:\t\t%.2f", motor[index].Current_Target);
    log_v("target velocity:\t%.2frpm", motor[index].Velocity_Target);
    log_v("target position:\t%.2fR", motor[index].Position_Target);
    log_v("real current:\t\t%.2fA", motor[index].Current_Real);
    log_v("real velocity:\t\t%.2frpm", motor[index].Velocity_Real);
    log_v("real position:\t\t%.2fR", motor[index].Position_Real);
    log_v("serial numbe:\t\t%02x %02x %02x %02x %02x %02x", motor[index].Serial_Num[0], motor[index].Serial_Num[1],motor[index].Serial_Num[2],motor[index].Serial_Num[3],motor[index].Serial_Num[4], motor[index].Serial_Num[5]);
    log_v("power voltage:\t\t%.2fV", motor[index].Voltage);
    log_v("velocity limit:\t\t%.2fRPM", motor[index].Velocity_Limit);
    log_v("motor temperture:\t%.2f°C", motor[index].Motor_Temp);
    log_v("pp acceleration:\t%.2f", motor[index].PP_Max_Acceleration);
    log_v("pp deceleration:\t%.2f", motor[index].PP_Max_Deceleration);
    log_v("pp max velocity:\t%.2fRPM", motor[index].PP_Max_Velocity);
    log_v("pv acceleration:\t%.2f", motor[index].PV_Max_Acceleration);
    log_v("pv deceleration:\t%.2f", motor[index].PV_Max_Deceleration);
    log_v("pv max velocity:\t%.2fRPM", motor[index].PV_Max_Velocity);
    log_v("max current range:\t%.2fA", motor[index].Current_MaxRange);
    log_v("limit current:\t\t%.2fA", motor[index].Current_Limit);
}
