#include "ne30_task.h"
#include "innfos_ne30.h"
#include "global.h"
#include "system.h"
#include "my_fdcan.h"
#include "myusb_cdc.h"

#define CURRENT_BUFFER_SIZE 100
Weight_Filter_t current_filter[MOTOR_NUM] = {0};
float *current_buffer[MOTOR_NUM] = {NULL, NULL};
float current_weight[CURRENT_BUFFER_SIZE] = {0};

static void current_weight_init(void)
{
    for (uint32_t i = 0; i < CURRENT_BUFFER_SIZE; i++)
    {
        current_weight[i] = i *0.1;
    }
}


static void motor_init(void)
{
    static uint8_t err_t[MOTOR_NUM] = {0};

    /* 电机1初始化 */
    while (motor[0].Power_State != 1)
    {
        err_t[0]++;
        SCA_Enable(&motor[0]);
        SCA_GetState(&motor[0]);
        vTaskDelay(500);
        if (err_t[0] >= 5){//3s后报警一次
            log_e("sca %d is not online", motor[0].id);
            return;
        }
    }
    log_i("sca%d power on and work in %s", motor[0].id, find_name_of_mode(motor[0].Mode));
    SCA_SetMode(&motor[0], SCA_Profile_Velocity_Mode);
    SCA_GetAllparameters(&motor[0]);

    /* 电机2初始化 */
    while (motor[1].Power_State != 1)
    {
        err_t[1]++;
        SCA_Enable(&motor[1]);
        SCA_GetState(&motor[1]);
        vTaskDelay(500);
        if (err_t[1] >= 5){//3s后报警一次
            log_e("sca %d is not online", motor[1].id);
            return;
        }
    }
    log_i("sca%d power on and work in %s", motor[1].id, find_name_of_mode(motor[1].Mode));
    SCA_SetMode(&motor[1], SCA_Profile_Velocity_Mode);
    SCA_SetPPMaxcVelocity(&motor[1], 30);
    SCA_GetAllparameters(&motor[1]);
}

/**
 * @brief 切换到期望模式
 * @param sca 
 */
static void motor_mode_update(Sca_t * sca)
{
    if(sca->Mode_Target != sca->Mode)
    {
        SCA_SetMode(sca, sca->Mode_Target);
    }
}

/**
 * @brief 根据当前模式进行控制
 * @param sca 
 */
static void motor_control_update(Sca_t * sca)
{
    switch ( sca->Mode)
    {
    case SCA_Current_Mode:
        SCA_SetCurrent(sca);
        break;
    case SCA_Profile_Position_Mode:
        SCA_SetPosition(sca);
        break;
    case SCA_Profile_Velocity_Mode:
        SCA_SetVelocity(sca);
        break;
    default:
        break;
    }
}

/**
 * @brief 单个电机停止
 * @param sca 
 */
static void motor_stop(Sca_t * sca)
{
    SCA_Disable(sca);
}

/**
 * @brief 所有电机停止
 */
static void motor_stop_all(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        SCA_Disable(&motor[i]);
    }
}

/**
 * @brief 电机掉线检查
 * @param  
 */
static void motor_online_check(void)
{
    static uint8_t err_t[MOTOR_NUM] = {0};
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (motor[i].Online_State == 0){
            err_t[i]++;
        }
        else{
            err_t[i] = 0; 
            motor[i].Online_State = 0;//重置心跳
            SCA_HeartBeat(&motor[i]);//获取心跳
        }

        if (err_t[i] >= 3){
            log_e("sca%d is not online", &motor[i].id);
            motor_stop_all();
            system_reset_soft("motor %d is not online", &motor[i].id);
        }       
    }
}

static void motor_error_check(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        SCA_GetErrorCode(&motor[i]);
        // for (uint8_t i = 0; i < 12; i++){
        //     if(motor[i].ptErrcode->Error_Code & (0x01 << i)){
        //         log_e("motor %d :%s", motor[i].id, find_name_of_error(i));
        //     }
        // }
        log_i("motor %d err: %x", motor[i].id, motor[i].Error_Code);
    }
}

/**
 * @brief sca control task function
 * @param pvparameters 
 */
static void sca_control_task(void * pvparameters)
{
    ScaInit(&motor[0], Lite_NE30_36, 0X15, SCA_Profile_Position_Mode, &fdcan1);//sca 句柄初始化
    ScaInit(&motor[1], NE30,         0x02, SCA_Current_Mode, &fdcan1);
    motor_init();//电机初始化
    current_buffer[0] = (float *)pvPortMalloc(sizeof(float) * CURRENT_BUFFER_SIZE);
    current_buffer[1] = (float *)pvPortMalloc(sizeof(float) * CURRENT_BUFFER_SIZE);
    current_weight_init();
    wfilter_init(&current_filter[0], current_buffer[0], current_weight, CURRENT_BUFFER_SIZE);
    wfilter_init(&current_filter[1], current_buffer[1], current_weight, CURRENT_BUFFER_SIZE);
    float current_cal[MOTOR_NUM] = {0, 0};//滤波后的电流值
    log_i("SCA Task Work! ");  
    for(;;)
    {       
        motor_mode_update(&motor[0]);//先切换到正确模式
        motor_mode_update(&motor[1]);
        motor_control_update(&motor[0]);//根据模式执行控制
        motor_control_update(&motor[1]);
        SCA_GetCVP(&motor[0]);
        SCA_GetCVP(&motor[1]);
        wfilter_add_data(&current_filter[0], motor[0].Current_Real);//添加采集数据
        wfilter_add_data(&current_filter[1], motor[1].Current_Real);
        current_cal[0] = wfilter_cal(&current_filter[0]);//计算滤波值
        current_cal[1] = wfilter_cal(&current_filter[1]);
        usb_cdc_println("%.2f, %.2f, %.2f, %.2f", current_cal[0], current_cal[0], motor[0].Position_Real, motor[1].Position_Real);
        motor[0].Position_Target = motor[1].Position_Real;
        motor[1].Current_Target = motor[0].Current_Real * 0.7;
        vTaskDelay(10);
    }
}

/**
 * @brief sca data process task function
 * @param pvparameters 
 */
void sca_data_process_task(void *pvparameters)
{
    for(;;)
    {
        xSemaphoreTake(fdcan1.rx_sema, portMAX_DELAY);
        for (uint8_t i = 0; i < MOTOR_NUM; i++)
        {
            if(fdcan1.rx_header_t->Identifier == motor[i].id)
            {
                SCA_DataProcess(&motor[i]);
                break;
            }
        }
        
    }
}

/**
 * @brief 1s callback
 * @param xTimer 
 */
void sca_tim_callback(TimerHandle_t xTimer)
{
    motor_online_check();  
    // motor_error_check();
}


/**
 * @defgroup sca control task相关参数
 */
#define SCA_CONTROL_TASK_PRIO 3  //任务优先级
#define SCA_CONTROL_TASK_SIZE 256 //任务堆栈大小
TaskHandle_t SCA_CONTROL_TASK_Handler;//任务句柄
void sca_control_task(void *pvparameters);

/**
 * @defgroup sca data process task相关参数
 */
#define SCA_DATA_PROCESS_TASK_PRIO 5
#define SCA_DATA_PROCESS_TASK_SIZE 256
TaskHandle_t SCA_DATA_PROCESS_TASK_Handler;
void sca_data_process_task(void *pvparameters);

/**
 * @defgroup sca timer 
 */
TimerHandle_t SCA_TIMER;
void sca_tim_callback(TimerHandle_t xTimer);

/**
 * @brief sca task initialization
 */
void sca_task_init(void)
{
    can_init(&fdcan1, &hfdcan1);
    xTaskCreate((TaskFunction_t )sca_control_task,           //任务函数
                (const char *   )"sca control",              //任务名称
                (uint16_t       )SCA_CONTROL_TASK_SIZE,      //任务堆栈大小
                (void *         )NULL,                       //任务参数
                (UBaseType_t    )SCA_CONTROL_TASK_PRIO,      //任务优先级
                (TaskHandle_t * )&SCA_CONTROL_TASK_Handler); //任务句句柄

    xTaskCreate((TaskFunction_t )sca_data_process_task,      //任务函数
                (const char *   )"sca data process",         //任务名称
                (uint16_t       )SCA_DATA_PROCESS_TASK_SIZE, //任务堆栈大小
                (void *         )NULL,                       //任务参数
                (UBaseType_t    )SCA_DATA_PROCESS_TASK_PRIO, //任务优先级
                (TaskHandle_t * )&SCA_CONTROL_TASK_Handler); //任务句句柄

    SCA_TIMER = xTimerCreate((const char* const)"heartbeat",
                             (TickType_t       )1000,
                             (const UBaseType_t)pdTRUE,
                             (void * const     )NULL,
                             (TimerCallbackFunction_t)sca_tim_callback);
    xTimerStart(SCA_TIMER, 0);
}