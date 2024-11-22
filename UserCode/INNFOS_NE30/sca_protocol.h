#ifndef __SCA_PROTOCOL_H
#define __SCA_PROTOCOL_H

#include "my_fdcan.h"

#define Actr_Enable     0x01
#define Actr_Disable    0x00



/* ���������º궨����Ϣ���������޸ģ����� */

//INNFOS CAN ͨ��Э��ָ��
//��һ���ȡָ��
#define R1_Heartbeat			0x00        //����        
#define R1_Mode					0x55        //��ѯģʽ
#define R1_PositionLimitState	0x8B        //��ѯλ����λʹ��/ʹ��
#define R1_PowerState			0x2B        //��ѯִ����ʹ��/ʹ��

//�ڶ����ȡָ��
#define R2_Voltage				0x45        //��ѯִ������ѹ(����Ϊ��ʵֵ��2^10��)
#define R2_Current_Max			0x53        //��ѯ����������
#define R2_MotorTemp			0x5F        //��ѯ����¶�
#define R2_MotorProtectTemp		0x6C        //��ѯ��������¶�
#define R2_MotorRecoverTemp		0x6E        //��ѯ����ָ��¶�
#define R2_Error				0xFF        //��ѯִ�����ı�����Ϣ

//�������ȡָ��
#define R3_Current				0x04        //��ѯִ��������ֵ(���ݳ���������Ϊ��ʵ����ֵ)
#define R3_Velocity				0x05        //��ѯִ�����ٶ�ֵ(��λ��RPM)
#define R3_Position				0x06        //��ѯִ����λ��ֵ(��λ��R)
#define R3_PPMaxVelocity		0x1C        //��ѯλ��������������ٶ�
#define R3_PPMaxAcceleration	0x1D        //��ѯλ���������������ٶ�
#define R3_PPMaxDeceleration	0x1E        //��ѯλ���������������ٶ�
#define R3_PVMaxVelocity		0x22        //��ѯ�ٶ��������ߵ�����ٶ�
#define R3_PVMaxAcceleration	0x23        //��ѯ�ٶ��������ߵ���Ӵ��ٶ�
#define R3_PVMaxDeceleration	0x24        //��ѯ�ٶ��������ߵ�������ٶ�
#define R3_CurrentLimit			0x59        //��ѯ��������ֵ(���ݳ���������Ϊ��ʵ����ֵ)
#define R3_VelocityLimit		0x5B        //��ѯ��ѯ�ٶ�����ֵ
#define R3_PositionLimitH		0x85        //��ѯλ�õ�����ֵ
#define R3_PositionLimitL		0x86        //��ѯλ�õ�����ֵ
#define R3_PositionOffset		0x8A        //��ѯ��ѯλ��ƫ��ֵ
#define R3_BlockEngy			0x7F        //��ѯ��ת����

//�������ȡָ��
#define R4_CVP					0x94        //��ȡִ������·�ٶ�λ��

//�������ȡָ��
#define R5_ShakeHands			0x02        //��ѯִ���������к�


//��һ��д������
#define W1_Mode					0x07
#define W1_CurrentFilterState	0X70
#define W1_VelocityFilterState	0x74
#define W1_PositionFilterState	0x78
#define W1_PositionLimitState	0x8C
#define W1_PowerState			0x2A




//������д������
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

//������д������
#define W4_ClearError			0xFE
#define W4_ClearHome			0x88
#define W4_Save					0x0D

//������д������
#define W5_ChangeID				0x3D


//��������ֵ����
// #define Velocity_Max	6000.0f			//�ٶ����ֵ���̶�Ϊ6000RPM������Ϊ�����ã�
#define BlkEngy_Scal	75.225f			//��ת��������ֵ
#define Profile_Scal	960.0f			//���β�������ֵ
#define IQ8				256.0f			//2^8
#define IQ10			1024.0f			//2^10
#define IQ24			16777216.0f		//2^24
#define IQ30			1073741824.0f	//2^30



uint8_t SCA_Write_1(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData);
uint8_t SCA_Write_4(Can_t * can, uint32_t id, uint8_t cmd);
uint8_t SCA_Write_5(Can_t * can, uint32_t id, uint8_t cmd, uint8_t TxData, uint8_t * serial_num);
uint8_t SCA_Read(Can_t * can, uint32_t id, uint8_t cmd);



#endif
