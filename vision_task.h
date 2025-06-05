/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
 * @version 0.1
 * @date 2023-03-11
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "dma.h"
#include "INS_task.h"
#include "arm_math.h"
#include "remote_control.h"
#include "referee.h"

/*  **�˹������Ĳ���**  */

//��̨ת�����ĵ�ǹ�ڵ���ֱ����
#define Z_STATIC -0.06f
//��̨ת�����ĵ����������ٶȵ�ľ���
#define DISTANCE_STATIC 0.053f
//ǹ��pitch��(�������ȷ��װ�����)
#define PITCH_STATIC 0.0f
//�ӵ����� С���� 0������ 1, ������� 2
#define BULLET_TYPE 0

//��̨yaw��ģʽ����0����ģʽ����1��ֹ��׼�м�ģʽ
#define AUTO_GIMBAL_YAW_MODE 0
// ѡ��װ�װ�ķ�����0 ˳��ѡ����1ӭ��ѡ
#define SELECT_ARMOR_DIR 1

//�������ٶ�
#define GRAVITY 9.8f

//����ʱ��ƫ��--��Ҫ�ǲ����Ӿ��ӳ�+�����ӵ����ӳ�
#define TIME_BIAS 15

//���������� m
#define ALLOW_ATTACK_DISTANCE 10.0f
//��������ekf����ֵ
#define ALLOE_ATTACK_P 2.0f

/*    **      **     */

//װ�װ�ѡ����ٶ���ֵ
#define SELECT_ARMOR_V_YAW_THRES 1.5f
//��ʱ�ȴ�
#define VISION_SEND_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_SEND_CONTROL_TIME_MS 1

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 450
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 1

#if (BULLET_TYPE == 0)
// ��������ϵ���򻯰�
#define AIR_K1 0.019//0.2f
#elif (BULLET_TYPE == 1)
// ��������ϵ���򻯰�
#define AIR_K1 0.00556
#else
// ��������ϵ���򻯰�
#define AIR_K1 0.012
#endif

//������ת�Ƕ���
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//�����˺���id�ֽ�ֵ�����ڸ�ֵ�����������Ϊ��ɫ��С�����ֵ����������Ϊ��ɫ
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100.0f

//��С�趨����
#define MIN_SET_BULLET_SPEED 20.1f
//����趨����
#define MAX_SET_BULLET_SPEED 25.0f
//��ʼ�趨����
#define BEGIN_SET_BULLET_SPEED 22.3f

//��װ�װ���
#define SMALL_ARMOR_WIDTH 0.135f
//Сװ�װ���
#define LARGE_ARM0R_WIDTH 0.230f
//װ�װ�߶�
#define ARMOR_HIGH 0.120f

//��ʼ�ӵ����е�����ֵ
#define T_0 0.0f
//��������
#define PRECISION 0.0001f
//��С������ֵ
#define MIN_DELTAT 0.001f
//����������
#define MAX_ITERATE_COUNT 30
//��������������ϵ��
#define ITERATE_SCALE_FACTOR 0.9f
//RK4��������--Խ��Խ��׼
#define RK_ITER 60

//msתs
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)
#endif // !TIME_MS_TO_S(x)

//ȫԲ����
#define ALL_CIRCLE (2 * PI)
//��ʼ����ʱ��
#define INIT_FILIGHT_TIME 0.5f
//���δ�������ݵ�ʱ�� s
#define MAX_NOT_RECEIVE_DATA_TIME 3.0f


//�ӵ�����
typedef enum
{
    BULLET_17 = 1,
    BULLET_42 = 2,
}bullet_type_e;

// ����ǹ��id
typedef enum
{
    SHOOTER_17_1 = 1,
    SHOOTER_17_2 = 2,
    SHOOTER_42 = 3,
} shooter_id_e;

//��������״̬
typedef enum
{
    //δ��ȡ
    UNLOADED,
    //�Ѷ�ȡ
    LOADED,
}receive_state_e;


//������ʼ֡����
typedef enum
{
    //��λ�����͵���λ��
    LOWER_TO_HIGH_HEAD = 0x5A,
    //��λ�����͵���λ��
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//װ�װ���ɫ
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;


typedef enum
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7,
    //ȫ��������
    ARMOR_ALL_ROBOT = 8,
}armor_id_e;

//�ڱ���������
typedef enum
{
    SHOOT_ATTACK,       // Ϯ��
    SHOOT_READY_ATTACK, // ׼��Ϯ��
    SHOOT_STOP_ATTACK,  // ֹͣϮ��
} shoot_command_e;

// �Ӿ�Ŀ��״̬
typedef enum
{
    TARGET_UNAPPEAR, // δʶ��Ŀ��
    TARGET_APPEAR,   // ʶ��Ŀ��
} vision_target_appear_state_e;

//�����ṹ��
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 r;
} vector_t;

// �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} send_packet_t;

// �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // װ�װ����� 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    float p;   //״̬Э�������ļ�
    float latency_time;
    uint16_t checksum;
} receive_packet_t;

//����ת��
typedef receive_packet_t target_data_t;

// �Ӿ����սṹ��
typedef struct
{
    // ����״̬λ
    uint8_t receive_state : 1;
    // �ϴν������ݵ�ʱ��
    fp32 last_receive_time;
    // ��ǰ��������ʱ��
    fp32 current_receive_time;

    // ��ǰʱ�� -- ���ڼ����Ƿ�ʱ��δ����
    fp32 current_time;
    //  ���ʱ��
    fp32 interval_time;
    // �������ݰ�
    receive_packet_t receive_packet;
} vision_receive_t;

// �ڱ���̨����˶�����,���˲���������ֵ
typedef struct
{
    // ������̨yaw����ֵ
    fp32 gimbal_yaw;
    // ������̨pitch����ֵ
    fp32 gimbal_pitch;
    //���ٶ�ǰ��
    fp32 feed_forward_omega;
} gimbal_vision_control_t;

// �ڱ��������˶���������
typedef struct
{
    // �Զ���������
    shoot_command_e shoot_command;
} shoot_vision_control_t;


// Ŀ��λ�ýṹ��
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;


//��������ṹ��
typedef struct
{
    // ��ǰ����
    fp32 current_bullet_speed;
    // ����ϵ��
    fp32 k1;
    //�ӵ�����ʱ��
    fp32 flight_time;
    // ���м��ʱ��
    fp32 time_bias;
    //Ԥ��ʱ��
    fp32 predict_time;

    // Ŀ��yaw
    fp32 target_yaw;
    // װ�װ�����
    uint8_t armor_num;
    //IMU��yaw��������ֱ����
    fp32 z_static;
    //ǹ��ǰ�ƾ���
    fp32 distance_static;
    //����ǹ�ܰ�װ���������pitch�İ�װ���
    fp32 pitch_static;

    //����װ�װ�λ��
    target_position_t all_target_position_point[4];

} solve_trajectory_t;

//��ֵ����
#define BULLET_SPEED_SIZE 5
typedef struct{
    fp32 bullet_speed[BULLET_SPEED_SIZE];
    fp32 est_bullet_speed;
}bullet_speed_t;

// �Ӿ�����ṹ��
typedef struct
{
    // ���Խ�ָ��
    const INS_t* vision_angle_point;
    // ����
    bullet_speed_t bullet_speed;
    // ���װ�װ����ɫ(�з�װ�װ����ɫ)
    uint8_t detect_armor_color;
    

    //Ŀ������
    target_data_t target_data;
    //�ϴ�Ŀ������
    target_data_t last_target_data;

    //��������
    solve_trajectory_t solve_trajectory;

    // ��������̨��׼λ������
    vector_t robot_gimbal_aim_vector;
    // �Ի���������Ϊԭ���ڹ���ϵ�µз������˵�yaw��
    fp32 body_to_enemy_robot_yaw;
    // ��̨����׼װ�װ�ĽǶ�
    gimbal_vision_control_t aim_armor_angle;

    //���յ����ݰ�ָ��
    vision_receive_t* vision_receive_point;
    //�������ݰ�
    send_packet_t send_packet;

    // �Ӿ�Ŀ��״̬
    vision_target_appear_state_e vision_target_appear_state;
    // ��̨����˶�����
    gimbal_vision_control_t gimbal_vision_control;
    // ���������������
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

// �Ӿ����ݴ�������
void vision_task(void const *pvParameters);

/**
 * @brief �������ݽ���
 *
 * @param buf ���յ�������
 * @param len ���յ������ݳ���
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief �����ݰ�ͨ��usb���͵�nuc
 *
 * @param send �������ݰ�
 */
void send_packet(vision_control_t* send);

extern vision_control_t vision_control;
extern vision_receive_t vision_receive;

/*       ����ӿ�      */

// ��ȡ��λ����̨����
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// ��ȡ��λ����������
const shoot_vision_control_t *get_vision_shoot_point(void);

/**
 * @brief �ж��Ӿ��Ƿ�ʶ��Ŀ��
 *
 * @return bool_t ����1 ʶ��Ŀ�� ����0 δʶ��Ŀ��
 */
bool_t judge_vision_appear_target(void);

/*    **      **     */

#endif // !VISION_TASK_H
