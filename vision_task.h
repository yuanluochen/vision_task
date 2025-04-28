/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
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
#include "referee.h"
#include "remote_control.h"

/*人工修正的参数*/

//重力加速度
#define GRAVITY 9.75225639f
//当前弹速--写函数(不带"";"的函数)/写常数
#define CUR_BULLET_SPEED 24.0f
//云台转轴中心到枪口的竖直距离
#define Z_STATIC 0.0f
//云台转轴中心到发射最大初速度点的距离
#define DISTANCE_STATIC 0.0483f
//空气阻力系数
#define AIR_K1 0.15f

/*           */


//允许发弹距离 m
#define ALLOW_ATTACK_DISTANCE 4.5f
//允许发弹概率
#define ALLOE_ATTACK_P 5.0f

//固有时间偏移
#define TIME_BIAS 6

//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 1

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 1

//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//机器人红蓝id分界值，大于该值则机器人自身为蓝色，小于这个值机器人自身为红色
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100


//最小设定弹速
#define MIN_SET_BULLET_SPEED 19.0f
//最大设定弹速
#define MAX_SET_BULLET_SPEED 25.0f
//初始设定弹速
#define BEGIN_SET_BULLET_SPEED 23.8f


//大装甲板宽度
#define SMALL_ARMOR_WIDTH 0.135f
//小装甲板宽度
#define LARGE_ARM0R_WIDTH 0.230f

//初始子弹飞行迭代数值
#define T_0 0.0f
//迭代精度
#define PRECISION 0.0001f
//最小迭代差值
#define MIN_DELTAT 0.001f
//最大迭代次数
#define MAX_ITERATE_COUNT 30

//比例补偿器比例系数
#define ITERATE_SCALE_FACTOR 0.9f

//ms转s
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)

#endif // !TIME_MS_TO_S(x)

//全圆弧度
#define ALL_CIRCLE (2 * PI)

//初始飞行时间
#define INIT_FILIGHT_TIME 0.5f


//最大未接受数据的时间 s
#define MAX_NOT_RECEIVE_DATA_TIME 3.0f


//子弹类型
typedef enum
{
    BULLET_17 = 1,
    BULLET_42 = 2,
}bullet_type_e;

// 发射枪管id
typedef enum
{
    SHOOTER_17_1 = 1,
    SHOOTER_17_2 = 2,
    SHOOTER_42 = 3,
} shooter_id_e;

//接收数据状态
typedef enum
{
    //未读取
    UNLOADED,
    //已读取
    LOADED,
}receive_state_e;


//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//装甲板颜色
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
    //全部机器人
    ARMOR_ALL_ROBOT = 8,
}armor_id_e;

//哨兵发射命令
typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

// 视觉目标状态
typedef enum
{
    TARGET_UNAPPEAR, // 未识别到目标
    TARGET_APPEAR,   // 识别到目标
} vision_target_appear_state_e;

//向量结构体
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
} vector_t;

// 发送数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
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

// 接收数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 装甲板数量 2-balance 3-outpost 4-normal
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
    float p;   //状态协方差矩阵的迹
    uint16_t checksum;
} receive_packet_t;

//类型转化
typedef receive_packet_t target_data_t;

// 视觉接收结构体
typedef struct
{
    // 接收状态位
    uint8_t receive_state : 1;
    // 上次接收数据的时间
    fp32 last_receive_time;
    // 当前接受数据时间
    fp32 current_receive_time;

    // 当前时间 -- 用于计算是否长时间未接受
    fp32 current_time;
    //  间隔时间
    fp32 interval_time;
    // 接收数据包
    receive_packet_t receive_packet;
} vision_receive_t;

// 哨兵云台电机运动命令,经滤波处理后的数值
typedef struct
{
    // 本次云台yaw轴数值
    fp32 gimbal_yaw;
    // 本次云台pitch轴数值
    fp32 gimbal_pitch;
    //角速度前馈
    fp32 feed_forward_omega;
} gimbal_vision_control_t;

// 哨兵发射电机运动控制命令
typedef struct
{
    // 自动发射命令
    shoot_command_e shoot_command;
} shoot_vision_control_t;


// 目标位置结构体
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;




//弹道计算结构体
typedef struct
{
    // 当前弹速
    fp32 current_bullet_speed;
    // 当前pitch
    fp32 current_pitch;
    // 当前yaw
    fp32 current_yaw;
    // 弹道系数
    fp32 k1;
    //子弹飞行时间
    fp32 flight_time;
    // 固有间隔时间
    fp32 time_bias;
    //预测时间
    fp32 predict_time;

    // 目标yaw
    fp32 target_yaw;

    // 装甲板数量
    uint8_t armor_num;

    //IMU到yaw轴电机的竖直距离
    fp32 z_static;
    //枪口前推距离
    fp32 distance_static;

    //所有装甲板位置
    target_position_t all_target_position_point[4];

} solve_trajectory_t;

//均值弹速
#define BULLET_SPEED_SIZE 5
typedef struct{
    fp32 bullet_speed[BULLET_SPEED_SIZE];
    fp32 est_bullet_speed;
    int full_flag;
    int pos;
}bullet_speed_t;

// 视觉任务结构体
typedef struct
{
    // 绝对角指针
    const INS_t* vision_angle_point;
    // 弹速
    bullet_speed_t bullet_speed;
    // 检测装甲板的颜色(敌方装甲板的颜色)
    uint8_t detect_armor_color;

    //目标数据
    target_data_t target_data;
    //上次目标数据
    target_data_t last_target_data;

    //弹道解算
    solve_trajectory_t solve_trajectory;
    //目标位置
    target_position_t target_position;


    const ext_shoot_data_t* shoot_data_point;

    // 机器人云台瞄准位置向量
    vector_t robot_gimbal_aim_vector;

    //接收的数据包指针
    vision_receive_t* vision_receive_point;
    //发送数据包
    send_packet_t send_packet;

    // 视觉目标状态
    vision_target_appear_state_e vision_target_appear_state;
    // 目标装甲板编号
    armor_id_e target_armor_id;
    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;
    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;



} vision_control_t;

// 视觉数据处理任务
void vision_task(void const *pvParameters);

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// 获取上位机跟随命令
const uint8_t *get_vision_target_follow_point(void);

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void);



/**
 * @brief 判断视觉是否识别到目标
 *
 * @return bool_t 返回1 识别到目标 返回0 未识别到目标
 */
bool_t judge_vision_appear_target(void);



/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 *
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 * @param target_distance 目标距离
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance);

/**
 * @brief 接收数据解码
 *
 * @param buf 接收到的数据
 * @param len 接收到的数据长度
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief 将数据包通过usb发送到nuc
 *
 * @param send 发送数据包
 */
void send_packet(vision_control_t* send);



extern vision_control_t vision_control;
extern vision_receive_t vision_receive;
extern uint8_t follow;

#endif // !VISION_TASK_H
