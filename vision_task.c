/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
 * @version 0.1
 * @date 2023-03-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "vision_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "referee.h"
// #include <stdio.h>
// #include <string.h>


uint8_t target_appear_flag = 0;

// 视觉任务初始化
static void vision_task_init(vision_control_t* init);
// 视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
// 设置目标装甲板颜色
static void vision_set_target_armor_color(vision_control_t* set_detect_color, robot_armor_color_e enemy_armor_color);
// 判断是否识别到目标
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
// 配置发送数据包
static void set_vision_send_packet(vision_control_t* set_send_packet);



// 初始化弹道解算的参数
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static);
// 选择最优击打目标
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
// 赋值云台瞄准位置
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// 计算弹道落点 -- 单方向空间阻力模型
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 计算弹道落点 -- 完全空气阻力模型
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 二维平面弹道模型，计算pitch轴的高度
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode);
//更新弹速，并估计当前弹速
static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed);

// 获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);

// 视觉任务结构体
vision_control_t vision_control = { 0 };
// 视觉接收结构体
vision_receive_t vision_receive = { 0 };

fp32 time_bias = 15.0f;

void vision_task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    // 视觉任务初始化
    vision_task_init(&vision_control);
    // 等待云台射击初始化完成

    // 系统延时
    vTaskDelay(VISION_CONTROL_TIME_MS);

    while(1)
    {
        // 更新数据
        vision_task_feedback_update(&vision_control);
        // 设置目标装甲板颜色
        vision_set_target_armor_color(&vision_control, BLUE);
        // 判断是否识别到目标
        vision_judge_appear_target(&vision_control);
        // 处理上位机数据,计算弹道的空间落点，并反解空间绝对角,并设置控制命令
        vision_data_process(&vision_control);

        // 配置发送数据包
        set_vision_send_packet(&vision_control);
        // 发送数据包
        send_packet(&vision_control);

        // 系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_task_init(vision_control_t* init)
{
    // 获取陀螺仪绝对角指针
    init->vision_angle_point = get_INS_point();
    // 获取接收数据包指针
    init->vision_receive_point = get_vision_receive_point();

    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //初始化一些基本的弹道参数
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, TIME_MS_TO_S(time_bias), Z_STATIC, DISTANCE_STATIC);
    //初始化弹速
    init->bullet_speed.pos = 0;
    init->bullet_speed.est_bullet_speed = BEGIN_SET_BULLET_SPEED;
    init->bullet_speed.full_flag = 0;
    memset(init->bullet_speed.bullet_speed, 0, sizeof(init->bullet_speed.bullet_speed[0]) * BULLET_SPEED_SIZE);
    //初始化视觉目标状态为未识别到目标
    init->vision_target_appear_state = TARGET_UNAPPEAR;
    
    //更新数据
    vision_task_feedback_update(init);
}
// extern shoot_date;

static void vision_task_feedback_update(vision_control_t* update)
{
    //更新弹速
    update_bullet_speed(&update->bullet_speed, CUR_BULLET_SPEED);
    update->solve_trajectory.current_bullet_speed = update->bullet_speed.est_bullet_speed;
    //获取目标数据
    if (update->vision_receive_point->receive_state == UNLOADED)
    {
        //拷贝数据
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //接收数值状态置为已读取
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed){
    static fp32 last_bullet_speed = 0;
    //添加弹速
    if (cur_bullet_speed != 0 && cur_bullet_speed != last_bullet_speed && 
        cur_bullet_speed >= MIN_SET_BULLET_SPEED && cur_bullet_speed <= MAX_SET_BULLET_SPEED){
        bullet_speed->bullet_speed[(bullet_speed->pos++) % BULLET_SPEED_SIZE] = cur_bullet_speed;
        last_bullet_speed = cur_bullet_speed;
    }
    //判断是否已满
    if (bullet_speed->full_flag == 0 && bullet_speed->pos == BULLET_SPEED_SIZE - 1){
        bullet_speed->full_flag = 1;
    }
    //估计弹速
    fp32 sum = 0;;
    for (int i = 0; 
        (bullet_speed->full_flag == 1 && i < BULLET_SPEED_SIZE) || (bullet_speed->full_flag == 0 && i < bullet_speed->pos);
        i++){
            sum += bullet_speed->bullet_speed[i];
        }
    bullet_speed->est_bullet_speed = sum / BULLET_SPEED_SIZE;
}


static void vision_set_target_armor_color(vision_control_t* set_detect_color, robot_armor_color_e enemy_armor_color)
{
    //设置目标颜色为目标颜色
    set_detect_color->detect_armor_color = enemy_armor_color;
}



static void vision_judge_appear_target(vision_control_t* judge_appear_target)
{
    //根据接收数据判断是否为识别到目标
    if (judge_appear_target->vision_receive_point->receive_packet.x == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.y == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.z == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.yaw == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vx == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vy == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vz == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.v_yaw == 0
       )
    {
        //未识别到目标
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else
    {
        //识别到目标

        //更新当前时间
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //判断当前时间是否距离上次接收的时间过长
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME)
        {
            //判断为未识别目标
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else
        {
          // 设置为识别到目标
          judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
        }
    }
}


static void vision_data_process(vision_control_t* vision_data)
{
    //判断是否识别到目标
    if (vision_data->vision_target_appear_state == TARGET_APPEAR)
    {
		target_appear_flag = 1;
        // 识别到目标
        //  选择最优装甲板
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->target_position);
        // 计算机器人瞄准位置
        calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, vision_data->target_data.vx, vision_data->target_data.vy, vision_data->target_data.vz, vision_data->solve_trajectory.predict_time);
        // 计算机器人pitch轴与yaw轴角度
        fp32 pitch_set = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)), vision_data->robot_gimbal_aim_vector.z, vision_data->solve_trajectory.distance_static, vision_data->solve_trajectory.z_static, 1);
        fp32 yaw_set = atan2f(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);
        fp32 distance = sqrtf(pow(vision_data->robot_gimbal_aim_vector.x,2) + pow(vision_data->robot_gimbal_aim_vector.y,2));
	
        //out
        vision_data->gimbal_vision_control.gimbal_pitch = pitch_set;
        vision_data->gimbal_vision_control.gimbal_yaw = yaw_set;

        vision_shoot_judge(vision_data, vision_data->gimbal_vision_control.gimbal_yaw - vision_data->vision_angle_point->Yaw, vision_data->gimbal_vision_control.gimbal_pitch - vision_data->vision_angle_point->Pitch, sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2)));

    }
    else
    {
		target_appear_flag = 0;
        //设置停止发射
        vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

}



/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 *
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 * @param target_distance 目标距离
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance)
{
    fp32 allow_attack_error = 0 ;
    //判断目标距离
    if (target_distance <= ALLOW_ATTACK_DISTANCE)
    {
        // 判断迹是否小于允许值
       if (shoot_judge->vision_receive_point->receive_packet.p < ALLOE_ATTACK_P)
       {
			if (shoot_judge->target_data.id == ARMOR_HERO)
			{
				allow_attack_error = atan2((LARGE_ARM0R_WIDTH / 2.0f) - 0.02, target_distance);
			}
			else
			{
				allow_attack_error = atan2((SMALL_ARMOR_WIDTH / 2.0f) - 0.02, target_distance);
			}
			// 小于一角度开始击打
			if (fabs(vision_begin_add_yaw_angle) <= allow_attack_error)
			{
				shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
			}
			else
			{
				shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
			}
       }
    }
    else
    {
        //远距离不击打
        shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

}

static void set_vision_send_packet(vision_control_t* set_send_packet)
{
    set_send_packet->send_packet.header = LOWER_TO_HIGH_HEAD;
    set_send_packet->send_packet.detect_color = set_send_packet->detect_armor_color;
    set_send_packet->send_packet.roll = set_send_packet->vision_angle_point->Roll;
    set_send_packet->send_packet.pitch = set_send_packet->vision_angle_point->Pitch;
    set_send_packet->send_packet.yaw = set_send_packet->vision_angle_point->Yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
}


void send_packet(vision_control_t* send)
{
    if (send == NULL)
    {
        return;
    }
    //添加CRC16到结尾
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //发送数据
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRC校验
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // 拷贝接收到的数据到临时内存中
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            // 数据正确，将临时数据拷贝到接收数据包中
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            // 接收数据数据状态标志为未读取
            vision_receive.receive_state = UNLOADED;

            // 保存时间
            vision_receive.last_receive_time = vision_receive.current_receive_time;
            // 记录当前接收数据的时间
            vision_receive.current_receive_time = TIME_MS_TO_S(HAL_GetTick());
            //计算时间间隔
            vision_receive.interval_time = vision_receive.current_receive_time - vision_receive.last_receive_time;
        }
    }
}

/**
 * @brief 初始化弹道计算的参数
 *
 * @param solve_trajectory 弹道计算结构体
 * @param k1 弹道参数
 * @param init_flight_time 初始飞行时间估计值
 * @param time_bias 固有间隔时间
 * @param z_static yaw轴电机到枪口水平面的垂直距离
 * @param distance_static 枪口前推距离
 */
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static)\
{
    solve_trajectory->k1 = k1;
    solve_trajectory->flight_time = init_flight_time;
    solve_trajectory->time_bias = time_bias;
    solve_trajectory->z_static = z_static;
    solve_trajectory->distance_static = distance_static;
    solve_trajectory->current_bullet_speed = MIN_SET_BULLET_SPEED;
}



/**
 * @brief 选择最优击打目标
 *
 * @param solve_trajectory 弹道计算结构体
 * @param vision_data 接收视觉数据
 * @param optimal_target_position 最优目标位置
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //计算预测时间 = 上一次的子弹飞行时间 + 固有偏移时间, 时间可能不正确，但可以接受
    solve_trajectory->predict_time = solve_trajectory->flight_time + solve_trajectory->time_bias;
    //计算子弹到达目标时的yaw角度
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //赋值装甲板数量
    solve_trajectory->armor_num = vision_data->armors_num;

    //选择目标的数组编号
    uint8_t select_targrt_num = 0;

    //计算所有装甲板的位置
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //由于装甲板距离机器人中心距离不同，但是一般两两对称，所以进行计算装甲板位置时，第0 2块用当前半径，第1 3块用上一次半径
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // 选择与机器人自身yaw差值最小的目标,排序选择最小目标
    solve_trajectory->current_yaw = atan2(vision_data->y, vision_data->x); //目标中心的yaw角
    fp32 yaw_error_min = fabs(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[0].yaw);
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        fp32 yaw_error_temp = fabsf(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[i].yaw);
        if (yaw_error_temp <= yaw_error_min)
        {
            yaw_error_min = yaw_error_temp;
            select_targrt_num = i;
        }
    }
    // if (yaw_error_min > 0.3)
    // {
    //     //根据速度方向选择下一块装甲板
    //     if (vision_data->v_yaw > 0.5){
    //         select_targrt_num += solve_trajectory->armor_num - 1;
    //     }
    //     else if (vision_data->v_yaw < -0.5){
    //         select_targrt_num += 1;
    //     }

    //     if (select_targrt_num >= solve_trajectory->armor_num) {
    //         select_targrt_num -= solve_trajectory->armor_num;
    //     }
    // }
    // 将选择的装甲板数据，拷贝打最优目标中去
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));

}

/**
 * @brief 计算装甲板瞄准位置
 *
 * @param robot_gimbal_aim_vector 机器人云台瞄准向量
 * @param target_position 目标位置
 * @param vx 机器人中心速度
 * @param vy 机器人中心速度
 * @param vz 机器人中心速度
 * @param predict_time 预测时间
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    // 由于目标与观测中心处于同一系，速度相同
    robot_gimbal_aim_vector->x = target_position->x + (fabs(vx) >= 0.02 ? vx : 0) * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + (fabs(vy) >= 0.02 ? vy : 0) * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + (fabs(vz) >= 0.02 ? vz : 0) * predict_time; 
}




/**
 * @brief 计算子弹落点
 * @author yuanluochen
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param x 水平距离
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 子弹落点
 */
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));

    //计算子弹落点高度
    fp32 bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}

/**
 * @brief 计算弹道落点 -- 完全空气阻力模型 该模型适用于大仰角击打的击打
 * @author yuanluochen
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    //子弹落点高度
    fp32 bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
    // printf("飞行时间%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        fp32 k_z = solve_trajectory->k1 * (1 / sin(theta));
        // 上升段
        // 初始竖直飞行速度
        fp32 v_z_0 = bullet_speed * sin(theta);
        // 计算上升段最大飞行时间
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // 判断总飞行时间是否小于上升最大飞行时间
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // 子弹存在上升段
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // 超过最大上升飞行时间 -- 存在下降段
            // 计算最大高度
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else
    {
        bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    }


    return bullet_drop_z;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的仰角，
 * @author yuanluochen
 *
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param y 竖直距离
 * @param x_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的x轴偏移量
 * @param y_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的y轴偏移量
 * @param bullet_speed 弹速
 * @param mode 计算模式：
          置 1 完全空气阻力模型
          置 0 单方向空气阻力模型
 * @return 返回pitch轴数值
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode)
{
    int count = 0;
    // 计算落点高度
    float bullet_drop_z = 0;
    //云台瞄准向量
    float aim_z = z;

    // 二维平面的打击角
    float theta = 0;
    // 计算值与真实值之间的误差
    float calc_and_actual_error = 0;
    // 比例迭代法
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // 计算仰角
        theta = atan2(aim_z, x);
        // 坐标系变换，从机器人转轴系变为发射最大速度位置坐标系
        // 计算子弹落点高度
        if (mode == 1){
            bullet_drop_z =
              calc_bullet_drop_in_complete_air(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                  solve_trajectory->current_bullet_speed, theta) +
              (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        }
        else{
            bullet_drop_z =
              calc_bullet_drop(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                  solve_trajectory->current_bullet_speed, theta) +
              (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset);
        }
            
        // 计算误差
        calc_and_actual_error = z - bullet_drop_z;
        // 对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // printf("第%d次瞄准，发射系x:%f, z补偿%f, z发射系落点%f ,z机体系落点%f\n", count, x - (arm_cos_f32(theta) * x_offset), (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z);
        // 判断误差是否符合精度要求
        count++;
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
//    printf("x = %f, 原始pitch = %f, pitch = %f, 迭代次数 = %d\n", x, -atan2(z, x) * 180 / 3.14 , -(theta * 180 / 3.14), count);
    //由于为右手系，theta为向下为正，所以置负
    return -theta;
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

//获取当前视觉是否识别到目标
bool_t judge_vision_appear_target(void)
{
    return vision_control.vision_target_appear_state == TARGET_APPEAR;
}



// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}


// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
