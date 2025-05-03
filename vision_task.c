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



// 视觉任务初始化
static void vision_task_init(vision_control_t* init);
// 视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
// 设置目标装甲板颜色
static void vision_set_target_armor_color(vision_control_t* set_detect_color, uint8_t cur_robot_id);
// 判断是否识别到目标
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
// 配置发送数据包
static void set_vision_send_packet(vision_control_t* set_send_packet);



// 初始化弹道解算的参数
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static, fp32 pitch_static);
// 选择最优击打目标
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, vector_t* robot_gimbal_aim_vector, fp32 cur_yaw, fp32 *body_to_enemy_robot_yaw);
// 计算弹道落点 -- 单方向空间阻力模型
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 计算弹道落点 -- 完全空气阻力模型
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// 计算弹道落点 -- RK4
static float calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, float x, float z, float bullet_speed, float theta);
// 二维平面弹道模型，计算pitch轴的高度
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, fp32 theta_offset, int mode);
//更新弹速，并估计当前弹速
static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed);
//判断发弹
static void vision_shoot_judge(vision_control_t* shoot_judge);

// 获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);

// 视觉任务结构体
vision_control_t vision_control = { 0 };
// 视觉接收结构体
vision_receive_t vision_receive = { 0 };


void vision_task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    // 视觉任务初始化
    vision_task_init(&vision_control);

    // 系统延时
    vTaskDelay(VISION_CONTROL_TIME_MS);

    while(1){
        // 更新数据---这里面有弹速要人工加弹速变量
        vision_task_feedback_update(&vision_control);
        // 设置目标装甲板颜色--人工加自身机器人id
        vision_set_target_armor_color(&vision_control, robo_date.operater_id);
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
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, TIME_MS_TO_S(TIME_BIAS), Z_STATIC, DISTANCE_STATIC, PITCH_STATIC);
    //初始化弹速
    init->bullet_speed.est_bullet_speed = 0;
    memset(init->bullet_speed.bullet_speed, 0, sizeof(init->bullet_speed.bullet_speed[0]) * BULLET_SPEED_SIZE);
    //初始化视觉目标状态为未识别到目标
    init->vision_target_appear_state = TARGET_UNAPPEAR;
    
    //更新数据
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    //更新弹速--在这里加弹速变量
    update_bullet_speed(&update->bullet_speed, shoot_date.bullet_speed);
    update->solve_trajectory.current_bullet_speed = update->bullet_speed.est_bullet_speed;
    //获取目标数据
    if (update->vision_receive_point->receive_state == UNLOADED){
        //拷贝数据
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //接收数值状态置为已读取
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed){
    static fp32 last_bullet_speed = 0;
    static int pos = -1;
    static int full_flag = 0;
    //添加弹速
    if (cur_bullet_speed != 0 && cur_bullet_speed != last_bullet_speed && 
        cur_bullet_speed >= MIN_SET_BULLET_SPEED && cur_bullet_speed <= MAX_SET_BULLET_SPEED){
        pos = (pos + 1) % BULLET_SPEED_SIZE;
        bullet_speed->bullet_speed[pos] = cur_bullet_speed;
        last_bullet_speed = cur_bullet_speed;
    }
    //判断是否已满
    if (full_flag == 0 && pos == BULLET_SPEED_SIZE - 1){
        full_flag = 1;
    }
    //估计弹速
    fp32 sum = 0;;
    for (int i = 0; 
        (full_flag == 1 && i < BULLET_SPEED_SIZE) || (full_flag == 0 && i <= pos);
        i++){
            sum += bullet_speed->bullet_speed[i];
        }
    if (pos >= 0){
        bullet_speed->est_bullet_speed = sum / (full_flag == 1 ? BULLET_SPEED_SIZE : (pos + 1));
    }
    if (bullet_speed->est_bullet_speed == 0)
        bullet_speed->est_bullet_speed = BEGIN_SET_BULLET_SPEED;
}


static void vision_set_target_armor_color(vision_control_t* set_detect_color, uint8_t cur_robot_id)
{
    //设置目标颜色为目标颜色
    if (cur_robot_id < ROBOT_RED_AND_BLUE_DIVIDE_VALUE){
        set_detect_color->detect_armor_color = RED;
    }
    else{
        set_detect_color->detect_armor_color = BLUE;
    }
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
       ){
        //未识别到目标
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else{
        //识别到目标
        //更新当前时间
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //判断当前时间是否距离上次接收的时间过长
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME){
            //判断为未识别目标
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else{
          // 设置为识别到目标
          judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
        }
    }
}


static void vision_data_process(vision_control_t* vision_data)
{
    //判断是否识别到目标
    if (vision_data->vision_target_appear_state == TARGET_APPEAR){
        //  选择最优装甲板
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->robot_gimbal_aim_vector, vision_data->vision_angle_point->Yaw, &vision_data->body_to_enemy_robot_yaw);
        // 计算机器人pitch轴与yaw轴角度
        fp32 pitch_set = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)), vision_data->robot_gimbal_aim_vector.z, vision_data->solve_trajectory.distance_static, vision_data->solve_trajectory.z_static, vision_data->solve_trajectory.pitch_static, 1);
        fp32 yaw_set = atan2f(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);

        vision_data->gimbal_vision_control.gimbal_pitch = pitch_set;
        vision_data->gimbal_vision_control.gimbal_yaw = yaw_set;
    }
    //判断击打
    vision_shoot_judge(vision_data);
}

/**
 * @brief 根据距离 观测器是否收敛 位置是否正 瞄准位置误差判断是否击打
 *
 * @param shoot_judge 视觉结构体
 */
static void vision_shoot_judge(vision_control_t* shoot_judge)
{
    //判断是否识别
    if (shoot_judge->vision_target_appear_state == TARGET_APPEAR){
        fp32 allow_attack_error_yaw = 0;
        fp32 allow_attack_error_pitch = 0;
        fp32 target_distance = sqrt(pow(shoot_judge->robot_gimbal_aim_vector.x, 2) + pow(shoot_judge->robot_gimbal_aim_vector.y, 2));
        // 判断目标距离是否过于远
        if (target_distance <= ALLOW_ATTACK_DISTANCE){
            // 判断观测器是否收敛
            if (shoot_judge->target_data.p < ALLOE_ATTACK_P){
                // 根据敌方机器人半径和当前瞄准位置判断是否过偏
                if (fabs(shoot_judge->body_to_enemy_robot_yaw - shoot_judge->vision_angle_point->Yaw) <= fabs(atan2(shoot_judge->target_data.r1 - 0.11, target_distance))){
                    // 根据装甲板大小和距离判断允许发弹误差角
                    allow_attack_error_pitch = atan2((ARMOR_HIGH / 2.0f) - 0.03, target_distance);
                    if (shoot_judge->target_data.id == ARMOR_HERO){
                        allow_attack_error_yaw = atan2((LARGE_ARM0R_WIDTH / 2.0f) - 0.03, target_distance);
                    }
                    else{
                        allow_attack_error_yaw = atan2((SMALL_ARMOR_WIDTH / 2.0f) - 0.03, target_distance);
                    }
                    fp32 yaw_error = shoot_judge->gimbal_vision_control.gimbal_yaw - shoot_judge->vision_angle_point->Yaw;
                    fp32 pitch_error = shoot_judge->gimbal_vision_control.gimbal_pitch - shoot_judge->vision_angle_point->Pitch;
                    // 小于一角度开始击打
                    if (fabs(yaw_error) <= fabs(allow_attack_error_yaw) && fabs(pitch_error) <= fabs(allow_attack_error_pitch)){
                        shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
                    }
                    else{
                        shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
                    }
                }
                else{
                    shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
                }
            }
            else{
                shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
            }
        }
        else{
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
        }
    }
    else{
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
    if (send == NULL){
        return;
    }
    //添加CRC16到结尾
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //发送数据
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2){
        return;
    }
    //CRC校验
    if (verify_CRC16_check_sum(buf, len)){
        receive_packet_t temp_packet = {0};
        // 拷贝接收到的数据到临时内存中
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD){
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
 * @param pitch_static 枪口pitch轴偏差
 */
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, 
                                        fp32 k1, 
                                        fp32 init_flight_time, 
                                        fp32 time_bias, 
                                        fp32 z_static, 
                                        fp32 distance_static, 
                                        fp32 pitch_static
                                    )
{
    solve_trajectory->k1 = k1;
    solve_trajectory->flight_time = init_flight_time;
    solve_trajectory->time_bias = time_bias;
    solve_trajectory->z_static = z_static;
    solve_trajectory->distance_static = distance_static;
    solve_trajectory->current_bullet_speed = BEGIN_SET_BULLET_SPEED;
    solve_trajectory->pitch_static = pitch_static;
}


/**
 * @brief 选择最优击打目标
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param vision_data 视觉接收数据
 * @param robot_gimbal_aim_vector 最后云台瞄准向量 
 * @param cur_yaw 当前yaw角用于补偿
 * @param body_to_enemy_robot_yaw 以机器人自身为原点在惯性系下敌方机器人的yaw角，在这里主要是给赋值用的
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, 
                                  target_data_t* vision_data, 
                                  vector_t* robot_gimbal_aim_vector, 
                                  fp32 cur_yaw, 
                                  fp32 *body_to_enemy_robot_yaw
                                )
{   
    //计算预测时间 = 上一次的子弹飞行时间 + 固有偏移时间 
    solve_trajectory->predict_time = solve_trajectory->flight_time + solve_trajectory->time_bias;
    //中心坐标位置预测
    vector_t robot_center = {
        .x = vision_data->x + solve_trajectory->predict_time * vision_data->vx,
        .y = vision_data->y + solve_trajectory->predict_time * vision_data->vy,
        .z = vision_data->z + solve_trajectory->predict_time * vision_data->vz,
    };
    // 云台运动时间近似补偿->主要补偿平移
    fp32 predict_time_offset = fabs(atan2f(robot_center.y, robot_center.x)- cur_yaw) * TRACK_GIMBAL_COTROL_OFFSET_K;
    solve_trajectory->predict_time += predict_time_offset;

    robot_center.x += vision_data->vx * predict_time_offset;
    robot_center.y += vision_data->vy * predict_time_offset;

    //计算子弹到达目标时的yaw角度
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //赋值装甲板数量
    solve_trajectory->armor_num = vision_data->armors_num;

    //选择目标的数组编号
    uint8_t select_targrt_num = 0;
    fp32 r = 0;
    //计算所有装甲板的位置
    for (int i = 0; i < solve_trajectory->armor_num; i++){
        // 由于装甲板距离机器人中心距离不同，但是一般两两对称，所以进行计算装甲板位置时，第0 2块用当前半径，第1 3块用上一次半径
        r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = robot_center.x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = robot_center.y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? robot_center.z : robot_center.z + vision_data->dz;
    }

    // 选择与敌方机器人中心差值最小的目标,排序选择最小目标
    (*body_to_enemy_robot_yaw) = atan2(robot_center.y, robot_center.x); //目标中心的yaw角
    fp32 yaw_error_min = fabs((*body_to_enemy_robot_yaw) - solve_trajectory->all_target_position_point[0].yaw);
    for (int i = 0; i < solve_trajectory->armor_num; i++){
        fp32 yaw_error_temp = fabsf((*body_to_enemy_robot_yaw) - solve_trajectory->all_target_position_point[i].yaw);
        if (yaw_error_temp <= yaw_error_min){
            yaw_error_min = yaw_error_temp;
            select_targrt_num = i;
        }
    }

    // 根据速度方向选择下一块装甲板
    if (SELECT_ARMOR_DIR == 1 && vision_data->p < EKF_CONVERGENCE_P && fabs(vision_data->v_yaw) > SELECT_ARMOR_V_YAW_THRES){
        fp32 distance = sqrt(pow(robot_center.x, 2) + pow(robot_center.y, 2));
        if (yaw_error_min > fabs(atan2(vision_data->r1 - 0.07f, distance))){
            if (vision_data->v_yaw > SELECT_ARMOR_V_YAW_THRES){
                select_targrt_num += solve_trajectory->armor_num - 1;
            }
            else if (vision_data->v_yaw < -SELECT_ARMOR_V_YAW_THRES){
                select_targrt_num += 1;
            }
            if (select_targrt_num >= solve_trajectory->armor_num){
                select_targrt_num -= solve_trajectory->armor_num;
            }
        }
    }
    
    // 将选择的装甲板数据，拷贝到云台瞄准向量
    robot_gimbal_aim_vector->x = solve_trajectory->all_target_position_point[select_targrt_num].x;
    robot_gimbal_aim_vector->y = solve_trajectory->all_target_position_point[select_targrt_num].y;
    robot_gimbal_aim_vector->z = solve_trajectory->all_target_position_point[select_targrt_num].z;
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
    if (theta > 0) {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(theta) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(theta) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        fp32 k_z = solve_trajectory->k1 * (1 / sin(theta));
        // 上升段
        // 初始竖直飞行速度
        fp32 v_z_0 = bullet_speed * sin(theta);
        // 计算上升段最大飞行时间
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // 判断总飞行时间是否小于上升最大飞行时间
        if (solve_trajectory->flight_time <= max_flight_up_time){
            // 子弹存在上升段
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else{
            // 超过最大上升飞行时间 -- 存在下降段
            // 计算最大高度
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else{
        bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    }
    return bullet_drop_z;
}

/**
 * @brief 四阶龙格库塔法拟合弹道 -- 暂时不能用（没有飞行时间），可以用来做对比用
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param z 高度
 * @param bullet_speed 弹速
 * @param theta 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_RK4(solve_trajectory_t* solve_trajectory, float x, float z, float bullet_speed, float theta){
    //算一个飞行时间，用完全空气阻力模型拟的，给自瞄预测用的
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
    // 初始化
    fp32 cur_x = x;
    fp32 cur_z = z;
    fp32 p = tan(theta / 180 * PI);
    fp32 v = bullet_speed;
    fp32 u = v / sqrt(1 + pow(p, 2));
    fp32 delta_x = x / RK_ITER;
    for (int j = 0; j < RK_ITER; j++){
        fp32 k1_u = -solve_trajectory->k1 * u * sqrt(1 + pow(p, 2));
        fp32 k1_p = -GRAVITY / pow(u, 2);
        fp32 k1_u_sum = u + k1_u * (delta_x / 2);
        fp32 k1_p_sum = p + k1_p * (delta_x / 2);

        fp32 k2_u = -solve_trajectory->k1 * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
        fp32 k2_p = -GRAVITY / pow(k1_u_sum, 2);
        fp32 k2_u_sum = u + k2_u * (delta_x / 2);
        fp32 k2_p_sum = p + k2_p * (delta_x / 2);

        fp32 k3_u = -solve_trajectory->k1 * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
        fp32 k3_p = -GRAVITY / pow(k2_u_sum, 2);
        fp32 k3_u_sum = u + k3_u * (delta_x / 2);
        fp32 k3_p_sum = p + k3_p * (delta_x / 2);

        fp32 k4_u = -solve_trajectory->k1 * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
        fp32 k4_p = -GRAVITY / pow(k3_u_sum, 2);

        u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
        p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

        cur_x += delta_x;
        cur_z += p * delta_x;
    }
    return cur_z;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的仰角，
 * @author yuanluochen
 *
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param z 竖直距离
 * @param x_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的x轴偏移量
 * @param z_offset 以机器人转轴坐标系为父坐标系，以发射最大速度点为子坐标系的y轴偏移量
 * @param theta_offset 枪管装歪了，装配误差补偿
 * @param bullet_speed 弹速
 * @param mode 计算模式：
          置 0 单方向空气阻力模型
          置 1 完全空气阻力模型
          置 2 RK4
 * @return 返回pitch轴数值
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, fp32 theta_offset, int mode)
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
    for (int i = 0; i < MAX_ITERATE_COUNT; i++){
        // 计算仰角
        theta = atan2(aim_z, x) + theta_offset;
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
        else if (mode == 2){
            bullet_drop_z =
              calc_bullet_drop_in_RK4(
                  solve_trajectory,
                  x - (arm_cos_f32(theta) * x_offset -
                       arm_sin_f32(theta) * z_offset),
                       aim_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset),
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
        if (fabs(calc_and_actual_error) < PRECISION){
            break;
        }
    }
    // printf("x = %f, 原始pitch = %f, pitch = %f, 迭代次数 = %d\n", x, -atan2(z, x) * 180 / 3.14 , -(theta * 180 / 3.14), count);
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
