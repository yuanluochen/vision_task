/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
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

// �Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
// �Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
// ����Ŀ��װ�װ���ɫ
static void vision_set_target_armor_color(vision_control_t* set_detect_color, robot_armor_color_e enemy_armor_color);
// �ж��Ƿ�ʶ��Ŀ��
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
static void vision_data_process(vision_control_t* vision_data);
// ���÷������ݰ�
static void set_vision_send_packet(vision_control_t* set_send_packet);



// ��ʼ����������Ĳ���
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static);
// ѡ�����Ż���Ŀ��
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
// ��ֵ��̨��׼λ��
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// ���㵯����� -- ������ռ�����ģ��
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// ���㵯����� -- ��ȫ��������ģ��
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta);
// ��άƽ�浯��ģ�ͣ�����pitch��ĸ߶�
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode);
//���µ��٣������Ƶ�ǰ����
static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed);

// ��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);

// �Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
// �Ӿ����սṹ��
vision_receive_t vision_receive = { 0 };

fp32 time_bias = 15.0f;

void vision_task(void const* pvParameters)
{
    // ��ʱ�ȴ����ȴ���λ���������ݳɹ�
    vTaskDelay(VISION_TASK_INIT_TIME);
    // �Ӿ������ʼ��
    vision_task_init(&vision_control);
    // �ȴ���̨�����ʼ�����

    // ϵͳ��ʱ
    vTaskDelay(VISION_CONTROL_TIME_MS);

    while(1)
    {
        // ��������
        vision_task_feedback_update(&vision_control);
        // ����Ŀ��װ�װ���ɫ
        vision_set_target_armor_color(&vision_control, BLUE);
        // �ж��Ƿ�ʶ��Ŀ��
        vision_judge_appear_target(&vision_control);
        // ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�,�����ÿ�������
        vision_data_process(&vision_control);

        // ���÷������ݰ�
        set_vision_send_packet(&vision_control);
        // �������ݰ�
        send_packet(&vision_control);

        // ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_task_init(vision_control_t* init)
{
    // ��ȡ�����Ǿ��Խ�ָ��
    init->vision_angle_point = get_INS_point();
    // ��ȡ�������ݰ�ָ��
    init->vision_receive_point = get_vision_receive_point();

    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //��ʼ��һЩ�����ĵ�������
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, TIME_MS_TO_S(time_bias), Z_STATIC, DISTANCE_STATIC);
    //��ʼ������
    init->bullet_speed.pos = -1;
    init->bullet_speed.est_bullet_speed = 0;
    init->bullet_speed.full_flag = 0;
    memset(init->bullet_speed.bullet_speed, 0, sizeof(init->bullet_speed.bullet_speed[0]) * BULLET_SPEED_SIZE);
    //��ʼ���Ӿ�Ŀ��״̬Ϊδʶ��Ŀ��
    init->vision_target_appear_state = TARGET_UNAPPEAR;
    
    //��������
    vision_task_feedback_update(init);
}
// extern shoot_date;

static void vision_task_feedback_update(vision_control_t* update)
{
    //���µ���
    update_bullet_speed(&update->bullet_speed, CUR_BULLET_SPEED);
    update->solve_trajectory.current_bullet_speed = update->bullet_speed.est_bullet_speed;
    //��ȡĿ������
    if (update->vision_receive_point->receive_state == UNLOADED)
    {
        //��������
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //������ֵ״̬��Ϊ�Ѷ�ȡ
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void update_bullet_speed(bullet_speed_t *bullet_speed, fp32 cur_bullet_speed){
    static fp32 last_bullet_speed = 0;
    //��ӵ���
    if (cur_bullet_speed != 0 && cur_bullet_speed != last_bullet_speed && 
        cur_bullet_speed >= MIN_SET_BULLET_SPEED && cur_bullet_speed <= MAX_SET_BULLET_SPEED){
        bullet_speed->bullet_speed[(++bullet_speed->pos) % BULLET_SPEED_SIZE] = cur_bullet_speed;
        last_bullet_speed = cur_bullet_speed;
    }
    //�ж��Ƿ�����
    if (bullet_speed->full_flag == 0 && bullet_speed->pos == BULLET_SPEED_SIZE - 1){
        bullet_speed->full_flag = 1;
    }
    //���Ƶ���
    fp32 sum = 0;;
    for (int i = 0; 
        (bullet_speed->full_flag == 1 && i < BULLET_SPEED_SIZE) || (bullet_speed->full_flag == 0 && i <= bullet_speed->pos);
        i++){
            sum += bullet_speed->bullet_speed[i];
        }
    if (bullet_speed->pos >= 0)
    {
        bullet_speed->est_bullet_speed = sum / (bullet_speed->full_flag == 1 ? BULLET_SPEED_SIZE : (bullet_speed->pos + 1));
    }
    if (bullet_speed->est_bullet_speed == 0)
        bullet_speed->est_bullet_speed = BEGIN_SET_BULLET_SPEED;
}


static void vision_set_target_armor_color(vision_control_t* set_detect_color, robot_armor_color_e enemy_armor_color)
{
    //����Ŀ����ɫΪĿ����ɫ
    set_detect_color->detect_armor_color = enemy_armor_color;
}



static void vision_judge_appear_target(vision_control_t* judge_appear_target)
{
    //���ݽ��������ж��Ƿ�Ϊʶ��Ŀ��
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
        //δʶ��Ŀ��
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else
    {
        //ʶ��Ŀ��

        //���µ�ǰʱ��
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //�жϵ�ǰʱ���Ƿ�����ϴν��յ�ʱ�����
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME)
        {
            //�ж�Ϊδʶ��Ŀ��
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else
        {
          // ����Ϊʶ��Ŀ��
          judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
        }
    }
}


static void vision_data_process(vision_control_t* vision_data)
{
    //�ж��Ƿ�ʶ��Ŀ��
    if (vision_data->vision_target_appear_state == TARGET_APPEAR)
    {
		target_appear_flag = 1;
        // ʶ��Ŀ��
        //  ѡ������װ�װ�
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->target_position);
        // �����������׼λ��
        calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, vision_data->target_data.vx, vision_data->target_data.vy, vision_data->target_data.vz, vision_data->solve_trajectory.predict_time);
        // ���������pitch����yaw��Ƕ�
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
        //����ֹͣ����
        vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

}



/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з��䣬�ж�yaw��pitch�ĽǶȣ������һ����Χ�ڣ������ֵ���ӣ����ӵ�һ����ֵ���жϷ��䣬���yaw��pitch��Ƕȴ��ڸ÷�Χ�����������
 *
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 * @param target_distance Ŀ�����
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance)
{
    fp32 allow_attack_error = 0 ;
    //�ж�Ŀ�����
    if (target_distance <= ALLOW_ATTACK_DISTANCE)
    {
        // �жϼ��Ƿ�С������ֵ
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
			// С��һ�Ƕȿ�ʼ����
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
        //Զ���벻����
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
    //���CRC16����β
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //��������
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRCУ��
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // �������յ������ݵ���ʱ�ڴ���
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            // ������ȷ������ʱ���ݿ������������ݰ���
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            // ������������״̬��־Ϊδ��ȡ
            vision_receive.receive_state = UNLOADED;

            // ����ʱ��
            vision_receive.last_receive_time = vision_receive.current_receive_time;
            // ��¼��ǰ�������ݵ�ʱ��
            vision_receive.current_receive_time = TIME_MS_TO_S(HAL_GetTick());
            //����ʱ����
            vision_receive.interval_time = vision_receive.current_receive_time - vision_receive.last_receive_time;
        }
    }
}

/**
 * @brief ��ʼ����������Ĳ���
 *
 * @param solve_trajectory ��������ṹ��
 * @param k1 ��������
 * @param init_flight_time ��ʼ����ʱ�����ֵ
 * @param time_bias ���м��ʱ��
 * @param z_static yaw������ǹ��ˮƽ��Ĵ�ֱ����
 * @param distance_static ǹ��ǰ�ƾ���
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
 * @brief ѡ�����Ż���Ŀ��
 *
 * @param solve_trajectory ��������ṹ��
 * @param vision_data �����Ӿ�����
 * @param optimal_target_position ����Ŀ��λ��
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //����Ԥ��ʱ�� = ��һ�ε��ӵ�����ʱ�� + ����ƫ��ʱ��, ʱ����ܲ���ȷ�������Խ���
    solve_trajectory->predict_time = solve_trajectory->flight_time + solve_trajectory->time_bias;
    //�����ӵ�����Ŀ��ʱ��yaw�Ƕ�
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //��ֵװ�װ�����
    solve_trajectory->armor_num = vision_data->armors_num;

    //ѡ��Ŀ���������
    uint8_t select_targrt_num = 0;

    //��������װ�װ��λ��
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //����װ�װ������������ľ��벻ͬ������һ�������Գƣ����Խ��м���װ�װ�λ��ʱ����0 2���õ�ǰ�뾶����1 3������һ�ΰ뾶
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // ѡ�������������yaw��ֵ��С��Ŀ��,����ѡ����СĿ��
    solve_trajectory->current_yaw = atan2(vision_data->y, vision_data->x); //Ŀ�����ĵ�yaw��
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
    //     //�����ٶȷ���ѡ����һ��װ�װ�
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
    // ��ѡ���װ�װ����ݣ�����������Ŀ����ȥ
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));

}

/**
 * @brief ����װ�װ���׼λ��
 *
 * @param robot_gimbal_aim_vector ��������̨��׼����
 * @param target_position Ŀ��λ��
 * @param vx �����������ٶ�
 * @param vy �����������ٶ�
 * @param vz �����������ٶ�
 * @param predict_time Ԥ��ʱ��
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    // ����Ŀ����۲����Ĵ���ͬһϵ���ٶ���ͬ
    robot_gimbal_aim_vector->x = target_position->x + (fabs(vx) >= 0.02 ? vx : 0) * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + (fabs(vy) >= 0.02 ? vy : 0) * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + (fabs(vz) >= 0.02 ? vz : 0) * predict_time; 
}




/**
 * @brief �����ӵ����
 * @author yuanluochen
 * 
 * @param solve_trajectory ��������ṹ��
 * @param x ˮƽ����
 * @param bullet_speed ����
 * @param theta ����
 * @return �ӵ����
 */
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));

    //�����ӵ����߶�
    fp32 bullet_drop_z = (float)(bullet_speed * sin(theta) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}

/**
 * @brief ���㵯����� -- ��ȫ��������ģ�� ��ģ�������ڴ����ǻ���Ļ���
 * @author yuanluochen
 * 
 * @param solve_trajectory ��������ṹ��
 * @param x ����
 * @param bullet_speed ����
 * @param theta ����
 * @return �������
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float theta)
{
    //�ӵ����߶�
    fp32 bullet_drop_z = 0;
    //�����ܷ���ʱ��
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(theta)));
    // printf("����ʱ��%f", solve_trajectory->flight_time);
    if (theta > 0) 
    {
        //������������ϵ�� ����ֱ����
        //���������� �ӵ��ٶȷ��������ĽǶ���������0����ֱ�������� hat(f_z) = f_z * sin(theta) ���������� ��ˮƽ�������� hat(f_x) = f_x * cos(theta) �������� f_x ������Ҫ����ֱ��������ϵ�����в���
        fp32 k_z = solve_trajectory->k1 * (1 / sin(theta));
        // ������
        // ��ʼ��ֱ�����ٶ�
        fp32 v_z_0 = bullet_speed * sin(theta);
        // ����������������ʱ��
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // �ж��ܷ���ʱ���Ƿ�С������������ʱ��
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // �ӵ�����������
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // ���������������ʱ�� -- �����½���
            // �������߶�
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // �����½�
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
 * @brief ��άƽ�浯��ģ�ͣ�����pitch������ǣ�
 * @author yuanluochen
 *
 * @param solve_tragectory ��������ṹ��
 * @param x ˮƽ����
 * @param y ��ֱ����
 * @param x_offset �Ի�����ת������ϵΪ������ϵ���Է�������ٶȵ�Ϊ������ϵ��x��ƫ����
 * @param y_offset �Ի�����ת������ϵΪ������ϵ���Է�������ٶȵ�Ϊ������ϵ��y��ƫ����
 * @param bullet_speed ����
 * @param mode ����ģʽ��
          �� 1 ��ȫ��������ģ��
          �� 0 �������������ģ��
 * @return ����pitch����ֵ
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z, fp32 x_offset, fp32 z_offset, int mode)
{
    int count = 0;
    // �������߶�
    float bullet_drop_z = 0;
    //��̨��׼����
    float aim_z = z;

    // ��άƽ��Ĵ����
    float theta = 0;
    // ����ֵ����ʵֵ֮������
    float calc_and_actual_error = 0;
    // ����������
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // ��������
        theta = atan2(aim_z, x);
        // ����ϵ�任���ӻ�����ת��ϵ��Ϊ��������ٶ�λ������ϵ
        // �����ӵ����߶�
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
            
        // �������
        calc_and_actual_error = z - bullet_drop_z;
        // ����׼�߶Ƚ��в���
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // printf("��%d����׼������ϵx:%f, z����%f, z����ϵ���%f ,z����ϵ���%f\n", count, x - (arm_cos_f32(theta) * x_offset), (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z - (arm_sin_f32(theta) * x_offset + arm_cos_f32(theta) * z_offset), bullet_drop_z);
        // �ж�����Ƿ���Ͼ���Ҫ��
        count++;
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
//    printf("x = %f, ԭʼpitch = %f, pitch = %f, �������� = %d\n", x, -atan2(z, x) * 180 / 3.14 , -(theta * 180 / 3.14), count);
    //����Ϊ����ϵ��thetaΪ����Ϊ���������ø�
    return -theta;
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

//��ȡ��ǰ�Ӿ��Ƿ�ʶ��Ŀ��
bool_t judge_vision_appear_target(void)
{
    return vision_control.vision_target_appear_state == TARGET_APPEAR;
}



// ��ȡ��λ����̨����
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}


// ��ȡ��λ����������
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
