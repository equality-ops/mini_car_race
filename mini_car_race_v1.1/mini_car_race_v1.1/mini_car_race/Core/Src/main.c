/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * 本文件内含BMI270�????螺仪的使用示�????
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct PIDcontrol
{
  int16_t target;
  int16_t actual;
  int16_t nowError;
  int16_t preError;
  int16_t prepreError;
  int32_t integral;
  float derivative;
  float kp, ki, kd;
  float kp2;  // 转向环二次比例系数
  float GKD;  // 转向环陀螺仪反馈项的增益系数
  float A, B; // 变速积分阈值
  float output;
}PID;

typedef struct DisperseControl
{
  int8_t disperse_count;      // 光电管返回值离散区域数量
  int8_t flags;               // 检测标志
}IF_DISPERSE;

typedef struct {
  float wheel_diameter; // 轮子直径
  float wheel_base;     // 轮子间距
  int16_t encoder_ppr;  // 编码器每转脉冲数
  float gyro_scale;     // 陀螺仪比例系数 （度每秒 -> 弧度每秒） 
  float motor_reducation_ratio;  //  电机减速比
  int8_t sampling_period;  // 采样周期(ms)
}ROBOT_CONFIG;

typedef struct {
  int32_t left_encoder_count;  // 左编码器计数
  int32_t right_encoder_count; // 右编码器计数
  float gyro_z_rate;      // 陀螺仪z轴角速度
}SENSOR_DATA;

typedef struct {
  float current_X;
  float current_Y;
  float prev_X;
  float prev_Y;
  float current_theta;
  float total_distance;
}POSE;

typedef struct {
  float Ready_angle_distance_Continuous_angle;
  float Finish_angle_distance_Continuous_angle;
  int8_t Pass_cross_line_times;
  int8_t Cross_line_detected_times;
}PATH;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PHOTO_NUM 12              // 光电管数量
#define integralLimit 20000       // 积分最大值
#define FILTER_SIZE 5             // 微分滤波窗口数量
#define FILTER_SIZE_ERROR 30     // 光电管误差滤波窗口数量
#define HIGH_BASE_SPEED 90       // 高速基准速度
#define READY_TURN_BASE_SPEED 60  // 准备直角转弯基准速度
#define TURN_BASE_SPEED 55        // 直角转弯基准速度     

#define LEFT_OUTPUTMAX 3600      // 左电机速度环输出最大值
#define LEFT_OUTPUTMIN -3600     // 左电机速度环输出最小值
#define RIGHT_OUTPUTMAX 3600     // 右电机速度环输出最大值
#define RIGHT_OUTPUTMIN -3600    // 右电机速度环输出最小值
#define TURN_OUTPUTMAX 3000      // 转向环输出最大值
#define TURN_OUTPUTMIN -3000     // 转向环输出最小值
#define FINAL_OUTPUTMAX 5400     // 最终输出最大值
#define FINAL_OUTPUTMIN -5400    // 最终输出最小值
#define DOTTED_LINE_PHOTO_ERROR_LIMIt 441.0f  // 判断虚线的光电管误差阈值
#define RIGHT_ANGLE_PHOTO_ERROR_LIMIT 1479.0f // 判断直角弯的光电管误差阈值
#define PHOTO_ERROR_MAX 500.0f   // 光电管误差能达到的最大值
#define PHOTO_ERROR_MIN -500.0f  // 光电管误差能达到的最小值

#define RIGHT_ANGLE_TURN_KP 0.25f   // 直角转弯时的kp值
#define RIGHT_ANGLE_TURN_KD 0.02f   // 直角转弯时的kd值
#define RIGHT_ANGLE_TURN_GKD -0.7f  // 直角转弯时的GKD值
#define LOSE_lINE_KP 0.5f          // 丢线时的kp值
#define LOSE_lINE_KD 0.0f          // 丢线时的kd值
#define LOSE_LINE_GKD -0.5f         // 丢线时的gkd值
#define RESTORE_KP 0.15f             // 恢复模式的kp值
#define RESTORE_KD 0.0f            // 恢复模式的kd值

#define RIGHT_ANGLE_DETECT_TIMES 6       // 直角转弯的检测次数
#define ROUNDABOUT_DETECT_TIMES 10         // 环岛的检测次数
#define CROSS_LINE_DETECT_TIMES 8        // 十字路口的检测次数

#define RIGHT_ANGLE_TURN_COUNT 100    // 直角转弯模式计数器阈值
#define RESTORE_NORMAL_COUNT 300     // 恢复模式计数器阈值
#define ROUNDABOUT_COUNT 135        // 环岛模式计数器阈值

#define First_distance 210.0f
#define Second_distance 660.0f

#define LEFT_MOTOR -1              // 左电机标志
#define RIGHT_MOTOR 1              // 右电机标志
#define TURN 0                     // 转向环标志

#define START_RIGHT_ANGLE_MODE 1      // 进入直角转弯模式标志
#define EXIT_RIGHT_ANGLE_MODE 0       // 退出直角转弯模式标志
#define READY_RIGHT_ANGLE_MODE 2      // 准备进入直角转弯模式标志
#define RESTORE_NORMAL_MODE 3         // 进入恢复模式标志
#define ROUNDABOUT_MODE 4             // 环岛模式标志
#define READY_DOTTED_LINE_MODE 5      // 准备通过虚线模式标志

#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 缓冲区数组定义,position为位置式PID，incremental为增量式PID,L为左电机，R为右电机`
// volatile是为了防止优化掉这些变量，static是为了限制变量作用域在本文件内
volatile static float diff_buffer_position_turn[FILTER_SIZE] = {0};
volatile static float diff_buffer_position_L[FILTER_SIZE] = {0};
volatile static float diff_buffer_position_R[FILTER_SIZE] = {0};
volatile static float diff_buffer_photo_error[FILTER_SIZE_ERROR] = {0};
volatile static float diff_buffer_gyro_z[FILTER_SIZE] = {0};

volatile static int16_t buf_index_speed_L = 0;   // 速度环左电机微分滤波索引
volatile static int16_t buf_index_speed_R = 0;   // 速度环右电机微分滤波索引
volatile static int16_t buf_index_turn = 0;    // 转向环微分滤波索引
volatile static int16_t buf_index_error = 0;   // 光电管误差索引
volatile static int16_t buf_index_gyro_z = 0;  // 陀螺仪数据索引

volatile static float Error_MAX = 0.0f; // 光电管误差最大值

volatile static float record_error = 0.0f;      // 直角转弯时光电管误差记录
volatile static float record_Error_MAX = 0.0f;  // 光电管最大误差计算函数

volatile static int8_t right_angle_detect_flags = 0;  // 直角弯检测次数
volatile static int8_t roundabout_detect_flags = 0;   // 环岛检测次数
volatile static int8_t record_path_flag = 0;          // 路径规划标志
volatile static int8_t right_angle_turn_record_times = 0; // 直角转弯模式记录次数

volatile uint8_t valid_count = 0;                     // 光电管亮起数量

volatile int16_t Left_actual = 0, Right_actual = 0, Direction_actual = 0; // 左右电机实际速度,和转向环实际位置
volatile int16_t Left_pwm = 0, Right_pwm = 0;                             // 左右电机输出的pwm

volatile static uint32_t count = 0;             // 多路复用时间计数器
volatile static uint32_t right_angle_turn_count = 0;      // 直角转弯模式时间计数器
volatile static uint32_t restore_count = 0;     // 恢复模式时间计数器 
volatile static uint32_t roundabout_count = 0;  // 环岛模式时间计数器
volatile static float weighted_sum_record = 0;  // 上一次光电管误差记录

volatile static float record_kp = 0.0f;                  // 用于记录转向环kp值
volatile static float record_kd = 0.0f;                  // 用于记录转向环kd值
volatile static float record_gkd = 0.0f;                 // 用于记录转向环gkd值

volatile static int8_t current_mode = EXIT_RIGHT_ANGLE_MODE;   // 是否处于直角转弯模式标志

volatile static int16_t photo_error_weight[12] = {-440,-360,-280,-200,-120,-120,120,120,200,280,360,440}; // 光电管加权值数组

PID speed_pid_left, speed_pid_right;                     // 速度环PID定义
PID direction_pid;                                       // 转向环PID定义
IF_DISPERSE if_disperse = {0,0};
ROBOT_CONFIG robot_config = {3.0f, 15.0f, 4096, 0.017, 1.0, 1}; // 智能车硬件参数初始化
SENSOR_DATA sensor_data = {0, 0, 0.0f}; // 传感器数据初始化
POSE pose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -5.0f}; // 位置数据初始化
PATH path_config = {-1.0f, -1.0f, 0, 0}; // 路径规划初始化

float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; // 陀螺仪数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 参数说明：raw_gyro_z 原始陀螺仪z轴数据
float filtered_gyro_z(float raw_gyro_z)
{ // 陀螺仪数据滤波函数
  // 更新滑动窗口
  diff_buffer_gyro_z[buf_index_gyro_z] = raw_gyro_z;
  buf_index_gyro_z = (buf_index_gyro_z + 1) % FILTER_SIZE;
  // 计算平均值
  int16_t i = 0;
  float sum = 0.0f;
  for (i = 0; i < FILTER_SIZE; i++)
  {
    sum += diff_buffer_gyro_z[i];
  }
  return sum / FILTER_SIZE;
}

// 功能：获取当前编码器和陀螺仪的数值
void Read_sensors(void)
{
  // 获取当前编码器值
  sensor_data.left_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  sensor_data.right_encoder_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  // 重置计数器
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  // 获取滤波后的陀螺仪数据
  sensor_data.gyro_z_rate = filtered_gyro_z(gyro_z); 
}

float Compute_dist(float enconder_count)
{
  float wheel_circumference = PI * robot_config.wheel_diameter;
  return ((float)enconder_count / robot_config.encoder_ppr) * robot_config.motor_reducation_ratio * wheel_circumference;
}


void Update_odometry(void)
{
  if(fabs(pose.total_distance + 5.0) <= 1e-7) return; // 如果总距离计数器关闭则不更新里程计

  // 计算左右轮的运动距离
  float dist_left = Compute_dist(sensor_data.left_encoder_count);
  float dist_right = Compute_dist(sensor_data.right_encoder_count);
  // 计算车辆整体运动
  float linear_dist = (dist_left + dist_right) / 2;
  // 更新总距离计数器
  pose.total_distance += linear_dist; 
}

int8_t Path_choose(void)
{
  if(pose.total_distance > path_config.Ready_angle_distance_Continuous_angle && path_config.Ready_angle_distance_Continuous_angle > 0.0f)
  {
    path_config.Ready_angle_distance_Continuous_angle = -1.0f; // 标记为已进入连续转弯
    path_config.Finish_angle_distance_Continuous_angle = Second_distance; // 此为后续需要完成连续转弯的距离
    pose.total_distance = 0.0f; // 重置总距离计数器
    record_path_flag = 1; // 准备进入连续转弯
  }
  else if(pose.total_distance > path_config.Finish_angle_distance_Continuous_angle && path_config.Finish_angle_distance_Continuous_angle > 0.0f)
  {
    path_config.Finish_angle_distance_Continuous_angle = -1.0f; // 标记为已完成连续转弯
    pose.total_distance = -5.0f; // 关闭总距离计数器
    record_path_flag = 0; // 完成连续转弯
  }

  return record_path_flag;
}

// 参数说明：weighted_value 光电管误差，diff_buffer_photo_error 光电管误差缓冲区
// 功能：更新缓冲区,寻找光电管误差最大值并返回
float FindMax_WeightedValue(float weighted_value, volatile uint8_t valid_count, volatile dierroff_buffer_photo_error[])
{ // 光电管误差寻最大值函数
  // 更新滑动窗口
  if(valid_count != 0)
  {
    dierroff_buffer_photo_error[buf_index_error] = weighted_value * valid_count;
    buf_index_error = (buf_index_error + 1) % FILTER_SIZE_ERROR;
  }
  // 寻找最大值并返回
  float MAX = dierroff_buffer_photo_error[0];
  for (int8_t i = 0; i < FILTER_SIZE_ERROR; i++)
  {
    if (fabs(dierroff_buffer_photo_error[i]) > fabs(MAX))
    {
      MAX = dierroff_buffer_photo_error[i];
    }
  }
  return MAX;
}

int8_t If_disperse(uint8_t photo_value, struct DisperseControl* if_disperse) // 判断光电管读数是否离散函数
{
  if(photo_value)
  {
    if_disperse->disperse_count = 0;
    switch (if_disperse->flags)
    {
      case 0:
      if_disperse->flags = 1;
      if_disperse->disperse_count = 1;
      break;
      case 1:
      break;
    }
  }
  else
  {
    if_disperse->flags = 0;
  }

  return if_disperse->disperse_count;
}

float Calculate_Photo_Error(void)
{ // 光电管误差计算函数
  valid_count = 0;
  int16_t weighted_sum = 0;
  uint16_t photo_value = 0;
  uint8_t disperse_sum = 0;

  MUX_get_value(&photo_value);// 获取每个通道的返回值

  for (int i = 0; i < PHOTO_NUM; i++)
  {
    uint8_t photo_value_record = (photo_value >> (PHOTO_NUM - i - 1)) & 1;
    disperse_sum += If_disperse(photo_value_record, &if_disperse);
    if (photo_value_record)
    {                                                
      weighted_sum += photo_error_weight[i]; // 计算加权和
      valid_count++;
    }
  }
  
  if(disperse_sum == 2)
  {
    roundabout_detect_flags++;
  }
  else
  {
    roundabout_detect_flags = 0;
  }

  if (valid_count == 0)
  {              // 特殊值表示丢线
    return 9999; // 丢线直接退出，防止weighted_sum_record被更新为9999
  }
  weighted_sum_record = (float)weighted_sum / valid_count; // 记录光电管误差并对其进行滤波处理,除以亮起数量取平均，是为了防止亮起数量变化导致误差突变
  return weighted_sum_record;                              // 返回光电管误差
}

// 参数说明：nowError 当前误差，preError 上次误差，kd 微分系数，diff_buffer 滤波缓冲区
float filtered_derivative(int32_t nowError, int32_t preError, float kd,volatile float diff_buffer[], int8_t mode)
{ // 微分缓冲滤波函数
  float raw_diff = 0;
  // 更新目前的微分项
  raw_diff = kd * (nowError - preError);
  // 更新滑动窗口
  if(mode == TURN)
  {
    diff_buffer[buf_index_turn] = raw_diff;
    buf_index_turn = (buf_index_turn + 1) % FILTER_SIZE;
  }
  else if(mode == LEFT_MOTOR)
  {
    diff_buffer[buf_index_speed_L] = raw_diff;
    buf_index_speed_L = (buf_index_speed_L + 1) % FILTER_SIZE;
  }
  else if(mode == RIGHT_MOTOR)
  {
    diff_buffer[buf_index_speed_R] = raw_diff;
    buf_index_speed_R = (buf_index_speed_R + 1) % FILTER_SIZE;
  }
  // 计算平均值
  int16_t i = 0;
  float sum = 0.0f;
  for (i = 0; i < FILTER_SIZE; i++)
  {
    sum += diff_buffer[i];
  }
  return sum / FILTER_SIZE;
}

// 参数解释：pid PID结构体指针，actual 实际位置，judge 判断操作对象
void ComeputePID_Position(PID *pid, int16_t actual, int8_t judge)
{
  pid->actual = actual;
  pid->preError = pid->nowError;
  pid->nowError = pid->target - pid->actual;
  pid->integral += pid->nowError;
  // 调用微分滤波函数
  if (judge == TURN)
  {
    pid->derivative = filtered_derivative(pid->nowError, pid->preError, pid->kd, diff_buffer_position_turn, TURN);
  }
  else
  {
    if (judge == LEFT_MOTOR)
    {
      pid->derivative = filtered_derivative(pid->nowError, pid->preError, pid->kd, diff_buffer_position_L, LEFT_MOTOR);
    }
    else if (judge == RIGHT_MOTOR)
    {
      pid->derivative = filtered_derivative(pid->nowError, pid->preError, pid->kd, diff_buffer_position_R, RIGHT_MOTOR);
    }
  }

  // 变速积分
  // 变速积分（处理正负积分）
  {
    float abs_integral = fabsf((float)pid->integral);
    float coefficient = 1.0f;
    if (pid->A == pid->B)
    {
      coefficient = (abs_integral > pid->A) ? 0.0f : 1.0f; // 避免除以0
    }
    else
    {
      if (abs_integral > pid->A)
      {
        coefficient = 0.0f; // 如果绝对值超过A，积分清零
      }
      else if (abs_integral > pid->B)
      {
        coefficient = (pid->A - abs_integral) / (pid->A - pid->B); // 如果绝对值在B到A之间，线性衰减
      }
    }
    pid->integral = (int32_t)((float)pid->integral * coefficient); // 应用系数（保留符号）
  }

  // 计算output
  if(judge == TURN)
  {
    pid->output = pid->kp * pid->nowError + abs(pid->nowError) * pid->nowError * pid->kp2 + pid->ki * pid->integral + pid->derivative + pid->GKD * sensor_data.gyro_z_rate;
  }
  else
  {
    pid->output = pid->kp * pid->nowError + pid->ki * pid->integral + pid->derivative;
  }
  

  // 输出限幅
  if (judge == LEFT_MOTOR)
  {
    if (pid->output > LEFT_OUTPUTMAX)
    {
      pid->output = LEFT_OUTPUTMAX;
    }
    else if (pid->output < LEFT_OUTPUTMIN)
    {
      pid->output = LEFT_OUTPUTMIN;
    }
  }
  else if (judge == RIGHT_MOTOR)
  {
    if (pid->output > RIGHT_OUTPUTMAX)
    {
      pid->output = RIGHT_OUTPUTMAX;
    }
    else if (pid->output < RIGHT_OUTPUTMIN)
    {
      pid->output = RIGHT_OUTPUTMIN;
    }
  }
  else
  {
    if (pid->output > TURN_OUTPUTMAX)
    {
      pid->output = TURN_OUTPUTMAX;
    }
    else if (pid->output < TURN_OUTPUTMIN)
    {
      pid->output = TURN_OUTPUTMIN;
    }
  }
}

// 参数说明：motor 电机选择，LEFT_MOTOR 左电机，RIGHT_MOTOR 右电机
void Compute_target(int8_t motor)
{ // 计算电机的目标速度
  if (motor == LEFT_MOTOR)
  {
    if(current_mode == START_RIGHT_ANGLE_MODE) // 直角转弯模式下基准速度线性降为LOW_BASE_SPEED
    {
      speed_pid_left.target = TURN_BASE_SPEED - direction_pid.output;
    }
    else if((current_mode == READY_RIGHT_ANGLE_MODE && right_angle_detect_flags >= 1) || current_mode == READY_DOTTED_LINE_MODE)
    {
      speed_pid_left.target = READY_TURN_BASE_SPEED - direction_pid.output;
    }
    else if(current_mode == RESTORE_NORMAL_MODE)
    {
      speed_pid_left.target = TURN_BASE_SPEED + (READY_TURN_BASE_SPEED - TURN_BASE_SPEED) * ((float)restore_count / RESTORE_NORMAL_COUNT) - direction_pid.output;
    }
    else // 一般情况
    {
      if(Path_choose())
      {
        speed_pid_left.target = READY_TURN_BASE_SPEED - direction_pid.output;
      }
      else
      {
        speed_pid_left.target = HIGH_BASE_SPEED - direction_pid.output;
      }
    }
  }
  else if (motor == RIGHT_MOTOR)
  {
    if(current_mode == START_RIGHT_ANGLE_MODE) // 直角转弯模式下基准速度线性降为LOW_BASE_SPEED
    {
      speed_pid_right.target = TURN_BASE_SPEED + direction_pid.output;
    }
    else if((current_mode == READY_RIGHT_ANGLE_MODE && right_angle_detect_flags >= 1) || current_mode == READY_DOTTED_LINE_MODE)
    {
      speed_pid_right.target = READY_TURN_BASE_SPEED + direction_pid.output;
    }
    else if(current_mode == RESTORE_NORMAL_MODE)
    {
      speed_pid_right.target = TURN_BASE_SPEED + (READY_TURN_BASE_SPEED - TURN_BASE_SPEED) * ((float)restore_count / RESTORE_NORMAL_COUNT) + direction_pid.output;
    }
    else // 一般情况
    {
      if(Path_choose())
      {
        speed_pid_right.target = READY_TURN_BASE_SPEED + direction_pid.output;
      }
      else
      {
        speed_pid_right.target = HIGH_BASE_SPEED + direction_pid.output;
      }
    }
  }
}

void PID_Init(void)
{ // 初始化PID参数
  direction_pid.kp = 0.41f;
  direction_pid.kp2 = 0.0003f;
  direction_pid.ki = 0.0f;
  direction_pid.kd = 0.0f;
  direction_pid.GKD = -0.28f;
  direction_pid.A = 800.0f;
  direction_pid.B = 200.0f;
  direction_pid.target = 0;

  speed_pid_left.kp = 20.0f;
  speed_pid_left.ki = 2.1f;
  speed_pid_left.kd = 0.0f;
  speed_pid_left.A = 1200.0f;
  speed_pid_left.B = 600.0f;
  speed_pid_left.target = HIGH_BASE_SPEED;

  speed_pid_right.kp = 20.0f;
  speed_pid_right.ki = 2.3f;
  speed_pid_right.kd = 0.0f;
  speed_pid_right.A = 1200.0f;
  speed_pid_right.B = 600.0f;
  speed_pid_right.target = HIGH_BASE_SPEED;

  record_kp = direction_pid.kp; // 记录最初的转向环kp值
  record_kd = direction_pid.kd; // 记录最初的转向环kd值
  record_gkd = direction_pid.GKD; // 记录最初的转向环gkd值
}

float Right_angle_mode(void) // 直角转弯模式函数
{
  // PID参数整定
  direction_pid.kp = RIGHT_ANGLE_TURN_KP;
  direction_pid.kd = RIGHT_ANGLE_TURN_KD;
  direction_pid.GKD = RIGHT_ANGLE_TURN_GKD;

  right_angle_turn_count++;

  return record_error; // 返回已经记录的误差
}

float Ready_right_angle_mode(float photo_error) // 准备进行直角转弯模式函数
{
  if(right_angle_detect_flags >= RIGHT_ANGLE_DETECT_TIMES - 1) // 判断是否连续多次满足进入直角转弯模式条件
  {
    current_mode = START_RIGHT_ANGLE_MODE;
    direction_pid.kp = RIGHT_ANGLE_TURN_KP; // 切换为直角转弯时的kp和kd值
    direction_pid.kd = RIGHT_ANGLE_TURN_KD; 
    direction_pid.GKD = RIGHT_ANGLE_TURN_GKD;
    return record_error;
  }
  else if(right_angle_detect_flags == 0)
  {
    if(photo_error > 0)
    {
      record_error = PHOTO_ERROR_MAX;
    }
    else if(photo_error < 0)
    {
      record_error = PHOTO_ERROR_MIN;
    }
    else
    {
      record_error = 0.0f; // 可能进入到十字路口，光电管误差应为0.0f
    }
  } 

  right_angle_detect_flags++; // 完成一次对直角弯标志的判断

  return photo_error;
}

float Restore_mode(float photo_error) // 恢复模式函数
{
  direction_pid.kp = RESTORE_KP;  
  direction_pid.kd = RESTORE_KD;
  if(photo_error == 9999)
  {
    photo_error = record_error;
  }

  restore_count++;
  return photo_error;
}

float Loseline_mode(void) // 丢线模式函数
{
  // 模式及检测次数重置
  right_angle_detect_flags = 0; // 直角弯检测次数重置
  current_mode = EXIT_RIGHT_ANGLE_MODE; // 退出直角转弯模式
  path_config.Cross_line_detected_times = 0; // 十字路口检测次数重置
  
  if(fabs(Error_MAX) > RIGHT_ANGLE_PHOTO_ERROR_LIMIT) // 判断是否达到直角转弯条件
  {
    direction_pid.kp = LOSE_lINE_KP;
    direction_pid.kd = LOSE_lINE_KD;
    direction_pid.GKD = LOSE_LINE_GKD;
    if(Error_MAX > 0.0f)
    {
      record_Error_MAX = PHOTO_ERROR_MAX;
    }
    else if(Error_MAX < 0.0f)
    {
      record_Error_MAX = PHOTO_ERROR_MIN;
    }
    return record_Error_MAX;
  }
  else if(fabs(Error_MAX) < DOTTED_LINE_PHOTO_ERROR_LIMIt)
  {
    direction_pid.kp = record_kp;
    direction_pid.kd = record_kd;
    direction_pid.GKD = record_gkd;
    return 0.0f;
  }
  else
  {
    direction_pid.kp = LOSE_lINE_KP;
    direction_pid.kd = LOSE_lINE_KD;
    direction_pid.GKD = LOSE_LINE_GKD;
    return Error_MAX;
  }
}



int8_t If_ready_right_angle_turn(float photo_error) // 判断是否准备进入直角转弯模式函数
{
  if((fabs(photo_error) < RIGHT_ANGLE_PHOTO_ERROR_LIMIT) && (valid_count == 3 || valid_count == 4 || valid_count == 5 || valid_count == 6))
  {
    return 1; // 准备进入直角转弯模式
  }
  else
  {
    return 0; // 不准备进入直角转弯模式
  }
}

int8_t If_on_roundabout(void) // 判断是否处于环岛模式函数
{
  if(roundabout_detect_flags >= ROUNDABOUT_DETECT_TIMES || current_mode == ROUNDABOUT_MODE)
  {
    return 1; // 处于环岛模式
  }
  else
  {
    return 0; // 不处于环岛模式
  }
}

int8_t If_on_right_angle_turn(float photo_error) // 判断是否处于直角转弯模式函数
{
  if((valid_count >= 7 && valid_count <= 9 && (fabs(photo_error) * (valid_count) >= RIGHT_ANGLE_PHOTO_ERROR_LIMIT)) || current_mode == START_RIGHT_ANGLE_MODE)
  {
    return 1; // 处于直角转弯模式
  }
  else
  {
    return 0; // 不处于直角转弯模式
  }
}

int8_t If_on_cross_line(float photo_error) // 判断是否处于十字路口模式函数
{
  if(valid_count == 12 && (fabs(photo_error) - 0.0f < 1e-7)) // 光电管误差为0且全部亮起
  {
    return 1; // 处于十字路口模式
  }
  else
  {
    return 0; // 不处于十字路口模式
  }
}

float Normal_mode(float photo_error) // 一般模式函数
{
  // 模式及检测次数重置
  right_angle_detect_flags = 0;  // 直角弯检测次数重置，防止上次使用时未置0的检测次数影响到下一次直角弯的连续帧判断
  path_config.Cross_line_detected_times = 0; // 十字路口检测次数重置，防止上次使用时未置0的检测次数影响到下一次十字路口的连续帧判断
  
  // PID参数整定
  direction_pid.kp = record_kp;  
  direction_pid.kd = record_kd;

  if(If_ready_right_angle_turn(photo_error)) // 准备进入直角转弯模式
  {
    current_mode = READY_RIGHT_ANGLE_MODE; 
  }
  else // 退出准备进入直角转弯模式
  {
    current_mode = EXIT_RIGHT_ANGLE_MODE;
  }
  
  return photo_error;
}

float Roundabout_mode(void) // 环岛模式函数
{
  // 模式及检测次数重置
  path_config.Cross_line_detected_times = 0; // 十字路口检测次数重置
  right_angle_detect_flags = 0; // 重置直角转弯检测次数
  if(current_mode != ROUNDABOUT_MODE)
  {
    current_mode = ROUNDABOUT_MODE;
  }

  // PID参数整定
  direction_pid.kp = record_kp;
  direction_pid.kd = record_kd;
  direction_pid.GKD = record_gkd;

  roundabout_count++; // 完成一次对环岛标志的判断
  
  if(roundabout_count <= 75)
  {
    return 0.0f;
  }
  else
  {
    return -500.0f;
  }
}

void Cross_line_mode(void) // 十字路口模式函数
{
  // 模式及检测次数重置
  right_angle_detect_flags = 0; // 重置直角转弯检测次数
  current_mode = EXIT_RIGHT_ANGLE_MODE;

  // PID参数整定
  direction_pid.kp = record_kp;
  direction_pid.kd = record_kd;
  direction_pid.GKD = record_gkd;
  
  if(path_config.Cross_line_detected_times != -1)
  {
    path_config.Cross_line_detected_times++;
  }

  if(path_config.Cross_line_detected_times >= CROSS_LINE_DETECT_TIMES)
  {
    path_config.Pass_cross_line_times++;
    path_config.Cross_line_detected_times = -1;
    if(path_config.Pass_cross_line_times >= 3)
    {
      current_mode = READY_DOTTED_LINE_MODE; // 准备进入虚线模式
      path_config.Pass_cross_line_times = 0; // 十字路口通过次数清零
      pose.total_distance = 0.0f; // 启用总距离计数器 

      // 此为后续需要准备进入直角转弯的距离
      path_config.Ready_angle_distance_Continuous_angle = First_distance;
    }
  }
}

void Turn_control(void) // 转向环控制
{ 
  if (count % robot_config.sampling_period == 0)
  {
    float photo_error = Calculate_Photo_Error();
    
    Error_MAX = FindMax_WeightedValue(weighted_sum_record, valid_count ,diff_buffer_photo_error); // 更新光电管误差最大值
    
    if(current_mode == RESTORE_NORMAL_MODE)
    {
      photo_error = Restore_mode(photo_error);
    }
    else
    {
      if(If_on_roundabout()) // 环岛情况
      {
        photo_error = Roundabout_mode();
      }
      else if(If_on_right_angle_turn(photo_error)) // 准备进入直角转弯模式
      { // 直角转弯情况
        if(current_mode == READY_RIGHT_ANGLE_MODE) // 进入直角转弯模式
        {
          photo_error = Ready_right_angle_mode(photo_error);
        }
        else if(current_mode == START_RIGHT_ANGLE_MODE) // 保持直角转弯模式
        {
          photo_error = Right_angle_mode();
        }
        else if(current_mode == EXIT_RIGHT_ANGLE_MODE) // 一般情况
        {
          direction_pid.kp = record_kp;  
          direction_pid.kd = record_kd;
        }
      }
      else if(photo_error == 9999) // 丢线情况
      {  
        photo_error = Loseline_mode();
      }
      else if(current_mode == READY_DOTTED_LINE_MODE) // 准备通过虚线情况
      {
        direction_pid.kp = record_kp;  
        direction_pid.kd = record_kd;
        direction_pid.GKD = record_gkd;
      }
      else if(If_on_cross_line(photo_error)) // 十字路口情况
      {
        Cross_line_mode();
      } 
      else // 一般情况
      {
        photo_error = Normal_mode(photo_error);
      }
    }

    if(right_angle_turn_count >= RIGHT_ANGLE_TURN_COUNT) // 退出直角转弯模式并进入恢复模式
    {
      current_mode = RESTORE_NORMAL_MODE; 
      right_angle_detect_flags = 0;  // 直角弯检测次数重置，防止上次使用时未置0的检测次数影响到下一次直角弯的连续帧判断
      right_angle_turn_count = 0; // 直角转弯计数器重置
    } 
  
    if(restore_count >= RESTORE_NORMAL_COUNT) // 退出恢复模式并进入正常模式
    {
      current_mode = EXIT_RIGHT_ANGLE_MODE; 
      direction_pid.GKD = record_gkd; // 恢复最初的gkd值
      record_error = 0.0f; // 直角弯误差记录重置
      restore_count = 0; // 恢复计数器重置
    }
    
    if(roundabout_count >= ROUNDABOUT_COUNT)
    {
      current_mode = EXIT_RIGHT_ANGLE_MODE;
      roundabout_count = 0; // 环岛计数器重置
      roundabout_detect_flags = 0; // 环岛检测次数重置
    }
    Direction_actual = photo_error;
    ComeputePID_Position(&direction_pid, Direction_actual, TURN);
  }
}

void Speed_Control(void)
{ // 速度环控制
  if (count % robot_config.sampling_period == 0)
  {
    Left_actual = sensor_data.left_encoder_count;
    Right_actual = sensor_data.right_encoder_count;

    ComeputePID_Position(&speed_pid_left, Left_actual, LEFT_MOTOR);
    ComeputePID_Position(&speed_pid_right, Right_actual, RIGHT_MOTOR);
  
    Left_pwm = speed_pid_left.output;
    Right_pwm = speed_pid_right.output;

    /* 最终输出限幅（包含负数情况） */
    /* 限幅：避免嵌套三目运算，增强可读性 */
    if (Left_pwm > FINAL_OUTPUTMAX)
    {
      Left_pwm = FINAL_OUTPUTMAX;
    }
    else if (Left_pwm < FINAL_OUTPUTMIN)
    {
      Left_pwm = FINAL_OUTPUTMIN;
    }

    if (Right_pwm > FINAL_OUTPUTMAX)
    {
      Right_pwm = FINAL_OUTPUTMAX;
    }
    else if (Right_pwm < FINAL_OUTPUTMIN)
    {
      Right_pwm = FINAL_OUTPUTMIN;
    }
  }
}

void Set_Motor_PWM(int8_t motor, int32_t final_pwm)
{ // 电机驱动函数
  if (motor == LEFT_MOTOR)
  { // 左电机驱动
    if (final_pwm > 0)
    {
      TIM1->CCR1 = final_pwm;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // 左电机正转
    }
    else
    {
      TIM1->CCR1 = -final_pwm;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // 左电机反转
    }
  }

  if (motor == RIGHT_MOTOR)
  { // 右电机驱动
    if (final_pwm > 0)
    {
      TIM1->CCR2 = final_pwm;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // 右电机正转
    }
    else
    {
      TIM1->CCR2 = -final_pwm;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // 右电机反转
    }
  }
}


int fputc(int ch, FILE *f) // 重定义函数
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ // 定时器中断
  if (htim == &htim2)
  {
    Read_sensors();
    Update_odometry();
    Turn_control();
    Compute_target(LEFT_MOTOR);
    Compute_target(RIGHT_MOTOR);
    Speed_Control();
    Set_Motor_PWM(LEFT_MOTOR, Left_pwm);
    Set_Motor_PWM(RIGHT_MOTOR, Right_pwm);
    count++;
    if (count > 1000)
    {
      count = 1;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  dodo_BMI270_init(); // 初始化陀螺仪
  PID_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  TIM1->CCR4 = 6500;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // // 速度环pid调试输出
    // if(count % 1 ==0){
    //   printf("%d %d %f %f %d %d\r\n", speed_pid_left.actual, speed_pid_right.actual, speed_pid_left.output, speed_pid_right.output, speed_pid_left.target, speed_pid_right.target);
    // }

    //以下为陀螺仪使用示例
    dodo_BMI270_get_data(); // 调用此函数会更新陀螺仪数据
    gyro_z=BMI270_gyro_transition(BMI270_gyro_z); // 将原始陀螺仪数据转换为物理值，单位为度每秒
    
    if(count % 100 == 0)
    {
      printf("%d\r\n",path_config.Pass_cross_line_times); // 输出当前模式，测试是否成功启动，正常使用时不需要这行代码
    }

    //  if(count % 200){
    //   printf("%f\r\n",gyro_z);//输出陀螺仪读数，测试是否成功启动，正常使用时不需要这行代码
    //  }


      // accel_x=BMI270_acc_transition(BMI270_accel_x);//将原始加速度计数据转换为物理值，单位为g，一般不需要使用此数据
    // accel_y=BMI270_acc_transition(BMI270_accel_y);
    // accel_z=BMI270_acc_transition(BMI270_accel_z);
    // printf("G: %f %f %f | A: %f %f %f\r\n", gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);//输出陀螺仪读数，测试是否成功启动，正常使用时不需要这行代码

    // //以下为读取光电管的示例（从左到右编号0~11）
    // uint16_t mux_value;
    // MUX_get_value(&mux_value);
    // for(int i=0;i<=11;i++){
    //   printf("%d,",MUX_GET_CHANNEL(mux_value,i));//获取第i个光电管的数值并输出
    // }
    // printf("\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
