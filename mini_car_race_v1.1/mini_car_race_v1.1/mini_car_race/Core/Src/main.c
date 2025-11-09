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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PHOTO_NUM 12             // 光电管数量
#define integralLimit 20000      // 积分最大值
#define FILTER_SIZE 5           // 微分滤波窗口数量
#define FILTER_SIZE_ERROR 100    // 光电管误差滤波窗口数量
#define HIGH_BASE_SPEED 70       // 高速基准速度
#define LOW_BASE_SPEED 40        // 低速基准速度     

#define LEFT_OUTPUTMAX 3600      // 左电机速度环输出最大值
#define LEFT_OUTPUTMIN -3600     // 左电机速度环输出最小值
#define RIGHT_OUTPUTMAX 3600     // 右电机速度环输出最大值
#define RIGHT_OUTPUTMIN -3600    // 右电机速度环输出最小值
#define TURN_OUTPUTMAX 3000      // 转向环输出最大值
#define TURN_OUTPUTMIN -3000     // 转向环输出最小值
#define FINAL_OUTPUTMAX 5400     // 最终输出最大值
#define FINAL_OUTPUTMIN -5400    // 最终输出最小值
#define PHOTO_ERROR_LIMIt 370.0f // 判断直角弯的光电管误差阈值
#define PHOTO_ERROR_MAX 800.0f   // 光电管误差能达到的最大值
#define PHOTO_ERROR_MIN -800.0f  // 光电管误差能达到的最小值

#define RIGHT_ANGLE_TURN_KP 0.25f // 直角转弯时的kp值
#define RIGHT_ANGLE_TURN_KD 0.08f // 直角转弯时的kd值
#define Lose_line_KP 0.25f         // 丢线时的kp值
#define lose_line_KD 0.08f        // 丢线时的kd值
#define RESTORE_KP 0.1f           // 恢复模式的kp值
#define RESTORE_KD 0.03f          // 恢复模式的kd值

#define DETECT_TIMES 4           // 直角转弯的检测次数

#define RIGHT_ANGLE_TURN_COUNT 100    // 直角转弯模式计数器阈值
#define RESTORE_NORMAL_COUNT 500     // 恢复模式计数器阈值

#define LEFT_MOTOR -1              // 左电机标志
#define RIGHT_MOTOR 1              // 右电机标志
#define TURN 0                     // 转向环标志

#define START_RIGHT_ANGLE_MODE 1   // 进入直角转弯模式标志
#define EXIT_RIGHT_ANGLE_MODE 0    // 退出直角转弯模式标志
#define READY_RIGHT_ANGLE_MODE 2   // 准备进入直角转弯模式标志
#define RESTORE_NORMAL_MODE 3      // 进入恢复模式标志
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

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

volatile static float buf[100];  // UART发送缓冲区

volatile static float Error_MAX = 0.0f; // 光电管误差最大值

volatile static float record_error = 0.0f;    // 直角转弯时光电管误差记录

volatile static int8_t detect_flags = 0;  //直角弯检测次数

volatile uint8_t valid_count = 0;                     // 光电管亮起数量
volatile uint8_t *valid_count_address = &valid_count; // 光电管亮起数量地址

volatile int16_t Left_actual = 0, Right_actual = 0, Direction_actual = 0; // 左右电机实际速度,和转向环实际位置
volatile int16_t Left_pwm = 0, Right_pwm = 0;                             // 左右电机输出的pwm

volatile static uint32_t count = 0;            // 时间计数器
volatile static uint32_t right_angle_turn_count = 0;       // 直角转弯计数器
volatile static uint32_t restore_count = 0;    // 恢复计数器 
volatile static float weighted_sum_record = 0; // 上一次光电管误差记录

volatile static float record_kp = 0.0f;                  // 用于记录转向环kp值
volatile static float record_kd = 0.0f;                  // 用于记录转向环kd值

volatile static int8_t if_right_angle_turn_mode = EXIT_RIGHT_ANGLE_MODE;   // 是否处于直角转弯模式标志

PID speed_pid_left, speed_pid_right;                     // 速度环PID声明
PID direction_pid;                                       // 转向环PID声明
float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; // 陀螺仪数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// 参数说明：weighted_value 光电管误差，diff_buffer_photo_error 光电管误差缓冲区
// 功能：更新缓冲区,寻找光电管误差最大值并返回
float FindMax_WeightedValue(float weighted_value, volatile float dierroff_buffer_photo_error[])
{ // 光电管误差寻最大值函数
  // 更新滑动窗口
  dierroff_buffer_photo_error[buf_index_error] = weighted_value;
  buf_index_error = (buf_index_error + 1) % FILTER_SIZE_ERROR;
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

float Calculate_Photo_Error(void)
{ // 光电管误差计算函数
  valid_count = 0;
  int16_t weighted_sum = 0;
  uint16_t photo_value = 0;

  MUX_get_value(&photo_value);// 获取每个通道的返回值

  for (int i = 0; i < PHOTO_NUM; i++)
  {
    if ((photo_value >> (PHOTO_NUM - i - 1)) & 1)
    {                                                
      weighted_sum += (2 * i - PHOTO_NUM + 1) * 40; // 计算加权和
      valid_count++;
    }
  }

  if (valid_count == 0)
  {              // 特殊值表示丢线
    return 9999; // 丢线直接退出，防止weighted_sum_record被更新为9999
  }
  weighted_sum_record = (float)weighted_sum / valid_count; // 记录光电管误差并对其进行滤波处理,除以亮起数量取平均，是为了防止亮起数量变化导致误差突变
  return weighted_sum_record;                              // 返回光电管误差
}

void Collect_photo_error(void)
{  // 光电管误差采集函数
  if(count % 1 == 0){
    volatile float photo_error = Calculate_Photo_Error();
    if(photo_error == 9999||(*valid_count_address >=7 && *valid_count_address <=9)){
        count = 50; // 此时强制开启转换环的直角转弯模式
    } 
    }
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
    float gyro_z_acquire = filtered_gyro_z(gyro_z); // 获取滤波后的陀螺仪数据
    pid->output = pid->kp * pid->nowError + abs(pid->nowError) * pid->nowError * pid->kp2 + pid->ki * pid->integral + pid->derivative + pid->GKD * gyro_z_acquire;
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
    if(if_right_angle_turn_mode == START_RIGHT_ANGLE_MODE) // 直角转弯模式下基准速度线性降为LOW_BASE_SPEED
    {
      speed_pid_left.target = LOW_BASE_SPEED - direction_pid.output;
    }
    else if(if_right_angle_turn_mode == RESTORE_NORMAL_MODE) // 恢复模式下基准速度线性增长恢复到HIGH_BASE_SPEED
    {
      speed_pid_left.target = LOW_BASE_SPEED + (HIGH_BASE_SPEED - LOW_BASE_SPEED) * ((float)restore_count / RESTORE_NORMAL_COUNT) - direction_pid.output;
    }
    else // 一般情况
    {
      speed_pid_left.target = HIGH_BASE_SPEED - direction_pid.output;
    }
  }
  else if (motor == RIGHT_MOTOR)
  {
    if(if_right_angle_turn_mode == START_RIGHT_ANGLE_MODE) // 直角转弯模式下基准速度线性降为LOW_BASE_SPEED
    {
      speed_pid_right.target = LOW_BASE_SPEED + direction_pid.output;
    }
    else if(if_right_angle_turn_mode == RESTORE_NORMAL_MODE) // 恢复模式下基准速度线性增长恢复到HIGH_BASE_SPEED
    {
      speed_pid_right.target = LOW_BASE_SPEED + (HIGH_BASE_SPEED - LOW_BASE_SPEED) * ((float)restore_count / RESTORE_NORMAL_COUNT) + direction_pid.output;
    }
    else // 一般情况
    {
      speed_pid_right.target = HIGH_BASE_SPEED + direction_pid.output;
    }
  }
}

void PID_Init(void)
{ // 初始化PID参数
  direction_pid.kp = 0.1f;
  direction_pid.kp2 = 0.0001f;
  direction_pid.ki = 0.0f;
  direction_pid.kd = 0.0f;
  direction_pid.GKD = -0.1f;
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
}

float Right_angle_mode(void) // 直角转弯模式函数
{
  //直角转弯模式下kp随时间线性变化
  float comepute_kp = (float)RIGHT_ANGLE_TURN_KP - (RIGHT_ANGLE_TURN_KP - record_kp) * (float)right_angle_turn_count / RIGHT_ANGLE_TURN_COUNT; 
  
  direction_pid.kp = comepute_kp;
  direction_pid.kd = RIGHT_ANGLE_TURN_KD;
  right_angle_turn_count++;
  return record_error; // 返回已经记录的误差
}

float Ready_right_angle_mode(float photo_error) // 准备进行直角转弯模式函数
{
  if(detect_flags >= DETECT_TIMES - 1) // 判断是否连续多次满足进入直角转弯模式条件
  {
    if_right_angle_turn_mode = START_RIGHT_ANGLE_MODE;
    direction_pid.kp = RIGHT_ANGLE_TURN_KP; // 切换为直角转弯时的kp和kd值
    direction_pid.kd = RIGHT_ANGLE_TURN_KD; 
    return record_error;
  }
  else if(detect_flags == 0)
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

  detect_flags++; // 完成一次对直角弯标志的判断
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
  else
  {
    photo_error = weighted_sum_record;
  }
  restore_count++;

  return photo_error;
}


void Turn_control(void)
{ // 转向环控制
  if (count % 1 == 0)
  {
    float photo_error = Calculate_Photo_Error();
    
    Error_MAX = FindMax_WeightedValue(weighted_sum_record, diff_buffer_photo_error); // 更新光电管误差最大值

    if(if_right_angle_turn_mode != RESTORE_NORMAL_MODE)
    {
      if((*valid_count_address >= 7 && *valid_count_address <= 9) || if_right_angle_turn_mode == START_RIGHT_ANGLE_MODE)
      { // 直角转弯情况
        if(if_right_angle_turn_mode == READY_RIGHT_ANGLE_MODE) // 进入直角转弯模式
        {
          photo_error = Ready_right_angle_mode(photo_error);
        }
        else if(if_right_angle_turn_mode == START_RIGHT_ANGLE_MODE) // 保持直角转弯模式
        {
          photo_error = Right_angle_mode();
        }
        else if(if_right_angle_turn_mode == EXIT_RIGHT_ANGLE_MODE) // 一般情况
        {
          direction_pid.kp = record_kp;  
          direction_pid.kd = record_kd;
        }
      }
      else if(photo_error == 9999) // 丢线情况
      {  
        direction_pid.kp = Lose_line_KP;
        direction_pid.kd = lose_line_KD;
        photo_error = Error_MAX; // 保持前一段路程的最大误差
        detect_flags = 0; // 直角弯检测次数重置
        if_right_angle_turn_mode = EXIT_RIGHT_ANGLE_MODE; // 退出直角转弯模式
      }
      else // 一般情况
      {
        if((fabs(photo_error) < PHOTO_ERROR_LIMIt) && (*valid_count_address == 3 || *valid_count_address == 4 || *valid_count_address == 5)) // 准备进入直角转弯模式
        {
          if_right_angle_turn_mode = READY_RIGHT_ANGLE_MODE; 
        }
        else // 退出准备进入直角转弯模式
        {
          if_right_angle_turn_mode = EXIT_RIGHT_ANGLE_MODE;
        }
        detect_flags = 0;  // 直角弯检测次数重置，防止上次使用时未置0的检测次数影响到下一次直角弯的连续帧判断
        direction_pid.kp = record_kp;  
        direction_pid.kd = record_kd;
      }
    }
    else // 恢复模式情况
    { 
      photo_error = Restore_mode(photo_error);
    }
    
    if(right_angle_turn_count >= RIGHT_ANGLE_TURN_COUNT) // 退出直角转弯模式并进入恢复模式
    {
      if_right_angle_turn_mode = RESTORE_NORMAL_MODE; 
      detect_flags = 0;  // 直角弯检测次数重置，防止上次使用时未置0的检测次数影响到下一次直角弯的连续帧判断
      right_angle_turn_count = 0; // 直角转弯计数器重置
    } 
  
    if(restore_count >= RESTORE_NORMAL_COUNT) // 退出恢复模式并进入正常模式
    {
      if_right_angle_turn_mode = EXIT_RIGHT_ANGLE_MODE; 
      record_error = 0.0f; // 直角弯误差记录重置
      restore_count = 0; // 恢复计数器重置
    }

    Direction_actual = photo_error;
    ComeputePID_Position(&direction_pid, Direction_actual, TURN);
  }
}

void Speed_Control(void)
{ // 速度环控制
  if (count % 1 == 0)
  {
    Left_actual = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); // 获取当前速度
    Right_actual = -(int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    __HAL_TIM_SET_COUNTER(&htim4, 0); // 重置计数器
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    ComeputePID_Position(&speed_pid_left, Left_actual, LEFT_MOTOR);
    ComeputePID_Position(&speed_pid_right, Right_actual, RIGHT_MOTOR);
  
    Left_pwm = speed_pid_left.output - direction_pid.output;
    Right_pwm = speed_pid_right.output + direction_pid.output;
     //Left_pwm = speed_pid_left.output;
     //Right_pwm = speed_pid_right.output;

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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|R_DIR_Pin|MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin|MUX_1_Pin|MUX_2_Pin|MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R_DIR_Pin */
  GPIO_InitStruct.Pin = R_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin|MUX_2_Pin|MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
