/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "stdio.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// *************************** 例程硬件连接说明 ***************************
// 核心板正常供电即可 无需额外连接


// *************************** 例程测试说明 ***************************


// **************************** 代码区域 ****************************
#define CHANNEL_NUMBER          (4)

#define PWM_CH1                 (TIM5_PWM_CH1_A0)
#define PWM_CH2                 (TIM5_PWM_CH2_A1)
#define PWM_CH3                 (TIM5_PWM_CH3_A2)
#define PWM_CH4                 (TIM5_PWM_CH4_A3)

#define Motor1_PWM              (PWM_CH4)           //左轮
#define Motor2_PWM              (PWM_CH2)           //右轮

#define Motor1_DIR              (A2)                //左轮
#define Motor2_DIR              (A0)                //右轮

#define PIT                             (TIM6_PIT )                              // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (TIM6_IRQn)                              // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

#define ENCODER1_QUADDEC                 (TIM3_ENCODER)                          // 正交编码器对应使用的编码器接口 这里使用 TIM3 的编码器功能
#define ENCODER1_QUADDEC_A               (TIM3_ENCODER_CH1_B4)                   // A 相对应的引脚
#define ENCODER1_QUADDEC_B               (TIM3_ENCODER_CH2_B5)                   // B 相对应的引脚
 
#define ENCODER2_QUADDEC		                (TIM4_ENCODER)                   // 带方向编码器对应使用的编码器接口 这里使用 TIM4 的编码器功能
#define ENCODER2_QUADDEC_A               (TIM4_ENCODER_CH1_B6)                   // A 对应的引脚
#define ENCODER2_QUADDEC_B                 (TIM4_ENCODER_CH2_B7)                 // B 对应的引脚

#define PWM_PERIOD 30000  // PWM周期 30KHz 
#define MOTOR_DEAD_VAL 800  // 电机死区值
#define MOTOR_MAX 3000  // 电机最大速度
#define MOTOR_STOP 400  // 电机停止速度 小于这个值认为电机停止 输出PWM为0

#define PIT_TIME 1  // 进入中断的时间间隔 单位 ms

#define MECH_MID -2.5f       // 定义机械中值（直立时目标角度），单位：度
#define CAR_ANGLE_SET   (MECH_MID+velocityControlOut)  // 目标角度 单位：度

// 速度积分限幅
#define SPEED_INTEGRAL_MAX       15     // 积分上限
#define SPEED_INTEGRAL_MIN      -15     // 积分下限
#define SPEED_CONTROL_PERIOD    25      // 速度控制周期（25ms）
#define SPEED_TARGET            0       // 默认目标速度（可以根据需要修改）
#define VELOCITY_FILTER_FACTOR  0.7f    // 速度低通滤波系数

/******************************************************** */
uint8_t Flag_IsStand = 0;               // 是否直立标志位

int16 encoder1_data = 0;
int16 encoder2_data = 0;
volatile int16 leftMotorPulseSigma = 0; // 编码器累加变量（假设在编码器中断中累加，每次控制后需清零）
volatile int16 rightMotorPulseSigma = 0;// 编码器累加变量（假设在编码器中断中累加，每次控制后需清零）
uint16 time = 0;                        // 在pit中断中的时间计数

// 速度外环 PID 参数
float velocity_kp = 0.015f;             // 速度环比例系数
float velocity_ki = 0.0005f;            // 速度环积分系数
// 速度外环 PID 内部变量
float carSpeed = 0.0f;                 // 当前车速
float carSpeedLast = 0.0f;             // 上一次滤波后的车速
float speedError = 0.0f;               // 速度误差
float speedIntegral = 0.0f;            // 速度误差积分

float P, I;  // 比例项和积分项  调试用 要删除


// 速度控制输出
float velocityControlOut = 0.0f;       // 速度环输出
/******************************************************** */
// 角度内环 PID 参数
float angle_kp = 600.0f;        // 比例系数
float angle_kd = 1000.0f;        // 微分系数
// 角度内环 PID 内部变量
float angleIntegral = 0.0f;   // 积分累计
float angleLastError = 0.0f;  // 上一次误差
float angleControlOut = 0.0f; // 角度内环 PID 输出
/******************************************************** */
typedef enum
{
    Left  = 0,
    Right = 1,
}motor_enum;

void Motor_Init(void)
{
    gpio_init(Motor1_DIR,GPO,GPIO_HIGH,GPO_PUSH_PULL);                          // 初始化电机1方向控制引脚
    gpio_init(Motor2_DIR,GPO,GPIO_HIGH,GPO_PUSH_PULL);                          // 初始化电机2方向控制引脚

    pwm_init(Motor1_PWM, 30000, 0);                                                // 初始化 PWM 通道 频率 30KHz 初始占空比 0%
    pwm_init(Motor2_PWM, 30000, 0);                                                // 初始化 PWM 通道 频率 30KHz 初始占空比 0%
    
    gpio_set_level(Motor1_DIR,GPIO_HIGH);                                       // 设置电机1方向为正转
    gpio_set_level(Motor2_DIR,GPIO_HIGH);                                       // 设置电机2方向为正转


}

/******************
 * 函数名：Motor_SetSpeed
 * 描述  ：设置电机速度
 * 输入  ：motor  电机选择
 *        speed  速度 范围 -10000~10000
 * 输出  ：无
 */
void Motor_SetSpeed(motor_enum motor, int16 speed)
{
    //调试用 不然我桌子鸡飞蛋打
    if (imu_Angle_Filted.Pitch - CAR_ANGLE_SET > 60 || imu_Angle_Filted.Pitch - CAR_ANGLE_SET < -60)
    {
        speed = 0;
    }
    
    if(speed > -MOTOR_STOP && speed < MOTOR_STOP)                                    // 速度小于电机停止速度
    {
        speed = 0;                                                                  // 设置速度为0
    }
    else if(0<speed&&speed<MOTOR_DEAD_VAL)                                           // 速度小于电机死区值
    {
        speed = MOTOR_DEAD_VAL;                                                     // 设置速度为电机死区值
    }
    else if(0>speed&&speed>-MOTOR_DEAD_VAL)                                         // 速度小于电机死区值
    {
        speed = -MOTOR_DEAD_VAL;                                                    // 设置速度为电机死区值
    }
    speed = (speed>MOTOR_MAX)?MOTOR_MAX:(speed<-MOTOR_MAX)?-MOTOR_MAX:speed;        // 限制速度范围


    // DIR1 为1 电机反转
    // DIR1 为0 电机正转
    // DIR2 为0 电机反转
    // DIR2 为1 电机正转
    if(speed > 0)
    {
        if(motor == Left)
        {
            gpio_set_level(Motor1_DIR,GPIO_LOW);                                        // 设置电机1方向为正转
            pwm_set_duty(Motor1_PWM, speed);                                            // 更新对应通道占空比
        }
        else
        {
            gpio_set_level(Motor2_DIR,GPIO_HIGH);                                       // 设置电机2方向为正转
            pwm_set_duty(Motor2_PWM, speed);                                            // 更新对应通道占空比
        }
    }
    else
    {
        if(motor == Left)
        {
            gpio_set_level(Motor1_DIR,GPIO_HIGH);                                        // 设置电机1方向为反转
            pwm_set_duty(Motor1_PWM, -speed);                                            // 更新对应通道占空比
        }
        else
        {
            gpio_set_level(Motor2_DIR,GPIO_LOW);                                        // 设置电机2方向为反转
            pwm_set_duty(Motor2_PWM, -speed);                                           // 更新对应通道占空比
        }
    }
}


//==================================================================
// 函数名：Velocity_Control
// 描述：速度环PI控制器，每25ms调用一次
// 参数：target - 目标速度
// 返回：速度环输出值（作为直立环的角度补偿）
//==================================================================
float Velocity_Control(float target)
{

    float encoderAvg;
    
    // 1. 获取当前速度 - 左右编码器平均值
    encoderAvg = (leftMotorPulseSigma + rightMotorPulseSigma) * 0.5f;
    
    // 清零累计值，为下一周期做准备
    leftMotorPulseSigma = rightMotorPulseSigma = 0;
    
    // 2. 低通滤波平滑处理速度值
    carSpeed = VELOCITY_FILTER_FACTOR * carSpeedLast + (1.0f - VELOCITY_FILTER_FACTOR) * encoderAvg;
    carSpeedLast = carSpeed;  // 保存本次滤波后的速度
    
    // 3. 计算速度偏差（目标速度 - 实际速度）
    speedError = target - carSpeed;
    
    // 4. 计算比例项和积分项
    P = speedError * velocity_kp;
    I = speedError * velocity_ki;
    
    // 5. 累加积分项
    speedIntegral += I;
    
    // 6. 积分限幅
    if (speedIntegral > SPEED_INTEGRAL_MAX)
        speedIntegral = SPEED_INTEGRAL_MAX;
    else if (speedIntegral < SPEED_INTEGRAL_MIN)
        speedIntegral = SPEED_INTEGRAL_MIN;
    
    // 7. 计算速度环PI控制输出
    velocityControlOut = P + speedIntegral;
    
    return velocityControlOut;
}


// 角度内环 PID 控制函数，每 5ms调用一次（与 IMU 解算周期一致）
// 根据 imu_Angle_Filted.Pitch 与目标机械中值 MECH_MID 的误差计算 PID 输出
void Angle_PID_Control(void)
{
    float error;
    float A_P, A_D;
    // static float A_D_last = 0.0f;
    error = imu_Angle_Filted.Pitch - CAR_ANGLE_SET;  // 计算角度误差
    
    // 计算比例项
    A_P = error * angle_kp;
    
    // 计算微分项
    // 给角速度环控制输出一个低通滤波 
    //A_D = (-imu_data.GY)*angle_kd; // 与P项时序没对齐 浪费我一天调参 不如error相减
    A_D = angle_kd * (error - angleLastError);
    angleLastError = error;
    
    // 角度内环控制输出
    angleControlOut =  A_P+A_D ;
    printf("%f,%f,%f\n",error,A_P,A_D);
    // 此处可根据实际需要，将 angleControlOut 与速度外环平滑输出叠加
    // 或作为独立内环输出直接用于后续电机 PWM 控制调整直立角度
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart

    // 此处编写用户代码 例如外设初始化代码等
    Motor_Init();                             // 初始化电机
    mpu6050_init(); 

    encoder_quad_init(ENCODER1_QUADDEC, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);          // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER2_QUADDEC, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);          // 初始化编码器模块与引脚 带方向增量编码器模式

    pit_ms_init(PIT, PIT_TIME);                                                           // 初始化 PIT 为周期中断

    interrupt_set_priority(PIT_PRIORITY, 0);                                              // 设置 PIT 对周期中断的中断优先级为 0
    // 此处编写用户代码 例如外设初始化代码等

                                                                        // 初始化 MPU6050
    //Motor_SetSpeed(Right, 2000);                                                            
    while(1)
    {
        //printf("%f,%f,%f,%f,%f\n",P,I,velocityControlOut,angleControlOut,imu_Angle_Filted.Pitch-MECH_MID);
        system_delay_ms(10);
    }


}

// **************************** 代码区域 ****************************




// **************************** 中断处理函数 ****************************
// 设置为

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{   
    time++;
    encoder1_data = encoder_get_count(ENCODER1_QUADDEC);                           // 获取编码器计数
    encoder2_data = -encoder_get_count(ENCODER2_QUADDEC);                          // 获取编码器计数
    encoder_clear_count(ENCODER1_QUADDEC);                                         // 清空编码器计数
    encoder_clear_count(ENCODER2_QUADDEC);                                         // 清空编码器计数

    leftMotorPulseSigma  += encoder1_data;                                         // 累加左轮脉冲
    rightMotorPulseSigma += encoder2_data;                                         // 累加右轮脉冲


    // 每次25ms进入速度环闭环pid控制
    // 每次5ms之中 进行mpu6050姿态解算 进行直立环控制



    if (time % 5 == 0)
    {
        mpu6050_get_acc();                                                             // 获取加速度计数据
        mpu6050_get_gyro();                                                            // 获取陀螺仪数据
        IMU_getEuleranAngles();                                                        // 获取欧拉角
        Angle_PID_Control();                                                           // 角度环控制
        Motor_SetSpeed(Left, angleControlOut);                                         // 左轮速度控制
        Motor_SetSpeed(Right, angleControlOut);                                        // 右轮速度控制
    }
    if (time % 25 == 0)
    {
        Velocity_Control(SPEED_TARGET);                                                // 速度环控制
        time=0;
    }


    
    
}

// *************************** 例程常见问题说明 ***************************
