#include "zf_common_headfile.h"
#include "math.h"


#define kp 				20.00f
#define ki 				0.001f
#define cycle_T 	0.008f //125Hz
#define half_T 		0.0025f
#define offset_n  200.0f//初始化误差采样次数



param_imu imu_data;
param_Angle imu_Angle_Filted;
param_Angle imu_Angle;

float q[4] = {1.0,0.0,0.0,0.0};//初始单位四元数
float exInt =0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;//重力数据偏差
float offset[6];//开始的原始数据误差值
float g=9.8f;//初始化重力加速度
float yaw_err;//yaw角的零偏误差

//////////////////////////////////////////////////////////////////////////////////
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);  //0x5f3759df是一个平方根倒数速算法
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//////////////////////////////////////////////////////////////////////////////////


//获取姿态角数据
void IMU_GetValues(void)
{
	 imu_data.AX = ((float)mpu6050_acc_x)/2048;
	 imu_data.AY = ((float)mpu6050_acc_y)/2048;
   imu_data.AZ = ((float)mpu6050_acc_z)/2048;
	
   imu_data.GX = ((float)mpu6050_gyro_x)*0.001064;
   imu_data.GY = ((float)mpu6050_gyro_y)*0.001064;
   imu_data.GZ = ((float)mpu6050_gyro_z)*0.001064;
}

//初始化IMU,采集offset_n次数据减去零点误差
void IMU_Init(void)
{
	int i=0;
	for(i=0;i<offset_n;i++)
	{
		IMU_GetValues();
		offset[0]+=imu_data.AX;
		offset[1]+=imu_data.AY;
		offset[2]+=imu_data.AZ;
		offset[3]+=imu_data.GX;
		offset[4]+=imu_data.GY;
		offset[5]+=imu_data.GZ;
	}
	for(i=0;i<6;i++)
	{
		offset[i]/=offset_n;
	}
}

//四元数解算
void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
	
	ax = imu_temp->AX-offset[0];
	ay = imu_temp->AY-offset[1];
	az = imu_temp->AZ-offset[2]+g;
	gx = imu_temp->GX-offset[3];
	gy = imu_temp->GY-offset[4];
	gz = imu_temp->GZ-offset[5];
	
	float vx, vy, vz; 
	float ex, ey, ez; 
	
	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];
	
	// 仅在加速度计测量有效时计算反馈（避免加速度计归一化时的NaN）
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		float norm = fast_sqrt(ax*ax+ay*ay+az*az);
		ax = ax * norm;
		ay = ay * norm;
		az = az * norm;
	}
	
	vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q2*q3 + q0*q1);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	
	exInt += ki * ex;
	eyInt += ki * ey;
	ezInt += ki * ez;
	
	gx += kp * ex + exInt;
	gy += kp * ey + eyInt;
	gz += kp * ez + ezInt;
	
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;
	
	float recipNorm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
	q[0] = q0 * recipNorm;
	q[1] = q1 * recipNorm;
	q[2] = q2 * recipNorm;
	q[3] = q3 * recipNorm;
	
}

void IMU_getEuleranAngles(void)
{
	IMU_GetValues();
	IMU_AHRSupdate(&imu_data);
	float yaw_Last=imu_Angle.Yaw;
	imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
	imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;
	imu_Angle.Yaw  += imu_data.GZ* 57.2957* cycle_T* 4;
	float error=imu_Angle.Yaw-yaw_Last;
	if(error>-0.008&&error<0.008)
	{
		yaw_err+=error;
	}
	//Angle低通滤波
	imu_Angle_Filted.Pitch = 0.2*imu_Angle.Pitch + 0.8*imu_Angle_Filted.Pitch;
	imu_Angle_Filted.Roll  = 0.2*imu_Angle.Roll  + 0.8*imu_Angle_Filted.Roll;	
	imu_Angle_Filted.Yaw   = 0.2*(imu_Angle.Yaw-yaw_err) + 0.8*imu_Angle_Filted.Yaw;
}   
    
