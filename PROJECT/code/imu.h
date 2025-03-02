#ifndef __IMU_H
#define __IMU_H

			
typedef struct{
	float AX;
	float AY;
	float AZ;
	float GX;
	float GY;
	float GZ;
				}param_imu;

	typedef struct{
	float Pitch;
	float Roll;
	float Yaw;
				}param_Angle;

extern param_Angle imu_Angle_Filted;
extern param_imu imu_data;
void IMU_getEuleranAngles(void);
void IMU_Init(void);
							
#endif

