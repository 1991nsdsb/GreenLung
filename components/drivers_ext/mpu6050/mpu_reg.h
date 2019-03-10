#ifndef MPU_REG
#define MPU_REG




typedef enum {
	SMPLRT_DIV	=0X19,
	CONFIG,
	GYRO_CONFIG,
	ACCEL_CONFIG,
	ACCEL_XOUT_H=0X3B,
	ACCEL_XOUT_L,
	ACCEL_YOUT_H,
	ACCEL_YOUT_L,
	ACCEL_ZOUT_H,
	ACCEL_ZOUT_L,
	
	TEMP_OUT_H=0x41,
	TEMP_OUT_L,
	
	GYRO_XOUT_H  =0X43,
	GYRO_XOUT_L,
	GYRO_YOUT_H,
	GYRO_YOUT_L,
	GYRO_ZOUT_H,
	GYRO_ZOUT_L,
	
	PWR_MEMT_1=0x6B,
	
	
}mpu_reg_address;


//低通滤波
#define DLPF_5HZ			(6)
#define DLPF_10HZ			(5)
#define DLPF_21HZ			(4)
#define DLPF_44HZ			(3)
#define DLPF_94HZ			(2)
#define DLPF_184HZ		(1)
#define DLPF_260HZ		(0)
#define DLPF_RESERVE	(7)

//低通滤波参数设置为0，7时，陀螺仪输出频率为8KHZ
//1-6时陀螺仪输出频率为1KHZ
//在陀螺仪输出频率为	1KHZ时使用以下宏定义采样频率
#define SAMPLE_50HZ		(19)
#define SAMPLE_125HZ	(7)

//不需要开启加速度计自检时使用
#define ACCEL_FS_2g		(0)
#define ACCEL_FS_4g		(8)
#define ACCEL_FS_8g		(0x10)
#define ACCEL_FS_16g	(0x18)

//不需要开启陀螺仪自检时使用
#define GYRO_FS_250		(0)
#define GYRO_FS_500		(8)
#define GYRO_FS_1000		(10)
#define GYRO_FS_2000	(18)


#define CLOCK_SEL_INTER8M		(0)
#define CLOCK_SEL_PllGyroX	(1)


#define WAKEUP	(0x00)


#endif
