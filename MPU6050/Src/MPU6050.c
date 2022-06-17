#include <MPU6050.h>
#include "stdio.h"

//�޸���־��2020 08-28 ע�������е�printf����֤û��printf��ʱ���������ʹ��

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short MPU6050_gyroRAW[3], MPU6050_accelRAW[3], sensors;
float MPU6050_Pitch, MPU6050_Roll, MPU6050_Yaw;
float MPU6050_PitchUncorected, MPU6050_RollUncorected, MPU6050_YawUncorected;

float MPU6050_PitchCorrectorRate = 0;
float MPU6050_RollCorrectorRate = 0;
float MPU6050_YawCorrectorRate = 0;
float MPU6050_PitchCorrector = 0;
float MPU6050_RollCorrector = 0;
float MPU6050_YawCorrector = 0;

//long MPU6050_time

float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

float MPU6050_gyroSensitivity;
float MPU6050_accSensitivity;

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;            // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03) {                   //����0x03ΪMPU6050
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);			//��ȡ��ǰ�����ǵ�״̬
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);			//���ݶ�ȡ��״̬����У׼
		
        mpu_get_accel_sens(&accel_sens);	//��ȡ��ǰ���ٶȼƵ�״̬
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);			//���ݶ�ȡ��״̬����У׼
		//printf("setting bias succesfully ......\r\n");
    }
}

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];

int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*��������:	    ���µ�ADC���ݸ��µ� FIFO���飬�����˲�����
*******************************************************************************/

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO ����
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;
	for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
	   sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);

    switch(range)
    {
    	case MPU6050_GYRO_FS_250: MPU6050_gyroSensitivity = 131; break;
    	case MPU6050_GYRO_FS_500: MPU6050_gyroSensitivity = 65.5; break;
    	case MPU6050_GYRO_FS_1000: MPU6050_gyroSensitivity = 32.8; break;
    	case MPU6050_GYRO_FS_2000: MPU6050_gyroSensitivity = 16.4; break;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);

    switch(range)
    {
    	case MPU6050_ACCEL_FS_2: MPU6050_accSensitivity = 16384; break;
    	case MPU6050_ACCEL_FS_4: MPU6050_accSensitivity = 8192; break;
    	case MPU6050_ACCEL_FS_8: MPU6050_accSensitivity = 4096; break;
    	case MPU6050_ACCEL_FS_16: MPU6050_accSensitivity = 2048; break;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				enabled =1   ˯��
			    enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_getDeviceID(void)
*��������:	    ��ȡ  MPU6050 WHO_AM_I ��ʶ	 ������ 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t MPU6050_testConnection(void)
*��������:	    ���MPU6050 �Ƿ��Ѿ�����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_initialize(void) {
	u8 temp[1]={0};
	u8 retry = 0;
	i2cRead(0x68,0x75,1,temp);
	// check for several times, don't give up too early
	do
	{
		i2cRead(0x68,0x75,1,temp);
		retry++;

		if(retry > 100) NVIC_SystemReset();
	}while(temp[0]!=0x68);

	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	MPU6050_setSleepEnabled(0);
	MPU6050_setI2CMasterModeEnabled(0);
	MPU6050_setI2CBypassEnabled(0);
}


/**************************************************************************
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MPU6050_DMPInit(void)
{ 
	u8 temp[1]={0};
	u8 retry = 0;
	i2cRead(0x68,0x75,1,temp);
	
//	printf("mpu_set_sensor complete ......\r\n");
//	printf("%d\n", temp[0]);

	// check for several times, don't give up too early
	do
	{
		i2cRead(0x68,0x75,1,temp);
		retry++;

		if(retry > 100) NVIC_SystemReset();
	}while(temp[0]!=0x68);

	delay_ms(100);
//	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
	{
//		printf("mpu_setting_sensor.....\r\n");
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
//			printf("mpu_set_sensor complete ......\r\n");
		}
//		printf("mpu configure fifo........\r\n");
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
//			printf("mpu_configure_fifo complete ......\r\n");
		}
//		printf("mpu setting sample rate......\r\n");
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		{
//			printf("mpu_set_sample_rate complete ......\r\n");
		}
//		printf("loading firmware......\r\n");
		if(!dmp_load_motion_driver_firmware())
		{
//			printf("dmp_load_motion_driver_firmware complete ......\r\n");
		}
//		printf("setting orientation.....\r\n");
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{
//			printf("dmp set_orientation complete ......\r\n");
		}
//		printf("enabling dmp features ......\r\n");
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO|
		DMP_FEATURE_GYRO_CAL))
		{
//			printf("dmp_enable_feature complete ......\r\n");
		}
//		printf("dmp setting fifo rate ......\r\n");
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		{
//			printf("dmp_set_fifo_rate complete ......\r\n");
		}
		run_self_test();
//		printf("setting dmp state.....\r\n");
		if(!mpu_set_dmp_state(1))
		{
//			printf("mpu_set_dmp_state complete ......\r\n");
		}
	}

	MPU6050_setZeroMotionDetectionThreshold(1); //set this value to make it very sensitive to movement
	MPU6050_setZeroMotionDetectionDuration(50); //make it as fast as possible detect if the sensor has no movement
}
/**************************************************************************
The output of this function will return to MPU6050_Pitch, MPU6050_Roll, MPU6050_Yaw
**************************************************************************/
int MPU6050_readDMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch, roll, yaw;

	if(dmp_read_fifo(MPU6050_gyroRAW, MPU6050_accelRAW, quat, &sensor_timestamp, &sensors, &more))
		return -1;

	if (sensors & INV_WXYZ_QUAT )
	{    
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

		 MPU6050_PitchUncorected = pitch;
		 MPU6050_RollUncorected = roll;
		 MPU6050_YawUncorected = yaw;

		 pitch += MPU6050_PitchCorrector;
		 pitch = fmod(pitch, 360);
		 if(pitch > 180) pitch -= 360;

		 roll += MPU6050_RollCorrector;
		 roll = fmod(roll, 360);
		 if(roll > 180) roll -= 360;

		 yaw += MPU6050_YawCorrector;
		 yaw = fmod(yaw,360);
		 if(yaw > 180) yaw -= 360;

		 MPU6050_Pitch = pitch;
		 MPU6050_Roll = roll;
		 MPU6050_Yaw = yaw;
	}

	return 0;
}

void MPU6050_readDMPAll(float* Pitch, float* Roll, float* Yaw)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch, roll, yaw;

	dmp_read_fifo(MPU6050_gyroRAW, MPU6050_accelRAW, quat, &sensor_timestamp, &sensors, &more);

	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

		 MPU6050_PitchUncorected = pitch;
		 MPU6050_RollUncorected = roll;
		 MPU6050_YawUncorected = yaw;

		 pitch += MPU6050_PitchCorrector;
		 pitch = fmod(pitch, 360);
		 if(pitch > 180) pitch -= 360;

		 roll += MPU6050_RollCorrector;
		 roll = fmod(roll, 360);
		 if(roll > 180) roll -= 360;

		 yaw += MPU6050_YawCorrector;
		 yaw = fmod(yaw, 360);
		 if(yaw > 180) yaw -= 360;

		 *Pitch	= MPU6050_Pitch	= pitch;
		 *Roll 	= MPU6050_Roll 	= roll;
		 *Yaw 	= MPU6050_Yaw 	= yaw;
	}
}

float MPU6050_readDMPPitch()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch;

	dmp_read_fifo(MPU6050_gyroRAW, MPU6050_accelRAW, quat, &sensor_timestamp, &sensors, &more);

	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;

		 MPU6050_PitchUncorected = pitch;

		 pitch += MPU6050_PitchCorrector;
		 pitch = fmod(pitch, 360);
		 if(pitch > 180) pitch -= 360;
		 MPU6050_Pitch = pitch;
	}

	return MPU6050_Pitch;
}
float MPU6050_readDMPRoll()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float roll;

	dmp_read_fifo(MPU6050_gyroRAW, MPU6050_accelRAW, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;

		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

		 MPU6050_RollUncorected = roll;

		 roll += MPU6050_RollCorrector;
		 roll = fmod(roll, 360);
		 if(roll > 180) roll -= 360;
		 MPU6050_Roll = roll;
	}

	return MPU6050_Roll;
}

float MPU6050_readDMPYaw()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float yaw;

	dmp_read_fifo(MPU6050_gyroRAW, MPU6050_accelRAW, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;

		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

		 MPU6050_YawUncorected = yaw;

		 yaw += MPU6050_YawCorrector;
		 yaw = fmod(yaw, 360);
		 if(yaw > 180) yaw -= 360;
		 MPU6050_Yaw = yaw;
	}
	else printf("skip...\n");

	return MPU6050_Yaw;
}
/**************************************************************************
�������ܣ���ȡMPU6050�����¶ȴ���������
��ڲ�������
����  ֵ�������¶�
**************************************************************************/
float MPU6050_readTemperature(void)
{	   
	float Temp;
	Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
	if(Temp>32768) Temp-=65536;
	Temp=(36.53+Temp/340)*10;
	return Temp;
}

void MPU6050_setPitchCorrectorRate(float pitchCorrectorRate)
{
	MPU6050_PitchCorrector = pitchCorrectorRate;
}

void MPU6050_setRollCorrectorRate(float rollCorrectorRate)
{
	MPU6050_RollCorrectorRate = rollCorrectorRate;
}

void MPU6050_setYawCorrectorRate(float yawCorrectorRate)
{
	MPU6050_YawCorrectorRate = yawCorrectorRate;
}

void MPU6050_updateAngleCorrector(void)
{
	MPU6050_PitchCorrector += MPU6050_PitchCorrectorRate;
	MPU6050_RollCorrector += MPU6050_RollCorrectorRate;
	MPU6050_YawCorrector += MPU6050_YawCorrectorRate;
}

void MPU6050_getDriftingRate(float pitch, float roll, float yaw, float* pitchRate, float* rollRate, float* yawRate)
{
	static long t = 0;

	*pitchRate = pitch/t;
	*rollRate = roll/t;
	*yawRate = yaw/t;

//	printf("%.2f %f %li\n\r", yaw, *yawRate, t);

	t++;
}

void MPU6050_getDecodedGyro(float* gyro)
{
	for(int i = 0; i < 0; i++)
	{
		gyro[i] = (float)MPU6050_gyroRAW[i] / MPU6050_gyroSensitivity;
	}
}

void MPU6050_getDecodedAcc(float* acc)
{
	for(int i = 0; i < 0; i++)
	{
		acc[i] = (float)MPU6050_accelRAW[i] / MPU6050_accSensitivity;
	}
}

uint8_t MPU6050_getZeroMotionDetectionThreshold()
{
	IICreadBytes(devAddr, MPU6050_RA_ZRMOT_THR, 1, buffer);
	return buffer[0];
}
void MPU6050_setZeroMotionDetectionThreshold(unsigned char threshold)
{
	IICwriteByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}
uint8_t MPU6050_getZeroMotionDetectionDuration()
{
	IICreadBytes(devAddr, MPU6050_RA_ZRMOT_DUR, 1, buffer);
	return buffer[0];
}
void MPU6050_setZeroMotionDetectionDuration(unsigned char duration)
{
	IICwriteByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}
uint8_t MPU6050_getMotionStatus()
{
	IICreadBytes(devAddr, MPU6050_RA_MOT_DETECT_STATUS, 1, buffer);
	return buffer[0];
}

int16_t MPU6050_getXGyroOffset() {
    IICreadBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050_setXGyroOffset(int16_t offset) {
	u8 buffer[] = {offset>>8 & 0xff, offset & 0xff};
    IICwriteBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
}

// YG_OFFS_USR* register

int16_t MPU6050_getYGyroOffset() {
    IICreadBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050_setYGyroOffset(int16_t offset) {
	u8 buffer[] = {offset>>8 & 0xff, offset & 0xff};
    IICwriteBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
}

// ZG_OFFS_USR* register

int16_t MPU6050_getZGyroOffset() {
    IICreadBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050_setZGyroOffset(int16_t offset) {
	u8 buffer[] = {offset>>8 & 0xff, offset & 0xff};
    IICwriteBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
}

void MPU6050_getAllGyroOffset(int16_t* gyroOffset)
{
	gyroOffset[0] = MPU6050_getXGyroOffset();
	gyroOffset[1] = MPU6050_getYGyroOffset();
	gyroOffset[2] = MPU6050_getZGyroOffset();
}

void MPU6050_setAllGyroOffset(int16_t* gyroOffset)
{
	MPU6050_setXGyroOffset(gyroOffset[0]);
	MPU6050_setYGyroOffset(gyroOffset[1]);
	MPU6050_setZGyroOffset(gyroOffset[2]);
}

void MPU6050_GyroCalibration(int loop)
{
	short gyroOffset[3];
	float gyroOffsetF[3];
	float kp, ki, kd;
	float P[3]={0};
	float I[3]={0};
	float D[3]={0};
	float error[3];
	float lastError[3];
	float maxI = 1.0;
	int indexMonitor = 2;

	kp = 0.1;
	ki = 2;
	kd = 0.0001;
	/************** FIRST STEP CALIBRATION **********/
	MPU6050_getAllGyroOffset(gyroOffset);
	for(int i =0; i < 3; i++) gyroOffsetF[i] = gyroOffset[i];
	for(int i = 0;i < loop; i++)
	{
		while(MPU6050_readDMP());
		for(int j = 0; j < 3; j++)
		{
			error[j] = -MPU6050_gyroRAW[j];
			P[j] = kp * error[j];
			I[j] += ki * error[j] * 0.001;
			D[j] = kd * (error[j] - lastError[j])/0.001;

			if(I[j]> maxI) I[j] = maxI;
			else if(I[j] < -maxI) I[j] = -maxI;

			if(fabs(error[j]) < 2) I[j] = 0;

			float adjustValue = P[j] + I[j] + D[j];
			if(j == 2) gyroOffsetF[j] += adjustValue;
			else gyroOffsetF[j] -= adjustValue;
			gyroOffset[j] = gyroOffsetF[j];

			lastError[j] = error[j];
		}
		MPU6050_setAllGyroOffset(gyroOffset);

		printf("P: %.1f I: %.1f D: %.1f e: %.1f oF: %.1f oS: %d\n", P[indexMonitor],
													I[indexMonitor],
													D[indexMonitor],
													error[indexMonitor],
													gyroOffsetF[indexMonitor],
													gyroOffset[indexMonitor]);
		delay_ms(1);
	}

	mpu_reset_fifo();
	mpu_reset_dmp();
}

int MPU6050_GyroContinuosCalibration(int observationTime, float threshold) //only Yaw
{
	static u8 previoustMotionStatus;
	u8 motionStatus;
	static long startTime;
	static float startYaw;
	static float gyroDriftRate = 0;
	int currentOffset;
	static int offset;
	int timeSpent;
	motionStatus = MPU6050_getMotionStatus(); // 0: moving, 1 : no movement

	if(motionStatus && !previoustMotionStatus)
	{
		startTime = getTick();
		startYaw = MPU6050_YawUncorected;
		gyroDriftRate = 0;
	}
	timeSpent = (getTick() - startTime);
	if(motionStatus && timeSpent > observationTime)
	{
		float deltaAngle = MPU6050_YawUncorected - startYaw;
		if(deltaAngle > 180) deltaAngle -= 360;
		if(deltaAngle < -180) deltaAngle += 360;

		gyroDriftRate = deltaAngle/timeSpent*1000;
		currentOffset = MPU6050_getZGyroOffset();
		offset = currentOffset - round((float)gyroDriftRate*32.7675);
		MPU6050_setZGyroOffset(offset);
		startTime = getTick();
		startYaw = MPU6050_YawUncorected;
	}

//	printf("yaw: %.2f ms:%d startTime:%li timeSpent:%d dr:%.4f offset:%d\n", MPU6050_Yaw, motionStatus, startTime, timeSpent, gyroDriftRate, offset);
	previoustMotionStatus = motionStatus;

	if(gyroDriftRate != 0 && fabs(gyroDriftRate) < threshold) return 1;
	return 0;
}

int MPU6050_getFifoCount()
{
	uint8_t data[2];
//	uint16_t fifoCount;
	IICreadBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, data);

	return data[0]<<8 | data[1];
}
//------------------End of File----------------------------
