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
short gyro[3], accel[3], sensors;
float MPU6050_Pitch, MPU6050_Roll, MPU6050_Yaw;

float MPU6050_PitchCorrectorRate = 0;
float MPU6050_RollCorrectorRate = 0;
float MPU6050_YawCorrectorRate = 0;
float MPU6050_PitchCorrector = 0;
float MPU6050_RollCorrector = 0;
float MPU6050_YawCorrector = 0;

float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

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
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
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

	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //����ʱ��
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);//������������� +-2000��ÿ��
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
	MPU6050_setSleepEnabled(0); //���빤��״̬
	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	MPU6050_setI2CBypassEnabled(0);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
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

	HAL_Delay(100);
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
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
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
}
/**************************************************************************
The output of this function will return to MPU6050_Pitch, MPU6050_Roll, MPU6050_Yaw
**************************************************************************/
void MPU6050_readDMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch, roll, yaw;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
	if (sensors & INV_WXYZ_QUAT )
	{    
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

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
}

void MPU6050_readDMPAll(float* Pitch, float* Roll, float* Yaw)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch, roll, yaw;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

		 pitch += MPU6050_PitchCorrector;
		 pitch = fmod(pitch, 360);
		 if(pitch > 180) pitch -= 360;

		 roll += MPU6050_RollCorrector;
		 roll = fmod(roll, 360);
		 if(roll > 180) roll -= 360;

		 yaw += MPU6050_YawCorrector;
		 yaw = fmod(yaw, 360);
		 if(yaw > 180) yaw -= 360;

		 *Pitch = pitch;
		 *Roll = roll;
		 *Yaw = yaw;
	}
}

float MPU6050_readDMPPitch()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float pitch;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;

		 pitch += MPU6050_PitchCorrector;
		 pitch = fmod(pitch, 360);
		 if(pitch > 180) pitch -= 360;
	}

	return pitch;
}
float MPU6050_readDMPRoll()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float roll;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;

		 roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

		 roll += MPU6050_RollCorrector;
		 roll = fmod(roll, 360);
		 if(roll > 180) roll -= 360;
	}

	return roll;
}

float MPU6050_readDMPYaw()
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	float yaw;

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	if (sensors & INV_WXYZ_QUAT )
	{
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;

		 yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;

		 yaw += MPU6050_YawCorrector;
		 yaw = fmod(yaw, 360);
		 if(yaw > 180) yaw -= 360;
	}

	return yaw;
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

	printf("%.2f %f %li\n\r", yaw, *yawRate, t);

	t++;
}
//------------------End of File----------------------------
