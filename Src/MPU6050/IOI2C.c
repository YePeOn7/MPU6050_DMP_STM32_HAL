#include "IOI2C.h"

I2C_HandleTypeDef i2c_handle;
GPIO_TypeDef * GPIO_SCL;
GPIO_TypeDef * GPIO_SDA;
uint32_t GPIO_PIN_SCL;
uint32_t GPIO_PIN_SDA;

void IIC_Init(I2C_HandleTypeDef i2cHandle)
{			
	i2c_handle = i2cHandle;
}

void IIC_InitLockupRecover(GPIO_TypeDef * _GPIO_SLC, uint32_t _GPIO_PIN_SCL, GPIO_TypeDef * _GPIO_SDA, uint32_t _GPIO_PIN_SDA)
{
	GPIO_SDA = _GPIO_SDA;
	GPIO_SCL = _GPIO_SLC;
	GPIO_PIN_SCL = _GPIO_PIN_SCL;
	GPIO_PIN_SDA = _GPIO_PIN_SDA;
}

void IIC_LockupRecover()
{
	if(!HAL_GPIO_ReadPin(GPIO_SDA, GPIO_PIN_SDA))
	{
		// Lockup Recovery process
		for(int i = 0; i < IIC_GPIO_NUMBER; i++)
		{
			if((1 << i) & GPIO_PIN_SCL)
			{
				// put the pin into output mode
				GPIO_SCL-> MODER &= ~(0b11 << 2*i);
				GPIO_SCL-> MODER |= (0b1 << 2*i);

				// inject 9 pulses to SCL
				for(int j = 0; j < 9; j++)
				{
					HAL_GPIO_WritePin(GPIO_SCL, GPIO_PIN_SCL, RESET);
					delay_ms(1);
					HAL_GPIO_WritePin(GPIO_SCL, GPIO_PIN_SCL, SET);
					delay_ms(1);
				}

				// put the pin back into AF mode
				GPIO_SCL-> MODER &= ~(0b11 << 2*i);
				GPIO_SCL-> MODER |= (0b10 << 2*i);
			}
		}


	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/


//int IIC_Start(void)
//{
//	SDA_OUT();      //sda�����
//	IIC_SDA=1;
//	if(!READ_SDA)return 0;
//	IIC_SCL=1;
//	delay_us(1);
// 	IIC_SDA=0;      //START:when CLK is high,DATA change form high to low
//	if(READ_SDA)return 0;
//	delay_us(1);
//	IIC_SCL=0;      //ǯסI2C���ߣ�׼�����ͻ��������
//	return 1;
//}
//
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void IIC_Stop(void)
//*��������:	    //����IICֹͣ�ź�
//*******************************************************************************/
//void IIC_Stop(void)
//{
//	SDA_OUT();     //sda�����
//	IIC_SCL=0;
//	IIC_SDA=0;     //STOP:when CLK is high DATA change form low to high
// 	delay_us(1);
//	IIC_SCL=1;
//	IIC_SDA=1;     //����I2C���߽����ź�
//	delay_us(1);
//}
//
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		u8 IIC_Wait_Ack(void)
//*��������:	    �ȴ�Ӧ���źŵ���
////����ֵ��1������Ӧ��ʧ��
////        0������Ӧ��ɹ�
//*******************************************************************************/
//int IIC_Wait_Ack(void)
//{
//	u8 ucErrTime=0;
//	SDA_IN();      //SDA����Ϊ����
//	IIC_SDA=1;
//	delay_us(1);
//	IIC_SCL=1;
//	delay_us(1);
//	while(READ_SDA)
//	{
//		ucErrTime++;
//		if(ucErrTime>50)
//		{
//			IIC_Stop();
//			return 0;
//		}
//	  delay_us(1);
//	}
//	IIC_SCL=0;   //ʱ�����0
//	return 1;
//}
//
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void IIC_Ack(void)
//*��������:	    ����ACKӦ��
//*******************************************************************************/
//void IIC_Ack(void)
//{
//	IIC_SCL=0;
//	SDA_OUT();
//	IIC_SDA=0;
//	delay_us(1);
//	IIC_SCL=1;
//	delay_us(1);
//	IIC_SCL=0;
//}
//
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void IIC_NAck(void)
//*��������:	    ����NACKӦ��
//*******************************************************************************/
//void IIC_NAck(void)
//{
//	IIC_SCL=0;
//	SDA_OUT();
//	IIC_SDA=1;
//	delay_us(1);
//	IIC_SCL=1;
//	delay_us(1);
//	IIC_SCL=0;
//}
///**************************ʵ�ֺ���********************************************
//*����ԭ��:		void IIC_Send_Byte(u8 txd)
//*��������:	    IIC����һ���ֽ�
//*******************************************************************************/
//void IIC_Send_Byte(u8 txd)
//{
//    u8 t;
//	SDA_OUT();
//    IIC_SCL=0;   //����ʱ�ӿ�ʼ���ݴ���
//    for(t=0;t<8;t++)
//    {
//        IIC_SDA=(txd&0x80)>>7;
//        txd<<=1;
//		delay_us(1);
//		IIC_SCL=1;
//		delay_us(1);
//		IIC_SCL=0;
//		delay_us(1);
//    }
//}


  
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    return (int)HAL_I2C_Mem_Write(&i2c_handle, addr << 1, reg, 1, data, len, I2C_TIMEOUT);
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	HAL_I2C_Mem_Read(&i2c_handle, addr << 1, reg, 1, buf, len, I2C_TIMEOUT);
    return 0;
}


///**************************ʵ�ֺ���********************************************
//*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
//*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
//*******************************************************************************/
//u8 IIC_Read_Byte(unsigned char ack)
//{
//	unsigned char i,receive=0;
//	SDA_IN();//SDA����Ϊ����
//    for(i=0;i<8;i++ )
//	{
//        IIC_SCL=0;
//        delay_us(2);
//		IIC_SCL=1;
//        receive<<=1;
//        if(READ_SDA)receive++;
//		delay_us(2);
//    }
//    if (ack)
//        IIC_Ack(); //����ACK
//    else
//        IIC_NAck();//����nACK
//    return receive;
//}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
//	unsigned char res=0;
//
//	IIC_Start();
//	IIC_Send_Byte(I2C_Addr);	   //����д����
//	res++;
//	IIC_Wait_Ack();
//	IIC_Send_Byte(addr); res++;  //���͵�ַ
//	IIC_Wait_Ack();
//	//IIC_Stop();//����һ��ֹͣ����
//	IIC_Start();
//	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ
//	IIC_Wait_Ack();
//	res=IIC_Read_Byte(0);
//    IIC_Stop();//����һ��ֹͣ����
//
	unsigned char data;
	HAL_I2C_Mem_Read(&i2c_handle, I2C_Addr, addr, 1, &data, 1, I2C_TIMEOUT);

	return data;
}

void IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
//    u8 count = 0;
//
//	IIC_Start();
//	IIC_Send_Byte(dev);
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);  ַ
//    IIC_Wait_Ack();
//	IIC_Start();
//	IIC_Send_Byte(dev+1);
//	IIC_Wait_Ack();
//
//    for(count=0;count<length;count++){
//
//		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
//		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
//	}
//    IIC_Stop();//����һ��ֹͣ����
//    return count;
	HAL_I2C_Mem_Read(&i2c_handle, dev, reg, 1, data, length, I2C_TIMEOUT);
}


u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
// 	u8 count = 0;
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //����д����
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //���͵�ַ
//    IIC_Wait_Ack();
//	for(count=0;count<length;count++){
//		IIC_Send_Byte(data[count]);
//		IIC_Wait_Ack();
//	 }
//	IIC_Stop();//����һ��ֹͣ����

	HAL_I2C_Mem_Write(&i2c_handle, dev, reg, 1, data, length, I2C_TIMEOUT);
    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
