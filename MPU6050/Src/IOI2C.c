#include "IOI2C.h"
#include <stdio.h>

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
  
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    return (int)HAL_I2C_Mem_Write(&i2c_handle, addr << 1, reg, 1, data, len, I2C_TIMEOUT);
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	return (int)HAL_I2C_Mem_Read(&i2c_handle, addr << 1, reg, 1, buf, len, I2C_TIMEOUT);
//    return 0;
}

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char data;
	HAL_I2C_Mem_Read(&i2c_handle, I2C_Addr, addr, 1, &data, 1, I2C_TIMEOUT);

	return data;
}

void IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
	HAL_I2C_Mem_Read(&i2c_handle, dev, reg, 1, data, length, I2C_TIMEOUT);
}


u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
	HAL_I2C_Mem_Write(&i2c_handle, dev, reg, 1, data, length, I2C_TIMEOUT);
    return 1; //status == 0;
}

u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
	return IICwriteBytes(dev, reg, 1, &data);
}

u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF >> (8-length)) << (bitStart-(length-1));
        data &= (mask >> (bitStart-(length-1)));
        data <<= (bitStart-(length-1));
        b &= ~mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

