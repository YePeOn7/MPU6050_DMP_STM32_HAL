#include "IOI2C.h"
#include <stdio.h>
#include "main.h"

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
	if((int)HAL_I2C_Mem_Read(&i2c_handle, addr << 1, reg, 1, buf, len, I2C_TIMEOUT) == 0)
		return 0;
	else{
		I2C2_ClearBusyFlagErratum(&i2c_handle);
		return -1;
	}
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

void I2C2_ClearBusyFlagErratum(I2C_HandleTypeDef *instance)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	int timeout = 100;
	int timeout_cnt = 0;

	// 1. Clear PE bit.
	instance->Instance->CR1 &= ~(0x0001);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = I2C2_SCL_PIN;
	HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = I2C2_SDA_PIN;
	HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

	// 3. Check SCL and SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
	{
		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
	{
		//Move clock to release I2C
		HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
		asm("nop");
		HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_RESET);

	//  5. Check SDA Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
	{
		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);

	//  7. Check SCL Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
	{
		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SCL_PORT, I2C2_SCL_PIN))
	{
		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN))
	{
		timeout_cnt++;
		if (timeout_cnt > timeout) return;
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;

	GPIO_InitStruct.Pin = I2C2_SCL_PIN;
	HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = I2C2_SDA_PIN;
	HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	instance->Instance->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	instance->Instance->CR1 &= ~0x8000;

	asm("nop");

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	instance->Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init(instance);
}
