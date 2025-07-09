#ifndef APPLICATIONS_SOFT_I2C_SOFT_I2C_H_
#define APPLICATIONS_SOFT_I2C_SOFT_I2C_H_

#include <stm32f4xx.h>

//IO方向设置
#define SDA_IN()  {GPIO_InitTypeDef GPIO_InitStruct = {0}; \
                   GPIO_InitStruct.Pin = IIC_SDA; \
                   GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
                   GPIO_InitStruct.Pull = GPIO_NOPULL; \
                   HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);} // PB14 输入

#define SDA_OUT() {GPIO_InitTypeDef GPIO_InitStruct = {0}; \
                   GPIO_InitStruct.Pin = IIC_SDA; \
                   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
                   GPIO_InitStruct.Pull = GPIO_NOPULL; \
                   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; \
                   HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);} // PB14 输出


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);               //发送IIC开始信号
void IIC_Stop(void);                //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);         //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void);              //IIC等待ACK信号
void IIC_Ack(void);                 //IIC发送ACK信号
void IIC_NAck(void);                //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);


#endif /* APPLICATIONS_SOFT_I2C_SOFT_I2C_H_ */
