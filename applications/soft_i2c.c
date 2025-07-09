#include "soft_i2c.h"
#include <rtdbg.h>
#include "stm32f4xx_hal.h"

#define IIC_SDA  GPIO_PIN_14  // PB14 是 SDA 引脚
#define IIC_SCL  GPIO_PIN_15  // PB15 是 SCL 引脚
#define I2C_PORT  GPIOB       // 使用 GPIOB 端口

#define READ_SDA (GPIOB->IDR & GPIO_PIN_14)  // 读取PB14的状态


// 设置 I2C 时钟频率为 100 kHz
#define I2C_DELAY 5  // 延时5微秒，适应100kHz频率

//新增微秒级延时函数
void delay_us(uint32_t udelay)
{
    uint32_t startval, tickn, delays, wait;

    startval = SysTick->VAL;
    tickn = HAL_GetTick();

    // 根据系统时钟频率（16 MHz）来调整延时计算
    delays = udelay * (SystemCoreClock / 1000000U); // 系统时钟频率除以 1000000 得到每微秒的时钟周期数

    if (delays > startval)
    {
        while (HAL_GetTick() == tickn)
        {
            // 等待系统计时器更新
        }
        wait = SystemCoreClock + startval - delays;  // 计算需要等待的时钟值
        while (wait < SysTick->VAL)
        {
            // 等待直到 SysTick 计时器值达到所需延迟
        }
    }
    else
    {
        wait = startval - delays;
        while (wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
            // 等待直到 SysTick 计时器值达到所需延迟
        }
    }
}




//初始化IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 启用 GPIOB 时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();  // 使用 HAL 函数替代原来的函数

    // 配置 GPIOB 引脚（SDA 和 SCL）
    GPIO_InitStructure.Pin = IIC_SDA | IIC_SCL;  // 设置 SDA 和 SCL 引脚
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;   // 开漏输出
    GPIO_InitStructure.Pull = GPIO_PULLUP;   // 上拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;  // 设置为中速，适合 100 kHz 模式
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStructure);  // 使用 HAL 库进行初始化

    // 设置初始状态
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_SET);  // SDA 高电平
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);  // SCL 高电平
}
//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT();     // SDA 设置为输出
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_RESET); // START: when CLK is high, DATA change from high to low
    delay_us(4);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET); // 钳住 I2C 总线，准备发送或接收数据
}

//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT(); // SDA 设置为输出
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_RESET); // STOP: when CLK is high, DATA change from low to high
    delay_us(4);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_SET); // 发送 I2C 总线结束信号
    delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); // SDA 设置为输入
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
    delay_us(1);

    while (READ_SDA) // 低电平为应答
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET); // 时钟输出0
    return 0;
}

//产生ACK应答
void IIC_Ack(void)
{
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
    SDA_OUT();
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
}

//不产生ACK应答
void IIC_NAck(void)
{
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
    SDA_OUT();
    HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET); // 拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        HAL_GPIO_WritePin(I2C_PORT, IIC_SDA, (txd & 0x80) >> 7);
        txd <<= 1;
        delay_us(2);
        HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
        delay_us(2);
        HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
        delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); // SDA 设置为输入
    for (i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(I2C_PORT, IIC_SCL, GPIO_PIN_SET);
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck(); // 发送 nACK
    else
        IIC_Ack(); // 发送 ACK
    return receive;
}
