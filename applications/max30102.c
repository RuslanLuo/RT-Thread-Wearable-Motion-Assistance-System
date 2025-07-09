#include "max30102.h"
#include <rtthread.h>
#include <rtdbg.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>  // 使用 HAL 库头文件


void max30102_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 启用 GPIOB 时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();  // 替换为 HAL 函数

    // 配置 GPIOB 引脚（INT 引脚）
    GPIO_InitStructure.Pin = GPIO_PIN_5; // 使用正确的引脚
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT; // 设置为输入模式
    GPIO_InitStructure.Pull = GPIO_NOPULL; // 无拉电阻（根据需要选择合适的拉电阻）
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM; // 设置速度为中
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  // 使用 HAL GPIO 初始化函数
}

int8_t maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
    uint8_t Ack1,Ack2,Ack3;

    IIC_Start();

    IIC_Send_Byte(I2C_WRITE_ADDR);//发送设备写地址
    Ack1 = IIC_Wait_Ack();

    IIC_Send_Byte(uch_addr);    //发送寄存器地址
    Ack2 = IIC_Wait_Ack();

    IIC_Send_Byte(uch_data);    //发送数据
    Ack3 = IIC_Wait_Ack();

    IIC_Stop();//产生一个停止条件

    if(Ack1 || Ack2 || Ack3)//如果
        return 0; //发送失败
    else
        return 1; //发送成功

}

int8_t maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
    uint8_t Ack1,Ack2,Ack3;;

    IIC_Start();
    IIC_Send_Byte(I2C_WRITE_ADDR);  //发送设备写地址
    Ack1 = IIC_Wait_Ack();

    IIC_Send_Byte(uch_addr);    //发送寄存器地址
    Ack2 = IIC_Wait_Ack();

      IIC_Start();
    IIC_Send_Byte(I2C_READ_ADDR);//发送设备读地址
    Ack3 = IIC_Wait_Ack();

    *puch_data = IIC_Read_Byte(0);//IIC读取字节

    IIC_Stop();

    if(Ack1 || Ack2 || Ack3)//如果
        return 0; //发送失败
    else
        return 1; //发送成功
}

int8_t maxim_max30102_init(void)
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    max30102_GPIO_Init();//初始化INT引脚
    if(!maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
        return 0;
    if(!maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00))
        return 0;
    if(!maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
        return 0;
    if(!maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
        return 0;
    if(!maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
        return 0;
    if(!maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=-1, fifo almost full = 17
        return 0;
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return 0;
    if(!maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
        return 0;

    if(!maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
        return 0;
    if(!maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
        return 0;
    if(!maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
        return 0;
    return 1;  //返回成功
}

int8_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    uint32_t un_temp;
    uint8_t uch_temp;
    char ach_i2c_data[6];
    uint8_t Ack1, Ack2, Ack3;

    *pun_red_led = 0;
    *pun_ir_led = 0;

    // Read and clear status register
    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);

    ach_i2c_data[0] = REG_FIFO_DATA;

    IIC_Start();
    IIC_Send_Byte(I2C_WRITE_ADDR);  // Send device write address
    Ack1 = IIC_Wait_Ack();

    IIC_Send_Byte(ach_i2c_data[0]); // Send register address
    Ack2 = IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(I2C_READ_ADDR);   // Send device read address
    Ack3 = IIC_Wait_Ack();

    // Read red LED data (IR data will be assigned to red LED)
    un_temp = IIC_Read_Byte(1);  // Read
    un_temp <<= 16;
    *pun_ir_led += un_temp;  // Assign to IR LED instead of red LED

    un_temp = IIC_Read_Byte(1);  // Read
    un_temp <<= 8;
    *pun_ir_led += un_temp;  // Assign to IR LED instead of red LED

    un_temp = IIC_Read_Byte(1);  // Read
    *pun_ir_led += un_temp;  // Assign to IR LED instead of red LED

    // Read IR LED data (Red data will be assigned to IR LED)
    un_temp = IIC_Read_Byte(1);  // Read
    un_temp <<= 16;
    *pun_red_led += un_temp;  // Assign to red LED instead of IR LED

    un_temp = IIC_Read_Byte(1);  // Read
    un_temp <<= 8;
    *pun_red_led += un_temp;  // Assign to red LED instead of IR LED

    un_temp = IIC_Read_Byte(1);  // Read
    *pun_red_led += un_temp;  // Assign to red LED instead of IR LED

    // Mask MSB [23:18]
    *pun_red_led &= 0x03FFFF;
    *pun_ir_led &= 0x03FFFF;

    IIC_Stop();  // Stop condition

    if (Ack1 || Ack2 || Ack3) // If any acknowledge fails
        return 0;  // Fail
    else
        return 1;  // Success
}


int8_t maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40))
        return 0;
    else
        return 1; //返回成功
}
