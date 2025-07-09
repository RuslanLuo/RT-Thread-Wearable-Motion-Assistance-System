#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <math.h>
#include "icm20608.h"
#include <drv_lcd.h>
#include "soft_i2c.h"
#include "max30102.h"
#include "algorithm.h"

#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>

#define SAMPLES_NUM BUFFER_SIZE  // 一次需要采集多少点数据

#define RTC_NAME       "rtc"
#define PIN_KEY0        GET_PIN(C, 0)

static uint32_t irBuffer[SAMPLES_NUM];
static uint32_t redBuffer[SAMPLES_NUM];
static int sampleIndex = 0;  // 当前写入数组的下标
short lastStep = 0;
short steps = 0;
short totalCaloriess = 0;
static int32_t spo2 = 0, heartRate = 0;
static int32_t spo21 = 0, heartRate1 = 0;
static int8_t spo2_valid = 0, hr_valid = 0;
static uint8_t current_page = 0;

time_t now;
rt_device_t rtc_device = RT_NULL;

//独立线程处理心率和血氧数据
static rt_thread_t hr_thread = RT_NULL;  // 创建线程句柄
static rt_thread_t tid = RT_NULL;

typedef struct
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
} axis_t;

typedef struct
{
    rt_int32_t x;
    rt_int32_t y;
    rt_int32_t z;
} axis_32_t;

icm20608_device_t dev;
rt_err_t result;
short dynamicPrecision = 4000;
short step = 0;
short sensitivity = 3100; // 灵敏度参数
axis_t newMax;
axis_t newMin;
axis_t oldMax;
axis_t oldMin;
axis_t newSample;
axis_t oldSample;

static short totalCalories = 0;

void heart_rate_and_spo2_thread_entry(void* parameter)
{
    while (1)
    {
        // 读取MAX30102数据
        uint32_t sensorData[2];
        int8_t readStatus = maxim_max30102_read_fifo(&sensorData[0], &sensorData[1]);  // 正确传递参数

        // 检查是否成功读取数据
        if (readStatus == 1)
        {
            irBuffer[sampleIndex] = sensorData[0];
            redBuffer[sampleIndex] = sensorData[1];
            sampleIndex++;

            //LOG_D("Current sample index: %d", sampleIndex);

            // 判断是否达到批量计算的采样数（例如500点）
            if (sampleIndex >= SAMPLES_NUM)
            {
                /*                LOG_D("Processing heart rate and SpO2...");*/
                // 调用算法计算心率和血氧
                maxim_heart_rate_and_oxygen_saturation(irBuffer,    // IR 数据数组
                        SAMPLES_NUM, // IR 数据长度
                        redBuffer,   // RED 数据数组
                        &spo2,       // 算出血氧
                        &spo2_valid, // 血氧是否有效
                        &heartRate,  // 算出心率
                        &hr_valid    // 心率是否有效
                        );

                // 重新开始采样
                sampleIndex = 0;
            }
        }
        else
        {
            LOG_E("Error reading data from MAX30102");
        }

        rt_thread_mdelay(20);  // 延时1ms
    }
}

//传感器初始化
void axisInit(void)
{
    dev = RT_NULL;
    dev = icm20608_init("i2c2");
    if (dev == RT_NULL)
    {
        LOG_E("icm20608 initializes failure");
    }
    else
    {
        LOG_D("icm20608 initializes success");
    }

    result = icm20608_calib_level(dev, 10);
    if (result == RT_EOK)
    {
        LOG_D("icm20608 calibrates success");
        LOG_D("accel_offset: X%6d  Y%6d  Z%6d", dev->accel_offset.x, dev->accel_offset.y, dev->accel_offset.z);
        LOG_D("gyro_offset : X%6d  Y%6d  Z%6d", dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);
    }
    else
    {
        LOG_E("The sensor calibrates failure");
        icm20608_deinit(dev);
    }
}

//获取加速度数据（未滤波）
axis_t axisGetData(void)
{
    axis_t data;
    icm20608_get_accel(dev, &data.x, &data.y, &data.z);
    return data;
}

//滤波获取加速度样本
axis_t axisGetSample(void)
{
    axis_t filter[5];
    axis_32_t filterSum = { 0, 0, 0 };
    axis_t sample;
    for (int i = 1; i < 5; ++i)
    {
        filter[i] = axisGetData();
        filterSum.x += filter[i].x;
        filterSum.y += filter[i].y;
        filterSum.z += filter[i].z;
    }
    sample.x = filterSum.x / 4;
    sample.y = filterSum.y / 4;
    sample.z = filterSum.z / 4;
    return sample;
}

//计算绝对值
short ABS(short a)
{
    return (a >= 0) ? a : -a;
}

//更新获取新的加速度值
void axisSlidUpdate(void)
{
    axis_t curSample = axisGetSample();
    if (ABS(curSample.x - newSample.x) > dynamicPrecision)
    {
        oldSample.x = newSample.x;
        newSample.x = curSample.x;
    }
    if (ABS(curSample.y - newSample.y) > dynamicPrecision)
    {
        oldSample.y = newSample.y;
        newSample.y = curSample.y;
    }
    if (ABS(curSample.z - newSample.z) > dynamicPrecision)
    {
        oldSample.z = newSample.z;
        newSample.z = curSample.z;
    }
}

short MAX(short a, short b)
{
    return (a >= b) ? a : b;
}

short MIN(short a, short b)
{
    return (a <= b) ? a : b;
}

//更新峰值
void axisPeakUpdate(void)
{
    short minValue = -32768;
    short maxValue = 32767;
    newMax.x = minValue;
    newMax.y = minValue;
    newMax.z = minValue;
    newMin.x = maxValue;
    newMin.y = maxValue;
    newMin.z = maxValue;
    for (int i = 0; i < 50; ++i)
    {
        axisSlidUpdate();
        newMax.x = MAX(newMax.x, newSample.x);
        newMax.y = MAX(newMax.y, newSample.y);
        newMax.z = MAX(newMax.z, newSample.z);
        newMin.x = MIN(newMin.x, newSample.x);
        newMin.y = MIN(newMin.y, newSample.y);
        newMin.z = MIN(newMin.z, newSample.z);
    }
    oldMax = newMax;
    oldMin = newMin;
}

//检测步数
void axisDetectStep(void)
{
    axis_t threshold;
    axisPeakUpdate();

    threshold.x = (oldMax.x + oldMin.x) / 2;
    threshold.y = (oldMax.y + oldMin.y) / 2;
    threshold.z = (oldMax.z + oldMin.z) / 2;

    if (ABS(oldSample.x - newSample.x) > sensitivity && oldSample.x > threshold.x && newSample.x < threshold.x)
    {
        step++;
    }
    else if (ABS(oldSample.y - newSample.y) > sensitivity && oldSample.y > threshold.y && newSample.y < threshold.y)
    {
        step++;
    }
    else if (ABS(oldSample.z - newSample.z) > sensitivity && oldSample.z > threshold.z && newSample.z < threshold.z)
    {
        step++;
    }
}

//估算热量
static short estimateCalories(axis_t accelData)
{
    short sum = (short) (ABS(accelData.x) + ABS(accelData.y) + ABS(accelData.z));
    short retVal = (short) (sum / 365);

    return retVal;
}

static void display_thread_entry(void *param)
{
    struct tm *time_info;
    char time_str[20];

    while (1)
    {
        /* 获取当前时间 */
        now = time(RT_NULL);
        time_info = localtime(&now);
        strftime(time_str, sizeof(time_str), "%H:%M:%S", time_info);
        axisDetectStep();
        if (step > lastStep)
        {
            //获取当前加速度数据
            axis_t accelData = axisGetData();

            //估算当前时刻热量消耗并累加
            short currentCalories = estimateCalories(accelData);
            totalCalories += currentCalories;

            lastStep = step;
        }

        steps = step + step;

        totalCaloriess = totalCalories + totalCalories;

        /* 按键检测与页面切换 */
        if (rt_pin_read(PIN_KEY0) == PIN_LOW)
        {
            rt_thread_mdelay(10); // 消抖
            if (rt_pin_read(PIN_KEY0) == PIN_LOW)
            {
                current_page = !current_page; // 切换页面
                lcd_clear(WHITE); // 清屏函数（需根据实际LCD驱动实现）
                while (rt_pin_read(PIN_KEY0) == PIN_LOW)
                    ; // 等待按键释放
            }
        }

        /* 根据当前页更新显示 */
        if (current_page == 0)
        {
            // 页1：时间、步数、热量
            lcd_show_string(10, 20, 24, "Time:", BLACK, WHITE);
            lcd_show_string(80, 20, 24, time_str, BLACK, WHITE);

            lcd_show_string(10, 60, 24, "Steps: %4d", steps, BLACK, WHITE);

            lcd_show_string(10, 100, 24, "Calories: %4d", totalCalories, BLACK, WHITE);
        }
        if (current_page == 1)
        {
            if (hr_valid == 0 || spo2 < 90)
            {
                lcd_show_string(10, 20, 24, "Time:", BLACK, WHITE);
                lcd_show_string(80, 20, 24, time_str, BLACK, WHITE);
                lcd_show_string(10, 60, 24, "HeartRate: %4d", 9999, BLACK, WHITE);

                lcd_show_string(10, 100, 24, "SpO2: %4d", 9999, BLACK, WHITE);
            }
            else
            {
                // 页2：时间、心率、血氧
                lcd_show_string(10, 20, 24, "Time:", BLACK, WHITE);
                lcd_show_string(80, 20, 24, time_str, BLACK, WHITE);

                lcd_show_string(10, 60, 24, "HeartRate: %4d", heartRate, BLACK, WHITE);

                lcd_show_string(10, 100, 24, "SpO2: %4d", spo2, BLACK, WHITE);
            }
        }
        rt_thread_mdelay(1);
    }

}

int main(void)
{
    HAL_Init();  // 初始化 HAL 库
    SystemClock_Config();  // 配置系统时钟
    GPIO_Config();  // 配置 PB14 和 PB15 引脚
    IIC_Init();  // 软件 I2C 初始化
    /* 寻找RTC设备 */
    rtc_device = rt_device_find(RTC_NAME);
    if (!rtc_device)
    {
        rt_kprintf("find %s failed!", RTC_NAME);
        return RT_ERROR;
    }
    /* 初始化RTC设备 */
    if (rt_device_open(rtc_device, 0) != RT_EOK)
    {
        rt_kprintf("open %s failed!", RTC_NAME);
        return RT_ERROR;
    }
    lcd_clear(WHITE);
//初始化 MAX30102
    maxim_max30102_init();
    axisInit();
    rt_pin_mode(PIN_KEY0, PIN_MODE_INPUT_PULLUP);

//创建心率和血氧数据处理线程
    hr_thread = rt_thread_create("hr_thread", heart_rate_and_spo2_thread_entry, RT_NULL, 2048, 10, 10); //线程的堆栈大小：2048   线程的优先级：10   线程的时间片：10ms
    if (hr_thread != RT_NULL)
    {
        LOG_D("Heart rate and SpO2 thread started");
        rt_thread_startup(hr_thread);  // 启动线程
    }
    else
    {
        LOG_E("Failed to create heart rate thread!");
    }

    tid = rt_thread_create("display", display_thread_entry,
    RT_NULL, 2048, 20, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    return 0;
}
