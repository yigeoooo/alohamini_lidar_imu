#include "lidar_ms200.h"

#include <stdlib.h>

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"


#include "uart1.h"
#include "ms200.h"



static const char *TAG = "LIDAR_MS200";


static ms200_data_t lidar_data = {0};
static SemaphoreHandle_t lidar_data_mutex;

static void lidar_data_mutex_init(void)
{
    if (lidar_data_mutex != NULL)
    {
        return;
    }

    lidar_data_mutex = xSemaphoreCreateMutex();
    if (lidar_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create lidar data mutex");
        abort();
    }
}


// 激光雷达解析数据任务
// Lidar data parsing mission
static void Lidar_Ms200_Task(void *arg)
{
    ESP_LOGI(TAG, "Start Lidar_Ms200_Task with core:%d", xPortGetCoreID());
    uint16_t rx_count = 0;
    
    while (1)
    {
        rx_count = Uart1_Available();
        if (rx_count)
        {
            // Uart1_Clean_Buffer();
            for (int i = 0; i < rx_count; i++)
            {
                Ms200_Data_Receive(Uart1_Read());
            }
        }
        if (Ms200_New_Package())
        {
            ms200_data_t next_data = {0};

            Ms200_Clear_New_Package_State();
            Ms200_Get_Data(&next_data);

            xSemaphoreTake(lidar_data_mutex, portMAX_DELAY);
            lidar_data = next_data;
            xSemaphoreGive(lidar_data_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    vTaskDelete(NULL);
}

// 复制当前激光雷达缓存数据
// Copy the current lidar cache data
void Lidar_Ms200_Copy_Data(ms200_data_t *out_data)
{
    if (out_data == NULL)
    {
        return;
    }

    if (lidar_data_mutex == NULL)
    {
        *out_data = lidar_data;
        return;
    }

    xSemaphoreTake(lidar_data_mutex, portMAX_DELAY);
    *out_data = lidar_data;
    xSemaphoreGive(lidar_data_mutex);
}

// 读取激光雷达某个点检测的距离
// Read the distance detected by the lidar at a point
uint16_t Lidar_Ms200_Get_Distance(uint16_t point)
{
    uint16_t distance = 0;

    if (point < MS200_POINT_MAX)
    {
        if (lidar_data_mutex != NULL)
        {
            xSemaphoreTake(lidar_data_mutex, portMAX_DELAY);
        }
        distance = lidar_data.points[point].distance;
        if (lidar_data_mutex != NULL)
        {
            xSemaphoreGive(lidar_data_mutex);
        }
    }

    return distance;
}

// 读取激光雷达某个点检测的强度
// Read the intensity detected by the lidar at a point
uint16_t Lidar_Ms200_Get_Intensity(uint16_t point)
{
    uint16_t intensity = 0;

    if (point < MS200_POINT_MAX)
    {
        if (lidar_data_mutex != NULL)
        {
            xSemaphoreTake(lidar_data_mutex, portMAX_DELAY);
        }
        intensity = lidar_data.points[point].intensity;
        if (lidar_data_mutex != NULL)
        {
            xSemaphoreGive(lidar_data_mutex);
        }
    }

    return intensity;
}



// 初始化MS200激光雷达
// Initialize the MS200 Lidar
void Lidar_Ms200_Init(void)
{
    lidar_data_mutex_init();

    xTaskCreatePinnedToCore(Lidar_Ms200_Task, "Lidar_Ms200_Task", 10*1024, NULL, 10, NULL, 1);
}



