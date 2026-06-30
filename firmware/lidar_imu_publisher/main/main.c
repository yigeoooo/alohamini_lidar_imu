#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <uros_network_interfaces.h>

#include "icm42670p.h"
#include "lidar_ms200.h"
#include "ms200.h"
#include "uart1.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }

#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT CONFIG_MICRO_ROS_AGENT_PORT

#define LIDAR_POINT_COUNT 360
#define DEG_TO_RAD (M_PI / 180.0)
#define STANDARD_GRAVITY 9.80665

static const char *TAG = "LIDAR_IMU";

static rcl_publisher_t publisher_lidar;
static rcl_publisher_t publisher_imu;
static sensor_msgs__msg__LaserScan msg_lidar;
static sensor_msgs__msg__Imu msg_imu;
static rcl_timer_t timer_lidar;
static rcl_timer_t timer_imu;

static unsigned long long time_offset = 0;

static void set_frame_id(rosidl_runtime_c__String *field, const char *content_frame_id)
{
    int len_namespace = strlen(ROS_NAMESPACE);
    int len_frame_id_max = len_namespace + strlen(content_frame_id) + 2;
    char *frame_id = malloc(len_frame_id_max);
    if (frame_id == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate frame_id");
        return;
    }

    if (len_namespace == 0)
    {
        sprintf(frame_id, "%s", content_frame_id);
    }
    else
    {
        sprintf(frame_id, "%s/%s", ROS_NAMESPACE, content_frame_id);
    }

    *field = micro_ros_string_utilities_set(*field, frame_id);
    free(frame_id);
}

static void lidar_ros_init(void)
{
    msg_lidar.angle_min = -180 * M_PI / 180.0;
    msg_lidar.angle_increment = 1 * M_PI / 180.0;
    msg_lidar.angle_max = msg_lidar.angle_min + (LIDAR_POINT_COUNT - 1) * msg_lidar.angle_increment;
    msg_lidar.range_min = 0.12;
    msg_lidar.range_max = 8.0;

    msg_lidar.ranges.data = (float *)malloc(LIDAR_POINT_COUNT * sizeof(float));
    msg_lidar.ranges.size = LIDAR_POINT_COUNT;
    msg_lidar.ranges.capacity = LIDAR_POINT_COUNT;
    for (size_t i = 0; i < msg_lidar.ranges.size; i++)
    {
        msg_lidar.ranges.data[i] = 0.0;
    }

    msg_lidar.intensities.data = (float *)malloc(LIDAR_POINT_COUNT * sizeof(float));
    msg_lidar.intensities.size = LIDAR_POINT_COUNT;
    msg_lidar.intensities.capacity = LIDAR_POINT_COUNT;
    for (size_t i = 0; i < msg_lidar.intensities.size; i++)
    {
        msg_lidar.intensities.data[i] = 10.0;
    }

    set_frame_id(&msg_lidar.header.frame_id, "laser_frame");
}

static void imu_ros_init(void)
{
    msg_imu.angular_velocity.x = 0.0;
    msg_imu.angular_velocity.y = 0.0;
    msg_imu.angular_velocity.z = 0.0;

    msg_imu.linear_acceleration.x = 0.0;
    msg_imu.linear_acceleration.y = 0.0;
    msg_imu.linear_acceleration.z = 0.0;

    msg_imu.orientation.x = 0.0;
    msg_imu.orientation.y = 0.0;
    msg_imu.orientation.z = 0.0;
    msg_imu.orientation.w = 1.0;
    msg_imu.orientation_covariance[0] = -1.0;

    set_frame_id(&msg_imu.header.frame_id, "imu_frame");
}

static void lidar_update_data_task(void *arg)
{
    uint16_t distance_mm[MS200_POINT_MAX] = {0};
    uint8_t intensity[MS200_POINT_MAX] = {0};

    while (1)
    {
        for (int i = 0; i < MS200_POINT_MAX; i++)
        {
            distance_mm[i] = Lidar_Ms200_Get_Distance(i);
            intensity[i] = Lidar_Ms200_Get_Intensity(i);
        }

        for (int i = 0; i < MS200_POINT_MAX; i++)
        {
            uint16_t index = (MS200_POINT_MAX - i) % MS200_POINT_MAX;
            if (index >= 180)
            {
                index = (index - 180) % MS200_POINT_MAX;
            }
            else
            {
                index = (index + 180) % MS200_POINT_MAX;
            }

            msg_lidar.ranges.data[i] = (float)(distance_mm[index] / 1000.0);
            msg_lidar.intensities.data[i] = (float)(intensity[index]);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void imu_update_data_task(void *arg)
{
    float imu_accel_g[3] = {0};
    float imu_gyro_dps[3] = {0};

    while (1)
    {
        Icm42670p_Get_Accel_g(imu_accel_g);
        Icm42670p_Get_Gyro_dps(imu_gyro_dps);

        msg_imu.angular_velocity.x = imu_gyro_dps[0] * DEG_TO_RAD;
        msg_imu.angular_velocity.y = imu_gyro_dps[1] * DEG_TO_RAD;
        msg_imu.angular_velocity.z = imu_gyro_dps[2] * DEG_TO_RAD;

        msg_imu.linear_acceleration.x = imu_accel_g[0] * STANDARD_GRAVITY;
        msg_imu.linear_acceleration.y = imu_accel_g[1] * STANDARD_GRAVITY;
        msg_imu.linear_acceleration.z = imu_accel_g[2] * STANDARD_GRAVITY;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static unsigned long get_millisecond(void)
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static void sync_time(void)
{
    unsigned long now = get_millisecond();
    rcl_ret_t rc = rmw_uros_sync_session(1000);
    if (rc == RCL_RET_OK)
    {
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
        time_offset = ros_time_ms - now;
        ESP_LOGI(TAG, "micro-ROS time synchronized");
    }
    else
    {
        time_offset = 0;
        ESP_LOGW(TAG, "micro-ROS time sync failed (%d), using boot-relative timestamps", (int)rc);
    }
}

static struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    unsigned long long now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

static void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    static uint32_t publish_count = 0;
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_lidar.header.stamp.sec = time_stamp.tv_sec;
        msg_lidar.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_lidar, &msg_lidar, NULL));
        if ((++publish_count % 100) == 0)
        {
            ESP_LOGI(TAG, "published scan samples: %lu", (unsigned long)publish_count);
        }
    }
}

static void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    static uint32_t publish_count = 0;
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        struct timespec time_stamp = get_timespec();
        msg_imu.header.stamp.sec = time_stamp.tv_sec;
        msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
        if ((++publish_count % 100) == 0)
        {
            ESP_LOGI(TAG, "published imu samples: %lu", (unsigned long)publish_count);
        }
    }
}

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    int state_agent = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected agent: %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "lidar_imu_publisher", ROS_NAMESPACE, &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher_lidar,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    RCCHECK(rclc_timer_init_default(
        &timer_lidar,
        &support,
        RCL_MS_TO_NS(90),
        timer_lidar_callback));

    RCCHECK(rclc_timer_init_default(
        &timer_imu,
        &support,
        RCL_MS_TO_NS(50),
        timer_imu_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_lidar));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));

    sync_time();

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    RCCHECK(rcl_publisher_fini(&publisher_lidar, &node));
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    Uart1_Init();
    Lidar_Ms200_Init();
    Icm42670p_Init();

    ESP_ERROR_CHECK(uros_network_interface_initialize());

    lidar_ros_init();
    imu_ros_init();

    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    xTaskCreatePinnedToCore(lidar_update_data_task,
                "lidar_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);

    xTaskCreatePinnedToCore(imu_update_data_task,
                "imu_update_data_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL, 1);
}
