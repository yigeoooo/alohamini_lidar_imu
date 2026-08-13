#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <micro_ros_utilities/string_utilities.h>
#include <rcl/context.h>
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

#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT CONFIG_MICRO_ROS_AGENT_PORT

#define LIDAR_POINT_COUNT 360
#if LIDAR_POINT_COUNT != MS200_POINT_MAX
#error "LIDAR_POINT_COUNT must match MS200_POINT_MAX"
#endif
#define LIDAR_PUBLISH_PERIOD_MS 90
#define EXECUTOR_SPIN_TIMEOUT_MS 5
#define DEG_TO_RAD (M_PI / 180.0)
#define STANDARD_GRAVITY 9.80665
#define IMU_ANGULAR_VELOCITY_VARIANCE 0.01
#define IMU_LINEAR_ACCELERATION_VARIANCE 0.25
#define TIME_SYNC_INITIAL_TIMEOUT_MS 1000
#define TIME_SYNC_RETRY_TIMEOUT_MS 100
#define TIME_SYNC_RETRY_INTERVAL_MS 5000
#define TIME_SYNC_REFRESH_INTERVAL_MS 60000
#define AGENT_STARTUP_PING_TIMEOUT_MS 1000
#define AGENT_STARTUP_RETRY_INTERVAL_MS 1000
#define ENTITY_CREATION_SESSION_TIMEOUT_MS 5000

static const char *TAG = "LIDAR_IMU";

static rcl_publisher_t publisher_lidar;
static rcl_publisher_t publisher_imu;
static sensor_msgs__msg__LaserScan msg_lidar;
static sensor_msgs__msg__Imu msg_imu;
static rcl_timer_t timer_lidar;
static rcl_timer_t timer_imu;
static SemaphoreHandle_t lidar_msg_mutex;
static SemaphoreHandle_t imu_msg_mutex;

static int64_t time_offset_ms = 0;
static unsigned long long last_timestamp_ms = 0;
static unsigned long last_time_sync_attempt_ms = 0;
static int time_synchronized = 0;
static int consecutive_lidar_failures = 0;
static int consecutive_imu_failures = 0;

static void set_frame_id(rosidl_runtime_c__String *field, const char *content_frame_id)
{
    *field = micro_ros_string_utilities_set(*field, content_frame_id);
}

static void lidar_ros_init(void)
{
    msg_lidar.angle_min = -180 * M_PI / 180.0;
    msg_lidar.angle_increment = 1 * M_PI / 180.0;
    msg_lidar.angle_max = msg_lidar.angle_min + (LIDAR_POINT_COUNT - 1) * msg_lidar.angle_increment;
    msg_lidar.scan_time = (float)LIDAR_PUBLISH_PERIOD_MS / 1000.0f;
    msg_lidar.time_increment = msg_lidar.scan_time / (float)LIDAR_POINT_COUNT;
    msg_lidar.range_min = 0.12;
    msg_lidar.range_max = 8.0;

    msg_lidar.ranges.data = (float *)malloc(LIDAR_POINT_COUNT * sizeof(float));
    if (msg_lidar.ranges.data == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate lidar ranges");
        abort();
    }
    msg_lidar.ranges.size = LIDAR_POINT_COUNT;
    msg_lidar.ranges.capacity = LIDAR_POINT_COUNT;
    for (size_t i = 0; i < msg_lidar.ranges.size; i++)
    {
        msg_lidar.ranges.data[i] = INFINITY;
    }

    msg_lidar.intensities.data = NULL;
    msg_lidar.intensities.size = 0;
    msg_lidar.intensities.capacity = 0;

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

    for (size_t i = 0; i < 9; i++)
    {
        msg_imu.orientation_covariance[i] = 0.0;
        msg_imu.angular_velocity_covariance[i] = 0.0;
        msg_imu.linear_acceleration_covariance[i] = 0.0;
    }
    msg_imu.orientation_covariance[0] = -1.0;
    msg_imu.angular_velocity_covariance[0] = IMU_ANGULAR_VELOCITY_VARIANCE;
    msg_imu.angular_velocity_covariance[4] = IMU_ANGULAR_VELOCITY_VARIANCE;
    msg_imu.angular_velocity_covariance[8] = IMU_ANGULAR_VELOCITY_VARIANCE;
    msg_imu.linear_acceleration_covariance[0] = IMU_LINEAR_ACCELERATION_VARIANCE;
    msg_imu.linear_acceleration_covariance[4] = IMU_LINEAR_ACCELERATION_VARIANCE;
    msg_imu.linear_acceleration_covariance[8] = IMU_LINEAR_ACCELERATION_VARIANCE;

    set_frame_id(&msg_imu.header.frame_id, "imu_frame");
}

static void msg_mutex_init(void)
{
    lidar_msg_mutex = xSemaphoreCreateMutex();
    imu_msg_mutex = xSemaphoreCreateMutex();
    if (lidar_msg_mutex == NULL || imu_msg_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create message mutexes");
        abort();
    }
}

static void lidar_update_data_task(void *arg)
{
    ms200_data_t lidar_snapshot = {0};

    while (1)
    {
        Lidar_Ms200_Copy_Data(&lidar_snapshot);

        xSemaphoreTake(lidar_msg_mutex, portMAX_DELAY);
        for (int i = 0; i < MS200_POINT_MAX; i++)
        {
            uint16_t index = (MS200_POINT_MAX - i) % MS200_POINT_MAX;

            uint16_t distance_mm = lidar_snapshot.points[index].distance;
            float distance_m = (float)distance_mm / 1000.0f;
            if (distance_mm == 0 || distance_m < msg_lidar.range_min || distance_m > msg_lidar.range_max)
            {
                msg_lidar.ranges.data[i] = INFINITY;
            }
            else
            {
                msg_lidar.ranges.data[i] = distance_m;
            }
        }
        xSemaphoreGive(lidar_msg_mutex);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void imu_update_data_task(void *arg)
{
    float imu_accel_g[3] = {0};
    float imu_gyro_dps[3] = {0};

    while (1)
    {
        Icm42670p_Get_Accel_Gyro(imu_accel_g, imu_gyro_dps);

        xSemaphoreTake(imu_msg_mutex, portMAX_DELAY);
        msg_imu.angular_velocity.x = imu_gyro_dps[0] * DEG_TO_RAD;
        msg_imu.angular_velocity.y = imu_gyro_dps[1] * DEG_TO_RAD;
        msg_imu.angular_velocity.z = imu_gyro_dps[2] * DEG_TO_RAD;

        msg_imu.linear_acceleration.x = imu_accel_g[0] * STANDARD_GRAVITY;
        msg_imu.linear_acceleration.y = imu_accel_g[1] * STANDARD_GRAVITY;
        msg_imu.linear_acceleration.z = imu_accel_g[2] * STANDARD_GRAVITY;
        xSemaphoreGive(imu_msg_mutex);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static unsigned long get_millisecond(void)
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

static void sync_time(uint32_t timeout_ms)
{
    unsigned long attempt_started_ms = get_millisecond();
    last_time_sync_attempt_ms = attempt_started_ms;
    rcl_ret_t rc = rmw_uros_sync_session(timeout_ms);
    if (rc == RCL_RET_OK)
    {
        unsigned long now = get_millisecond();
        int64_t ros_time_ms = (int64_t)rmw_uros_epoch_millis();
        int64_t new_offset_ms = ros_time_ms - (int64_t)now;
        int64_t candidate_time_ms = (int64_t)now + new_offset_ms;
        if (candidate_time_ms < last_timestamp_ms)
        {
            new_offset_ms += (int64_t)last_timestamp_ms - candidate_time_ms;
            ESP_LOGW(TAG, "Clock sync would move backwards; clamping to last published stamp");
        }
        time_offset_ms = new_offset_ms;
        time_synchronized = 1;
        ESP_LOGI(TAG, "micro-ROS time synchronized");
    }
    else if (time_synchronized)
    {
        ESP_LOGW(TAG, "micro-ROS time refresh failed (%d), keeping previous offset", (int)rc);
    }
    else
    {
        ESP_LOGW(TAG, "micro-ROS time sync failed (%d), waiting without publishing", (int)rc);
    }
}

static void maybe_sync_time(void)
{
    unsigned long now = get_millisecond();
    unsigned long interval_ms = time_synchronized ? TIME_SYNC_REFRESH_INTERVAL_MS : TIME_SYNC_RETRY_INTERVAL_MS;
    if ((now - last_time_sync_attempt_ms) >= interval_ms)
    {
        sync_time(TIME_SYNC_RETRY_TIMEOUT_MS);
    }
}

static struct timespec get_timespec(void)
{
    struct timespec tp = {0};
    int64_t candidate_ms = (int64_t)get_millisecond() + time_offset_ms;
    unsigned long long now = candidate_ms > 0 ? (unsigned long long)candidate_ms : 0;
    if (now <= last_timestamp_ms)
    {
        now = last_timestamp_ms + 1;
    }
    last_timestamp_ms = now;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

static void copy_lidar_msg_for_publish(sensor_msgs__msg__LaserScan *publish_msg,
                                       float ranges[LIDAR_POINT_COUNT])
{
    xSemaphoreTake(lidar_msg_mutex, portMAX_DELAY);
    *publish_msg = msg_lidar;
    memcpy(ranges, msg_lidar.ranges.data, LIDAR_POINT_COUNT * sizeof(float));
    xSemaphoreGive(lidar_msg_mutex);

    publish_msg->ranges.data = ranges;
    publish_msg->ranges.size = LIDAR_POINT_COUNT;
    publish_msg->ranges.capacity = LIDAR_POINT_COUNT;
    publish_msg->intensities.data = NULL;
    publish_msg->intensities.size = 0;
    publish_msg->intensities.capacity = 0;
}

static void copy_imu_msg_for_publish(sensor_msgs__msg__Imu *publish_msg)
{
    xSemaphoreTake(imu_msg_mutex, portMAX_DELAY);
    *publish_msg = msg_imu;
    xSemaphoreGive(imu_msg_mutex);
}

static void timer_lidar_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    static uint32_t publish_count = 0;
    static float publish_ranges[LIDAR_POINT_COUNT];
    sensor_msgs__msg__LaserScan publish_msg;

    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        if (!time_synchronized)
            return;
        struct timespec time_stamp = get_timespec();
        copy_lidar_msg_for_publish(&publish_msg, publish_ranges);
        publish_msg.header.stamp.sec = time_stamp.tv_sec;
        publish_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        rcl_ret_t rc = rcl_publish(&publisher_lidar, &publish_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            consecutive_lidar_failures++;
            ESP_LOGW(TAG, "scan publish failed: %d (%d consecutive)", (int)rc, consecutive_lidar_failures);
        }
        else
        {
            consecutive_lidar_failures = 0;
        }
        if ((++publish_count % 100) == 0)
        {
            ESP_LOGI(TAG, "published scan samples: %lu", (unsigned long)publish_count);
        }
    }
}

static void timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    static uint32_t publish_count = 0;
    sensor_msgs__msg__Imu publish_msg;

    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        if (!time_synchronized)
            return;
        struct timespec time_stamp = get_timespec();
        copy_imu_msg_for_publish(&publish_msg);
        publish_msg.header.stamp.sec = time_stamp.tv_sec;
        publish_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        rcl_ret_t rc = rcl_publish(&publisher_imu, &publish_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            consecutive_imu_failures++;
            ESP_LOGW(TAG, "imu publish failed: %d (%d consecutive)", (int)rc, consecutive_imu_failures);
        }
        else
        {
            consecutive_imu_failures = 0;
        }
        if ((++publish_count % 100) == 0)
        {
            ESP_LOGI(TAG, "published imu samples: %lu", (unsigned long)publish_count);
        }
    }
}

static void reset_runtime_state(void)
{
    time_synchronized = 0;
    last_time_sync_attempt_ms = 0;
    consecutive_lidar_failures = 0;
    consecutive_imu_failures = 0;
}

static void micro_ros_task(void *arg)
{
    (void)arg;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    memset(&support, 0, sizeof(support));
    support.context = rcl_get_zero_initialized_context();
    support.clock.type = RCL_CLOCK_UNINITIALIZED;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    publisher_lidar = rcl_get_zero_initialized_publisher();
    publisher_imu = rcl_get_zero_initialized_publisher();
    timer_lidar = rcl_get_zero_initialized_timer();
    timer_imu = rcl_get_zero_initialized_timer();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rmw_init_options_t *rmw_options = NULL;
    rcl_ret_t rc = RCL_RET_ERROR;
    const char *failed_stage = "unknown";

    failed_stage = "rcl_init_options_init";
    rc = rcl_init_options_init(&init_options, allocator);
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "micro-ROS init options ready");

    failed_stage = "rcl_init_options_set_domain_id";
    rc = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    failed_stage = "rmw_uros_options_set_udp_address";
    rc = rmw_uros_options_set_udp_address(ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    ESP_LOGI(TAG, "Waiting for micro-ROS Agent at %s:%s", ROS_AGENT_IP, ROS_AGENT_PORT);
    while (rmw_uros_ping_agent_options(AGENT_STARTUP_PING_TIMEOUT_MS, 1, rmw_options) != RMW_RET_OK)
    {
        ESP_LOGE(TAG, "micro-ROS Agent is not available yet; retrying in %d ms",
                 AGENT_STARTUP_RETRY_INTERVAL_MS);
        vTaskDelay(pdMS_TO_TICKS(AGENT_STARTUP_RETRY_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "micro-ROS Agent is reachable; starting node initialization");

    failed_stage = "rclc_support_init_with_options";
    rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "micro-ROS support initialized");

    failed_stage = "rclc_node_init_default";
    rc = rclc_node_init_default(&node, "lidar_imu_publisher", ROS_NAMESPACE, &support);
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "micro-ROS node started: /%s", "lidar_imu_publisher");

    failed_stage = "rmw_uros_set_context_entity_creation_session_timeout";
    rc = rmw_uros_set_context_entity_creation_session_timeout(
        rcl_context_get_rmw_context(&support.context), ENTITY_CREATION_SESSION_TIMEOUT_MS);
    if (rc != RMW_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "micro-ROS entity creation timeout set to %d ms",
             ENTITY_CREATION_SESSION_TIMEOUT_MS);

    failed_stage = "rclc_publisher_init_default(scan)";
    rc = rclc_publisher_init_default(&publisher_lidar, &node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "LaserScan publisher started: /scan");

    failed_stage = "rclc_publisher_init_default(imu)";
    rc = rclc_publisher_init_default(&publisher_imu, &node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "IMU publisher started: /imu");

    failed_stage = "rclc_timer_init_default(scan)";
    rc = rclc_timer_init_default(&timer_lidar, &support,
                                 RCL_MS_TO_NS(LIDAR_PUBLISH_PERIOD_MS), timer_lidar_callback);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    failed_stage = "rclc_timer_init_default(imu)";
    rc = rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(50), timer_imu_callback);
    if (rc != RCL_RET_OK)
        goto startup_failed;
    ESP_LOGI(TAG, "sensor timers started: scan=%d ms, imu=%d ms",
             LIDAR_PUBLISH_PERIOD_MS, 50);

    failed_stage = "rclc_executor_init";
    rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    failed_stage = "rclc_executor_add_timer(scan)";
    rc = rclc_executor_add_timer(&executor, &timer_lidar);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    failed_stage = "rclc_executor_add_timer(imu)";
    rc = rclc_executor_add_timer(&executor, &timer_imu);
    if (rc != RCL_RET_OK)
        goto startup_failed;

    reset_runtime_state();
    sync_time(TIME_SYNC_INITIAL_TIMEOUT_MS);
    if (time_synchronized)
        ESP_LOGI(TAG, "micro-ROS node startup complete; publishing /scan and /imu");
    else
        ESP_LOGW(TAG, "micro-ROS node started, but time sync is pending; publishing will wait");

    while (1)
    {
        rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS));
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT)
        {
            ESP_LOGE(TAG, "executor spin failed: %d; keeping entities alive and continuing", (int)rc);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        maybe_sync_time();
        usleep(1000);
    }

startup_failed:
    rcl_error_string_t error_string = rcl_get_error_string();
    ESP_LOGE(TAG,
             "micro-ROS node startup failed at %s (rc=%d): %s; no automatic teardown or reconnect will be attempted",
             failed_stage, (int)rc, error_string.str);
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
    msg_mutex_init();

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
