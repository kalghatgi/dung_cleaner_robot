#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <time.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
//////////////////////////////
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#include <std_msgs/msg/float32_multi_array.h>

////////////////////////////////////////////// ROS_DOMAIN_ID //////////////////////////////////////////////
#define ROS_DOMAIN_ID 1
///////////////////////////////////////////////// SENSORS /////////////////////////////////////////////////
#define MOTOR1_PWM_GPIO GPIO_NUM_12            // PWM (conveyor motor)
#define MOTOR1_DIRECTION_GPIO GPIO_NUM_14      // Digital Output
#define MOTOR2_PWM_GPIO GPIO_NUM_25            // PWM (bin motor)
#define MOTOR2_DIRECTION_GPIO GPIO_NUM_26      // Digital Output
#define MAX_LIMIT_SWITCH_GPIO GPIO_NUM_4       // Digital Input
#define MIN_LIMIT_SWITCH_GPIO GPIO_NUM_15      // Digital Input
#define DISTANCE_SENSOR_GPIO GPIO_NUM_18       // Digital Input
#define BIN_EMPTY_BUTTON_GPIO GPIO_NUM_2      // Digital Input
#define RELAY_GPIO GPIO_NUM_33 // Relay control pin

#define NUMBER_OF_MOTORS 2
#define GPIO_DIGITAL_INPUT_PINS_MASK ((1ULL << MAX_LIMIT_SWITCH_GPIO) | (1ULL << MIN_LIMIT_SWITCH_GPIO) | (1ULL << DISTANCE_SENSOR_GPIO) | (1ULL << BIN_EMPTY_BUTTON_GPIO))
#define GPIO_DIGITAL_OUTPUT_PINS_MASK ((1ULL << MOTOR1_DIRECTION_GPIO) | (1ULL << MOTOR2_DIRECTION_GPIO) | (1ULL << RELAY_GPIO))
#define PWM_HS_TIMER LEDC_TIMER_0
#define PWM_HS_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR1_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR2_PWM_CHANNEL LEDC_CHANNEL_1
#define PWM_RESOLUTION_BITS LEDC_TIMER_11_BIT
#define PWM_RESOLUTION pow(2, (float)PWM_RESOLUTION_BITS)
#define PWM_FREQUENCY 20000 // Should be lesser than 80MHz/(2^PWM_RESOLUTION)

float Motor_Duty_Cycle[NUMBER_OF_MOTORS];
static const char* TAG = "bin_control";

rcl_publisher_t motor_status_publisher;
std_msgs__msg__Float32MultiArray motor_status;

void Set_Inverted_PWM(ledc_mode_t _mode, ledc_channel_t _channel, float _percent_duty)
{
    uint32_t _duty = uint32_t((PWM_RESOLUTION - 1) * (_percent_duty / 100.0f));
    ledc_set_duty_and_update(_mode, _channel, _duty, 0);
}

void Set_Motor_Speed()
{
    float _motor1_percent_duty_cycle = Motor_Duty_Cycle[0];
    float _motor2_percent_duty_cycle = Motor_Duty_Cycle[1];

    if(_motor1_percent_duty_cycle >= 0)
    { // CCW
        Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_PWM_CHANNEL, fabs(_motor1_percent_duty_cycle));
        gpio_set_level(MOTOR1_DIRECTION_GPIO, 0);
        ESP_LOGI(TAG, "Motor 1 moving CCW with duty cycle: %.2f%%", _motor1_percent_duty_cycle);
    }
    else
    { // CW
        Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_PWM_CHANNEL, fabs(_motor1_percent_duty_cycle));
        gpio_set_level(MOTOR1_DIRECTION_GPIO, 1);
        ESP_LOGI(TAG, "Motor 1 moving CW with duty cycle: %.2f%%", _motor1_percent_duty_cycle);
    }
    if(_motor2_percent_duty_cycle >= 0)
    { // CCW
        Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_PWM_CHANNEL, fabs(_motor2_percent_duty_cycle));
        gpio_set_level(MOTOR2_DIRECTION_GPIO, 0);
        ESP_LOGI(TAG, "Motor 2 moving CCW with duty cycle: %.2f%%", _motor2_percent_duty_cycle);
    }
    else
    { // CW
        Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_PWM_CHANNEL, fabs(_motor2_percent_duty_cycle));
        gpio_set_level(MOTOR2_DIRECTION_GPIO, 1);
        ESP_LOGI(TAG, "Motor 2 moving CW with duty cycle: %.2f%%", _motor2_percent_duty_cycle);
    }
}

void TASK_motor_control(void *args)
{
    bool bottom_limit_reached = false;
    bool top_limit_reached = false;
    bool distance_limit = false;
    bool bin_empty_button_pressed = false;
    static bool relay_on = false;

    while (true)
    {
        bottom_limit_reached = gpio_get_level(MIN_LIMIT_SWITCH_GPIO);
        top_limit_reached = gpio_get_level(MAX_LIMIT_SWITCH_GPIO);
        distance_limit = gpio_get_level(DISTANCE_SENSOR_GPIO);
        bin_empty_button_pressed = gpio_get_level(BIN_EMPTY_BUTTON_GPIO);

        ESP_LOGI(TAG, "Bottom limit switch: %d", bottom_limit_reached);
        ESP_LOGI(TAG, "Top limit switch: %d", top_limit_reached);
        ESP_LOGI(TAG, "Distance sensor: %d", distance_limit);
        ESP_LOGI(TAG, "Bin empty button pressed: %d", bin_empty_button_pressed);

        // New logic to control motor1 and relay output
        // Logic 1: Distance sensor is true, lower limit is false, top limit is true, bin button is false
        // if (distance_limit && !bottom_limit_reached && top_limit_reached && !bin_empty_button_pressed)
        if (!bin_empty_button_pressed)
        {
            if (distance_limit && !bottom_limit_reached && top_limit_reached)
            {
                Motor_Duty_Cycle[0] = 50.0; // Motor1 at 10% duty cycle
                Motor_Duty_Cycle[1] = 0.0;  // Motor2 off
                gpio_set_level(RELAY_GPIO, 1); // Turn on the relay
                ESP_LOGI(TAG, "Condition 1: Conveyor ON, Relay ON for 500 ticks.");
                vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ticks

                gpio_set_level(RELAY_GPIO, 0); // Turn off the relay
                ESP_LOGI(TAG, "Relay OFF for 2000 ticks.");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 2000 ticks
            }
            else if(!distance_limit && !bottom_limit_reached && top_limit_reached)
            {
                Motor_Duty_Cycle[0] = 0.0; // Stop Motor1
                gpio_set_level(RELAY_GPIO, 1); // Turn on the relay
                ESP_LOGI(TAG, "Condition 2: Bin Full Conveyor stopped.");
                vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ticks

                gpio_set_level(RELAY_GPIO, 0); // Turn off the relay
                ESP_LOGI(TAG, "Relay OFF for 2000 ticks.");
                vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 2000 ticks
            }
        }
        else if (bin_empty_button_pressed)
        {
            if(!bottom_limit_reached && top_limit_reached && !distance_limit)
            {
                Motor_Duty_Cycle[0] = 0.0;
                Motor_Duty_Cycle[1] = 50.0;
                gpio_set_level(RELAY_GPIO, 0); // Turn off the relay
                ESP_LOGI(TAG, "Condition 3: Bin full, Lift Bin.");
                if(!top_limit_reached && bottom_limit_reached)
                {
                    Motor_Duty_Cycle[0] = 0.0;
                    Motor_Duty_Cycle[1] = 0.0;
                    gpio_set_level(RELAY_GPIO, 1); // Turn on the relay
                    vTaskDelay(pdMS_TO_TICKS(700)); // Delay for 1000 ticks
                    gpio_set_level(RELAY_GPIO, 0); // Turn off the relay
                    vTaskDelay(pdMS_TO_TICKS(3000)); // Delay for 1000 ticks
                }
            }
        }
        else if(!bin_empty_button_pressed && bottom_limit_reached)
        {
            Motor_Duty_Cycle[0] = 0.0;
            Motor_Duty_Cycle[1] = -50.0;
            gpio_set_level(RELAY_GPIO, 1); // Turn off the relay
            ESP_LOGI(TAG, "Condition 4: Bin Empty, Lower Bin");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ticks
            if (!bottom_limit_reached)
            {
                Motor_Duty_Cycle[0] = 0.0;
                Motor_Duty_Cycle[1] = 0.0;
                gpio_set_level(RELAY_GPIO, 0); // Turn off the relay
                ESP_LOGI(TAG, "Condition 5: Bin Empty.");
            }
        }
        else // Distance sensor is true
        {
            gpio_set_level(RELAY_GPIO, 0); // Turn off the relay permanently
            relay_on = false;
            ESP_LOGI(TAG, "Relay OFF: Distance sensor is true.");
        }

        Set_Motor_Speed();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}


void Setup_PWM()
{
    /////////////////// Timer configuration ///////////////////
    ledc_timer_config_t motor_timer = {
        .speed_mode = PWM_HS_MODE,
        .duty_resolution = PWM_RESOLUTION_BITS,
        .timer_num = PWM_HS_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ledc_timer_config(&motor_timer);
    /////////////////// Channel configuration ///////////////////
    // Channel 1 //
    ledc_channel_config_t motor1_channel;
    motor1_channel.gpio_num   = MOTOR1_PWM_GPIO;
    motor1_channel.speed_mode = PWM_HS_MODE;
    motor1_channel.channel    = MOTOR1_PWM_CHANNEL;
    motor1_channel.intr_type  = LEDC_INTR_DISABLE;
    motor1_channel.timer_sel  = PWM_HS_TIMER;
    motor1_channel.duty       = 0;
    motor1_channel.hpoint     = 0;
    motor1_channel.flags.output_invert = 0;
    ledc_channel_config(&motor1_channel);
    // Channel 2 //
    ledc_channel_config_t motor2_channel;
    motor2_channel.gpio_num   = MOTOR2_PWM_GPIO;
    motor2_channel.speed_mode = PWM_HS_MODE;
    motor2_channel.channel    = MOTOR2_PWM_CHANNEL;
    motor2_channel.intr_type  = LEDC_INTR_DISABLE;
    motor2_channel.timer_sel  = PWM_HS_TIMER;
    motor2_channel.duty       = 0;
    motor2_channel.hpoint     = 0;
    motor2_channel.flags.output_invert = 0;
    ledc_channel_config(&motor2_channel);
    /////////////////// Fade configuration ///////////////////
    ledc_fade_func_install(0);
    Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_PWM_CHANNEL, 0.0);
    Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_PWM_CHANNEL, 0.0);
    ESP_LOGI("IO_config", "PWM - check");
}

void Setup_Digital_IO()
{
    ////////////////////////////////// DIGITAL INPUTS //////////////////////////////////
    gpio_config_t Digital_Input_Pins_Config = (gpio_config_t)
    {
        .pin_bit_mask = GPIO_DIGITAL_INPUT_PINS_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&Digital_Input_Pins_Config);

    // Configure individual pull-ups and pull-downs as needed
    gpio_set_pull_mode(MAX_LIMIT_SWITCH_GPIO, GPIO_PULLUP_ONLY);  // GPIO 4 - pull-up
    gpio_set_pull_mode(MIN_LIMIT_SWITCH_GPIO, GPIO_PULLUP_ONLY);  // GPIO 15 - pull-up
    gpio_set_pull_mode(BIN_EMPTY_BUTTON_GPIO, GPIO_PULLUP_ONLY);  // GPIO 2 - pull-up
    gpio_set_pull_mode(DISTANCE_SENSOR_GPIO, GPIO_PULLDOWN_ONLY); // GPIO 18 - pull-down

    ESP_LOGI("IO_config", "Digital Inputs - check");

    ////////////////////////////////// DIGITAL OUTPUTS //////////////////////////////////
    gpio_config_t Digital_Output_Pins_Config = (gpio_config_t)
    {
        .pin_bit_mask = GPIO_DIGITAL_OUTPUT_PINS_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&Digital_Output_Pins_Config);
    ESP_LOGI("IO_config", "Digital Outputs - check");
}

void Setup_UART()
{
    // when designing the custom hardware, use UART0 for flash and logging as usual, meanwhile use UART1 for microROS
    // ******************************* UART0 config for microROS transport ******************************* //
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    static uart_port_t microros_uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(
        true,
        (void *) &microros_uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#else
#error micro-ROS transports misconfigured
#endif
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_node_t node;
    rclc_node_init_default(&node, "bin_control_node", "", &support);

    // Create publisher
    rcl_publisher_t publisher;
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_status_topic");

    // Create a message
    std_msgs__msg__Float32MultiArray msg;
    std_msgs__msg__Float32MultiArray__init(&msg);
    msg.data.capacity = 2;
    msg.data.size = 2;
    msg.data.data = (float *)malloc(2 * sizeof(float));

    while (true)
    {
        msg.data.data[0] = Motor_Duty_Cycle[0];
        msg.data.data[1] = Motor_Duty_Cycle[1];
        rcl_publish(&publisher, &msg, NULL);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Publish every second
    }

    // Clean up
    std_msgs__msg__Float32MultiArray__fini(&msg);
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    Setup_PWM();
    Setup_Digital_IO();
    Setup_UART();

    xTaskCreate(TASK_motor_control, "Motor Control Task", 2048, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "Micro ROS Task", 4096, NULL, 5, NULL);
}
