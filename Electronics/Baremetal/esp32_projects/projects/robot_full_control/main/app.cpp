#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "DFRobot_BMX160.h"
#include "cJSON.h"

///////////////////////////////////////////////// SENSORS /////////////////////////////////////////////////
#define BASE_MOTOR1_ENCODER_A_GPIO GPIO_NUM_39 // Digital Input (LEFT)
#define BASE_MOTOR1_ENCODER_B_GPIO GPIO_NUM_36 // Digital Input
#define BASE_MOTOR2_ENCODER_A_GPIO GPIO_NUM_19 // Digital Input (RIGHT)
#define BASE_MOTOR2_ENCODER_B_GPIO GPIO_NUM_18 // Digital Input
#define I2C_SDA GPIO_NUM_21 // Digital IO (I2C)
#define I2C_SCL GPIO_NUM_22 // Digital IO (I2C)
#define I2C_CLOCK_SPEED 400000 // Digital IO (I2C)
//////////////////////////////////////////////// ACTUATORS ////////////////////////////////////////////////
#define BASE_MOTOR1_PWM_GPIO GPIO_NUM_26	// PWM (LEFT)
#define BASE_MOTOR1_DIRECTION_GPIO GPIO_NUM_25 // Digital Output
#define BASE_MOTOR2_PWM_GPIO GPIO_NUM_17 // PWM (RIGHT)
#define BASE_MOTOR2_DIRECTION_GPIO GPIO_NUM_16 // Digital Output

#define DEGREES_TO_RADIANS (M_PI / 180)
#define PWM_HS_TIMER LEDC_TIMER_0
#define PWM_HS_MODE LEDC_HIGH_SPEED_MODE
#define BASE_MOTOR1_PWM_CHANNEL LEDC_CHANNEL_0
#define BASE_MOTOR2_PWM_CHANNEL LEDC_CHANNEL_1
#define NUMBER_OF_MOTORS_AT_BASE 2
#define NUMBER_OF_MOTORS_AT_PAYLOAD 2
#define NUMBER_OF_LIMIT_SWITCHES_AT_PAYLOAD 2
#define NUMBER_OF_DISTANCE_SENSORS_AT_PAYLOAD 1
#define GPIO_DIGITAL_INPUT_PINS_MASK ((1ULL << BASE_MOTOR1_ENCODER_B_GPIO) | (1ULL << BASE_MOTOR2_ENCODER_B_GPIO))
#define GPIO_DIGITAL_OUTPUT_PINS_MASK ((1ULL << BASE_MOTOR1_DIRECTION_GPIO) | (1ULL << BASE_MOTOR2_DIRECTION_GPIO))
#define GPIO_INTERRUPT_INPUT_PINS_MASK ((1ULL << BASE_MOTOR1_ENCODER_A_GPIO) | (1ULL << BASE_MOTOR2_ENCODER_A_GPIO))
#define ESP_INTR_FLAG_DEFAULT 0
#define RING(x, min, max) ((x < min) ? max : (x > max) ? min : x)
#define LIMIT(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define PWM_RESOLUTION_BITS LEDC_TIMER_11_BIT
#define PWM_RESOLUTION pow(2, (float)PWM_RESOLUTION_BITS)
#define PWM_FREQUENCY 20000 // Should be lesser than 80MHz/(2^PWM_RESOLUTION)
#define UART_PORT UART_NUM_0
#define UART_BAUD_RATE 460800
#define UART_BUFFER_SIZE 4096
#define UART_SENDER_PERIOD 10 // (milliseconds) the period at which the data should be sent to the ROS node over UART
#define ENCODER_CPR 2000 // Reference: https://robokits.co.in/motors/rhino-planetary-geared-24v-motor/100w-24v-encoder-servo-motor/rhino-servo-24v-60rpm-100w-ig52-extra-heavy-duty-planetary-encoder-servo-motor-160kgcm#:~:text=Quad%20Encoder%20requires-,2000,-Pulses%20Per%20Revolution
#define ENCODER_PPR (ENCODER_CPR / 4)
#define MOTOR_GEAR_RATIO 46.566 // Reference: https://robokits.co.in/motors/rhino-planetary-geared-24v-motor/100w-24v-encoder-servo-motor/rhino-servo-24v-60rpm-100w-ig52-extra-heavy-duty-planetary-encoder-servo-motor-160kgcm#:~:text=ratio%20is%201%20%3A-,47,-the%20Optical%20encoder
#define BASE_MOTOR_ROTATION_PER_ENCODER_COUNT (1.0f / (ENCODER_PPR * MOTOR_GEAR_RATIO)) // Base motor is just the motor without considering its gearbox
#define PACKET_START_CHARACTER '\r'
#define PACKET_END_CHARACTER '\n'

typedef enum
{
    P = 0,
    PI,
    PD,
    PID,
}PID_STRATEGY;

const ledc_channel_t BASE_MOTOR_PWM_CHANNEL[NUMBER_OF_MOTORS_AT_BASE] = {BASE_MOTOR1_PWM_CHANNEL, BASE_MOTOR2_PWM_CHANNEL};
const gpio_num_t BASE_MOTOR_DIRECTION_GPIO[NUMBER_OF_MOTORS_AT_BASE] = {BASE_MOTOR1_DIRECTION_GPIO, BASE_MOTOR2_DIRECTION_GPIO};
// const ledc_channel_t PAYLOAD_MOTOR_PWM_CHANNEL[NUMBER_OF_MOTORS_AT_PAYLOAD] {BASE_MOTOR1_PWM_CHANNEL, BASE_MOTOR2_PWM_CHANNEL}
// const gpio_num_t PAYLOAD_MOTOR_DIRECTION_GPIO[NUMBER_OF_MOTORS_AT_PAYLOAD] {BASE_MOTOR1_DIRECTION_GPIO, BASE_MOTOR2_DIRECTION_GPIO}
int64_t instantaneous_encoder_count[NUMBER_OF_MOTORS_AT_BASE] = {0UL};
int64_t previous_encoder_count[NUMBER_OF_MOTORS_AT_BASE] = {0U}, new_encoder_count[NUMBER_OF_MOTORS_AT_BASE] = {0U};
int8_t base_motor_duty_cycle[NUMBER_OF_MOTORS_AT_BASE] = {0U},
    payload_motor_duty_cycle[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0U};
float base_motor_torque_command[NUMBER_OF_MOTORS_AT_BASE] = {0.0f},
    base_motor_velocity_command[NUMBER_OF_MOTORS_AT_BASE] = {0.0f},
    base_motor_position_command[NUMBER_OF_MOTORS_AT_BASE] = {0.0f};
float payload_motor_torque_command[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f},
    payload_motor_velocity_command[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f},
    payload_motor_position_command[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f};
float base_motor_torque_feedback[NUMBER_OF_MOTORS_AT_BASE] = {0.0f},
    base_motor_velocity_feedback[NUMBER_OF_MOTORS_AT_BASE] = {0.0f},
    base_motor_position_feedback[NUMBER_OF_MOTORS_AT_BASE] = {0.0f};
float payload_motor_torque_feedback[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f},
    payload_motor_velocity_feedback[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f},
    payload_motor_position_feedback[NUMBER_OF_MOTORS_AT_PAYLOAD] = {0.0f};
bool payload_limit_switch_status[NUMBER_OF_LIMIT_SWITCHES_AT_PAYLOAD] = {false};
float payload_distance_sensor_reading[NUMBER_OF_DISTANCE_SENSORS_AT_PAYLOAD] = {0.0f};
const uint8_t pid_strategy = P;

DFRobot_BMX160 bmx160;
sBmx160SensorData_t mag_uT, gyro_DPS, accel_G;
struct timespec ros_node_last_received_message_time_stamp;
struct timespec current_time_stamp;
static TimerHandle_t uart_sender_timer;
static QueueHandle_t uart_queue;
uint64_t motor_control_current_time = 0, motor_control_previous_time = 0;

/* 
json from_ros_node = 
{
    {"rbc", // rbc = robot base control
        {"wc", // wc = wheel control
            {
                "t", // t = torque
                {
                    0.0, 0.0 // 2 motors
                },
                "v", // v = velocity
                {
                    0.0, 0.0 // 2 motors
                },
                "p", // p = position
                {
                    0.0, 0.0 // 2 motors
                }
            }
        }
    },
    {"rpc", // rpc = robot payload control
        {"cc", // cc = cleaner control
            {"t", // t = torque
                {
                    0.0, 0.0 // 2 motors
                }
            }
        }
    }
};
*/
/*
json to_ros_node = 
{
    {"rbf", // rbc = robot base feedback
        {"wf", // wf = wheel feedback
            {
                "t", // t = torque
                {
                    0.0, 0.0 // 2 motors
                },
                "v", // v = velocity
                {
                    0.0, 0.0 // 2 motors
                },
                "p", // p = position
                {
                    0.0, 0.0 // 2 motors
                }
            }
        },
        {"imu", // imu = inertial measurement unit
            {"a", // a = accelerometer
                {
                    0.0, 0.0, 0.0 // 3 axes
                }
            },
            {"g", // g = gyroscope
                {
                    0.0, 0.0, 0.0 // 3 axes
                }
            },
            {"m", // m = magnetometer
                {
                    0.0, 0.0, 0.0 // 3 axes
                }
            }
        }
    },
    {"rpf", // rpc = robot payload feedback
        {"ls", // ls = limit switch
            {
                false, false // 2 switches
            }
        },
        {"ds", // ds = distance sensor
            {
                0.0 // 1 sensor
            }
        }
    }
};
*/

void Set_Inverted_PWM(ledc_mode_t _mode, ledc_channel_t _channel, float _percent_duty)
{
	uint32_t _duty = uint32_t((PWM_RESOLUTION - 1) * (_percent_duty / 100.0f));
	ledc_set_duty_and_update(_mode, _channel, _duty, 0);
}
void motor_control(void)
{
    static float _error_instantaneous[NUMBER_OF_MOTORS_AT_BASE] = {0.0f},
        _error_previous[NUMBER_OF_MOTORS_AT_BASE] = {0.0f};
    static float _error_integral[NUMBER_OF_MOTORS_AT_BASE] = {0.0f};

    motor_control_current_time = esp_timer_get_time();
    float _dT = (motor_control_current_time - motor_control_previous_time)*0.000001;
    motor_control_previous_time = motor_control_current_time;

    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_BASE; _m++)
    {
        new_encoder_count[_m] = instantaneous_encoder_count[_m];
        int32_t _delta_count = new_encoder_count[_m] - previous_encoder_count[_m];
        previous_encoder_count[_m] = new_encoder_count[_m];
        base_motor_velocity_feedback[_m] = (_delta_count * BASE_MOTOR_ROTATION_PER_ENCODER_COUNT / _dT); // rotations per second

        _error_instantaneous[_m] = (base_motor_velocity_command[_m] - base_motor_velocity_feedback[_m]);
        _error_integral[_m] += (_error_instantaneous[_m] * _dT);
        float _error_derivative = ((_error_instantaneous[_m] - _error_previous[_m]) / _dT);

        switch (pid_strategy)
        {
            case (P):
            {
                const float Kp = 2.25f;
                base_motor_duty_cycle[_m] = (Kp * _error_instantaneous[_m]);

                break;
            }
            case (PI):
            {
                const float Kp = 0.75, Ki = 0.00005;
                base_motor_duty_cycle[_m] = ((Kp * _error_instantaneous[_m]) + (Ki * _error_integral[_m]));
                break;
            }
            case (PD):
            {
                const float Kp = 0.75, Kd = 0.000125;
                base_motor_duty_cycle[_m] = ((Kp * _error_instantaneous[_m]) + (Kd * _error_derivative));
                break;
            }
            case (PID):
            {
                const float Kp = 0.75, Ki = 0.00005, Kd = 0.000125;
                base_motor_duty_cycle[_m] = ((Kp * _error_instantaneous[_m]) + (Ki * _error_integral[_m]) + (Kd * _error_derivative));
                break;
            }
            default:
            {
                printf("Invalid motor control strategy selected\n");
                break;
            }
        }
        
        // scale the duty cycle and apply limits.
        base_motor_duty_cycle[_m] = LIMIT((base_motor_duty_cycle[_m]*100), -100, 100);

        Set_Inverted_PWM(PWM_HS_MODE, BASE_MOTOR_PWM_CHANNEL[_m], fabs(base_motor_duty_cycle[_m]));
        gpio_set_level(BASE_MOTOR_DIRECTION_GPIO[_m], ((base_motor_duty_cycle[_m]>=0)?0:1)); // 0 = CCW, 1 = CW
    }

    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_PAYLOAD; _m++)
    {
        payload_motor_duty_cycle[_m] = LIMIT(payload_motor_torque_command[_m], -100, 100);

        // Set_Inverted_PWM(PWM_HS_MODE, PAYLOAD_MOTOR_PWM_CHANNEL[_m], fabs(payload_motor_duty_cycle[_m]));
        // gpio_set_level(PAYLOAD_MOTOR_DIRECTION_GPIO[_m], ((payload_motor_duty_cycle[_m]>=0)?0:1)); // 0 = CCW, 1 = CW
    }
}

static void IRAM_ATTR Motor1_Encoder_ISR_Handler(void *args)
{
	if (gpio_get_level(BASE_MOTOR1_ENCODER_B_GPIO) > 0)
		instantaneous_encoder_count[0]++;
	else
		instantaneous_encoder_count[0]--;
}
static void IRAM_ATTR Motor2_Encoder_ISR_Handler(void *args)
{
	if (gpio_get_level(BASE_MOTOR2_ENCODER_B_GPIO) > 0)
		instantaneous_encoder_count[1]--;
	else
		instantaneous_encoder_count[1]++;
}
void TASK__uart_reception_timeout(void *args)
{
	while(true)
	{
		clock_gettime(CLOCK_REALTIME, &current_time_stamp);
		while((current_time_stamp.tv_sec - ros_node_last_received_message_time_stamp.tv_sec) < 0.200) // 200ms timeout period
		{
			clock_gettime(CLOCK_REALTIME, &current_time_stamp);
			vTaskDelay(pdMS_TO_TICKS(10));
		}

        for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_BASE; _m++)
        {
            base_motor_velocity_command[_m] = 0.0f;
        }

		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}
void TASK__ros_node_data_receiver(void *args)
{
    uart_event_t event;
    uint8_t _rx_data[UART_BUFFER_SIZE];
    
    while(true)
    {
        if(xQueueReceive(uart_queue, &event, portMAX_DELAY))
        {
            if(event.type == UART_DATA)
            {
                clock_gettime(CLOCK_REALTIME, &ros_node_last_received_message_time_stamp);

                int len = uart_read_bytes(UART_PORT, _rx_data, event.size, pdMS_TO_TICKS(5));
                if(len > 0)
                {
                    _rx_data[len] = '\0';
                    
                    cJSON *root = cJSON_Parse((const char *)_rx_data);
                    if(NULL == root)
                    {
                        printf("Error while parsing the JSON data\r\n");
                    }
                    else
                    {
                        const cJSON *rbc = cJSON_GetObjectItem(root, "rbc");
                        if (rbc)
                        {
                            const cJSON *wc = cJSON_GetObjectItem(rbc, "wc");
                            if (wc)
                            {
                                const cJSON *torque = cJSON_GetObjectItem(wc, "t");
                                if(torque)
                                {
                                    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_BASE; _m++)
                                    {
                                        base_motor_torque_command[_m] = cJSON_GetArrayItem(torque, _m)->valuedouble;
                                    }
                                }
                                const cJSON *velocity = cJSON_GetObjectItem(wc, "v");
                                if(velocity)
                                {
                                    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_BASE; _m++)
                                    {
                                        base_motor_velocity_command[_m] = cJSON_GetArrayItem(velocity, _m)->valuedouble;
                                    }
                                }
                                const cJSON *position = cJSON_GetObjectItem(wc, "p");
                                if(position)
                                {
                                    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_BASE; _m++)
                                    {
                                        base_motor_position_command[_m] = cJSON_GetArrayItem(position, _m)->valuedouble;
                                    }
                                }
                            }
                        }

                        const cJSON *rpc = cJSON_GetObjectItem(root, "rpc");
                        if (rpc)
                        {
                            const cJSON *cc = cJSON_GetObjectItem(rpc, "cc");
                            if (cc)
                            {
                                const cJSON *torque = cJSON_GetObjectItem(cc, "t");
                                if(torque)
                                {
                                    for(uint8_t _m=0; _m<NUMBER_OF_MOTORS_AT_PAYLOAD; _m++)
                                    {
                                        payload_motor_torque_command[_m] = cJSON_GetArrayItem(torque, _m)->valuedouble;
                                    }
                                }
                            }
                        }
                    }
                } // END if(len > 0)
            } // END if(event.type == UART_DATA)
        } // END if(xQueueReceive(uart_queue, &event, portMAX_DELAY))
    } // END while(true)
}
void TIMER__ros_node_data_sender_callback(TimerHandle_t xTimer)
{
    // The sender is always going to work even when there is no data received from the ROS node.
    // This gives a periodicity to the motor control.
    motor_control();

    // Obtain all the IMU data.
    bmx160.getAllData(&mag_uT, &gyro_DPS, &accel_G);

    // add additional lines for obtaining the distance sensor readings, and the limit switch state.

    // package the data into JSON format
    char _to_ros_node[UART_BUFFER_SIZE] = {'\0'};
    sprintf
    (
        _to_ros_node,
        "%c{\"rbf\":{\"wf\":{\"t\":[%i,%i],\"v\":[%.2f,%.2f],\"p\":[%lli,%lli]},\"imu\":{\"a\":[%f,%f,%f],\"g\":[%f,%f,%f],\"m\":[%f,%f,%f]}},\"rpf\":{\"ls\":[%d,%d],\"ds\":%.1f}}%c",
        PACKET_START_CHARACTER,
        base_motor_duty_cycle[0], base_motor_duty_cycle[1],
        base_motor_velocity_feedback[0], base_motor_velocity_feedback[1],
        (new_encoder_count[0] % ENCODER_PPR), (new_encoder_count[1] % ENCODER_PPR),
        (accel_G.x * 9.80665f), (accel_G.y * 9.80665f), (accel_G.z * 9.80665f), // meters per second^2
        (gyro_DPS.x * DEGREES_TO_RADIANS), (gyro_DPS.y * DEGREES_TO_RADIANS), (gyro_DPS.z * DEGREES_TO_RADIANS), // radians per second
        (mag_uT.x * 0.001f), (mag_uT.y * 0.001f), (mag_uT.z * 0.001f), // nano-Tesla
        payload_limit_switch_status[0], payload_limit_switch_status[1],
        payload_distance_sensor_reading[0],
        PACKET_END_CHARACTER
    );
    uart_write_bytes(UART_PORT, &_to_ros_node[0], strlen(_to_ros_node));
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
	motor1_channel.gpio_num   = BASE_MOTOR1_PWM_GPIO;
	motor1_channel.speed_mode = PWM_HS_MODE;
	motor1_channel.channel    = BASE_MOTOR1_PWM_CHANNEL;
	motor1_channel.intr_type  = LEDC_INTR_DISABLE;
	motor1_channel.timer_sel  = PWM_HS_TIMER;
	motor1_channel.duty       = 0;
	motor1_channel.hpoint     = 0;
	motor1_channel.flags.output_invert = 0;
	ledc_channel_config(&motor1_channel);
	// Channel 2 //
	ledc_channel_config_t motor2_channel;
	motor2_channel.gpio_num   = BASE_MOTOR2_PWM_GPIO;
	motor2_channel.speed_mode = PWM_HS_MODE;
	motor2_channel.channel    = BASE_MOTOR2_PWM_CHANNEL;
	motor2_channel.intr_type  = LEDC_INTR_DISABLE;
	motor2_channel.timer_sel  = PWM_HS_TIMER;
	motor2_channel.duty       = 0;
	motor2_channel.hpoint     = 0;
	motor2_channel.flags.output_invert = 0;
	ledc_channel_config(&motor2_channel);
	/////////////////// Fade configuration ///////////////////
	ledc_fade_func_install(0);
	Set_Inverted_PWM(PWM_HS_MODE, BASE_MOTOR1_PWM_CHANNEL, 0.0);
	Set_Inverted_PWM(PWM_HS_MODE, BASE_MOTOR2_PWM_CHANNEL, 0.0);
	ESP_LOGI("IO_config", "PWM - check");
}
void Setup_INTR()
{
	gpio_config_t Interrupt_Pins_Config = (gpio_config_t) // encA of motor 1 and 2
	{
		.pin_bit_mask = GPIO_INTERRUPT_INPUT_PINS_MASK,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	gpio_config(&Interrupt_Pins_Config);

	gpio_set_intr_type(BASE_MOTOR1_ENCODER_A_GPIO, GPIO_INTR_POSEDGE); // don't know why this reduntant line has to be mentioned, but it all fails without it.
	gpio_set_intr_type(BASE_MOTOR2_ENCODER_A_GPIO, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);													  //install gpio isr service
	gpio_isr_handler_add(BASE_MOTOR1_ENCODER_A_GPIO, Motor1_Encoder_ISR_Handler, (void *)BASE_MOTOR1_ENCODER_A_GPIO); //Attach ISR handler to encA pin of motor 1 and 2
	gpio_isr_handler_add(BASE_MOTOR2_ENCODER_A_GPIO, Motor2_Encoder_ISR_Handler, (void *)BASE_MOTOR2_ENCODER_A_GPIO);
}
void Setup_Digital_IO()
{
	////////////////////////////////// DIGITAL INPUTS //////////////////////////////////
	gpio_config_t Digital_Input_Pins_Config = (gpio_config_t) // encB of motor 1 and 2
	{
		.pin_bit_mask = GPIO_DIGITAL_INPUT_PINS_MASK,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&Digital_Input_Pins_Config);
	ESP_LOGI("IO_config", "Digital Inputs - check");

	////////////////////////////////// DIGITAL OUTPUTS //////////////////////////////////
	gpio_config_t Digital_Output_Pins_Config = (gpio_config_t) // motor 1 and 2 direction control
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
void Setup_IMU()
{
	ESP_LOGI("IMU_config", "Setting up BMX160 via I2C");
	while(bmx160.begin() != true)
	{
		ESP_LOGE("IMU_config", "begin failed!");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	ESP_LOGI("IMU_config", "connection successful!");
}
void Setup_UART()
{
	uart_config_t uart_config;
    uart_config.baud_rate = UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, -1, -1, -1, -1);
    uart_driver_install(UART_PORT, (UART_BUFFER_SIZE*2), (UART_BUFFER_SIZE*2), 2, &uart_queue, 0);
}
void Setup_Timers()
{
    uart_sender_timer = xTimerCreate("ROS node data sender", pdMS_TO_TICKS(UART_SENDER_PERIOD), pdTRUE, NULL, TIMER__ros_node_data_sender_callback);
    xTimerStart(uart_sender_timer, 0);
}

extern "C" void app_main(void)
{
	Setup_PWM();
	Setup_INTR();
	Setup_Digital_IO();
	Setup_IMU();
    Setup_UART();
    Setup_Timers();
	
    xTaskCreate(TASK__uart_reception_timeout, "UART reception timeout", 2048, NULL, 4, NULL);
    xTaskCreate(TASK__ros_node_data_receiver, "ROS data receiver", 8192, NULL, 5, NULL);
}
