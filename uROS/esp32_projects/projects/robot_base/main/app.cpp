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
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "DFRobot_BMX160.h"
//////////////////////////////
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

////////////////////////////////////////////// ROS_DOMAIN_ID //////////////////////////////////////////////
#define ROS_DOMAIN_ID 1
///////////////////////////////////////////////// SENSORS /////////////////////////////////////////////////
#define MOTOR1_ENCODER_A_GPIO GPIO_NUM_34		// Digital Input | Quadrature
#define MOTOR1_ENCODER_B_GPIO GPIO_NUM_35		// Digital Input | Quadrature
#define MOTOR2_ENCODER_A_GPIO GPIO_NUM_39		// Digital Input | Quadrature
#define MOTOR2_ENCODER_B_GPIO GPIO_NUM_36		// Digital Input | Quadrature
#define MOTOR1_SHUNT_P_GPIO ADC2_CHANNEL_7		// GPIO_NUM_27 | Analog Input | R001 (1mOhm) shunt
#define MOTOR2_SHUNT_P_GPIO ADC1_CHANNEL_5		// GPIO_NUM_33 | Analog Input | R001 (1mOhm) shunt
#define I2C_SDA GPIO_NUM_21						// Digital IO (I2C)
#define I2C_SCL GPIO_NUM_22						// Digital IO (I2C)
#define I2C_CLOCK_SPEED 400000					// Digital IO (I2C)
#define VOLTAGE_SENSOR_GPIO ADC2_CHANNEL_3		// GPIO_NUM_15 | Analog Input | Voltage divider
//////////////////////////////////////////////// ACTUATORS ////////////////////////////////////////////////
#define MOTOR2_A_PWM_GPIO GPIO_NUM_25			// Analog Output (PWM)
#define MOTOR2_B_PWM_GPIO GPIO_NUM_32			// Analog Output (PWM)
#define MOTOR1_A_PWM_GPIO GPIO_NUM_12			// Analog Output (PWM)
#define MOTOR1_B_PWM_GPIO GPIO_NUM_26			// Analog Output (PWM)
#define MOTOR_DRIVER_DISABLE GPIO_NUM_5			// Digital Output | Active LOW
#define RGB_LED_R GPIO_NUM_19					// Analog Output (PWM)
#define RGB_LED_G GPIO_NUM_18					// Analog Output (PWM)
#define RGB_LED_B GPIO_NUM_17					// Analog Output (PWM)
///////////////////////////////////////////////// SERIAL /////////////////////////////////////////////////
#define UART1_TX GPIO_NUM_13					// Digital IO (Serial)
#define UART1_RX GPIO_NUM_14					// Digital IO (Serial)
#define CAN_TX_GPIO GPIO_NUM_4						// Digital IO (Serial)
#define CAN_RX_GPIO GPIO_NUM_2						// Digital IO (Serial)

#define Pi 3.141592653589793238
#define NUMBER_OF_MOTORS 2
#define GPIO_DIGITAL_INPUT_PINS_MASK ((1ULL << MOTOR1_ENCODER_B_GPIO) | (1ULL << MOTOR2_ENCODER_B_GPIO))
#define GPIO_DIGITAL_OUTPUT_PINS_MASK ((1ULL << MOTOR_DRIVER_DISABLE))
#define GPIO_INTERRUPT_INPUT_PINS_MASK ((1ULL << MOTOR1_ENCODER_A_GPIO) | (1ULL << MOTOR2_ENCODER_A_GPIO))
#define IMU_DAQ_Period 0.010 // milliseconds
#define ESP_INTR_FLAG_DEFAULT 0
#define RING(x, min, max) ((x < min) ? max : (x > max) ? min : x)
#define LIMIT(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SUPPRESS(x, max) ((x > max) ? max : x)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); } }
#define STRING_BUFFER_LEN 20
#define PWM_HS_TIMER LEDC_TIMER_0
#define PWM_HS_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR1_A_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR1_B_PWM_CHANNEL LEDC_CHANNEL_1
#define MOTOR2_A_PWM_CHANNEL LEDC_CHANNEL_2
#define MOTOR2_B_PWM_CHANNEL LEDC_CHANNEL_3
#define PWM_RESOLUTION_BITS LEDC_TIMER_11_BIT
#define PWM_RESOLUTION pow(2, (float)PWM_RESOLUTION_BITS)
#define PWM_FREQUENCY 20000 // Should be lesser than 80MHz/(2^PWM_RESOLUTION)
#define ADC_CALIBRATION_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF
#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define ADC_RESOLUTION_BIT ADC_WIDTH_BIT_12

int64_t Motor1_Encoder_Value = 0, Motor2_Encoder_Value = 0;
float Motor_Duty_Cycle[NUMBER_OF_MOTORS];
static const char* TAG = "IMU";
DFRobot_BMX160 bmx160;
sBmx160SensorData_t mag_uT, gyro_DPS, accel_G;
static esp_adc_cal_characteristics_t motor1_shunt_p_adc_chars;
static esp_adc_cal_characteristics_t motor2_shunt_p_adc_chars;
static esp_adc_cal_characteristics_t voltage_sensor_adc_chars;

rcl_publisher_t encoder_raw_publisher;
rcl_publisher_t imu_raw_publisher;
rcl_publisher_t mag_raw_publisher;
rcl_subscription_t wheel_speed_subscriber;
std_msgs__msg__Int64MultiArray encoder_raw;
sensor_msgs__msg__Imu imu_raw;
sensor_msgs__msg__MagneticField mag_raw;
sensor_msgs__msg__JointState wheel_speed;
struct timespec wheel_speed_message_time_stamp;
struct timespec current_time_stamp;

void Set_Inverted_PWM(ledc_mode_t _mode, ledc_channel_t _channel, float _percent_duty)
{
	uint32_t _duty = uint32_t((PWM_RESOLUTION - 1) * 1.0/100 * (100 - _percent_duty));
    ledc_set_duty_and_update(_mode, _channel, _duty, 0);
}
void Set_Motor_Speed()
{
	float _left_motor_percent_duty_cycle = LIMIT(Motor_Duty_Cycle[0], -100, 100);
	float _right_motor_percent_duty_cycle = LIMIT(Motor_Duty_Cycle[1], -100, 100);

	if(_left_motor_percent_duty_cycle >= 0)
	{ // CCW
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_A_PWM_CHANNEL, abs(_left_motor_percent_duty_cycle));
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_B_PWM_CHANNEL, 0);
	}
	else
	{ // CW
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_A_PWM_CHANNEL, 0);
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_B_PWM_CHANNEL, abs(_left_motor_percent_duty_cycle));
	}
	if(_right_motor_percent_duty_cycle >= 0)
	{ // CCW
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_A_PWM_CHANNEL, abs(_right_motor_percent_duty_cycle));
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_B_PWM_CHANNEL, 0);
	}
	else
	{ // CW
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_A_PWM_CHANNEL, 0);
		Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_B_PWM_CHANNEL, abs(_right_motor_percent_duty_cycle));
	}
}
void ROS2_wheel_speed_Subscription_Callback(const void *_message_in)
{
	// ros2 topic pub --once /amr/wheel_speed sensor_msgs/msg/JointState "{effort: [-10.5, 10.5]}"
	clock_gettime(CLOCK_REALTIME, &wheel_speed_message_time_stamp);
	sensor_msgs__msg__JointState *wheel_speed_message = (sensor_msgs__msg__JointState *) _message_in;
	Motor_Duty_Cycle[0] = (float)wheel_speed_message->effort.data[0]; // expect between -100.0% to +100.0%
	Motor_Duty_Cycle[1] = (float)wheel_speed_message->effort.data[1];

	Set_Motor_Speed();
}
void ROS2_encoder_raw_Publisher_Callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		encoder_raw.data.data[0] = Motor1_Encoder_Value;
		encoder_raw.data.data[1] = Motor2_Encoder_Value;

		RCSOFTCHECK(rcl_publish(&encoder_raw_publisher, (const void *)&encoder_raw, NULL));
	}
}
void ROS2_imu_raw_Publisher_Callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		imu_raw.header.stamp.sec = ts.tv_sec;
		imu_raw.header.stamp.nanosec = ts.tv_nsec;

		imu_raw.linear_acceleration.x = accel_G.x * 9.80665; // send in meters/sec^2
		imu_raw.linear_acceleration.y = accel_G.y * 9.80665;
		imu_raw.linear_acceleration.z = accel_G.z * 9.80665;
		imu_raw.angular_velocity.x = gyro_DPS.x * (M_PI/180); // send in radians/second
		imu_raw.angular_velocity.y = gyro_DPS.y * (M_PI/180);
		imu_raw.angular_velocity.z = gyro_DPS.z * (M_PI/180);
		
		RCSOFTCHECK(rcl_publish(&imu_raw_publisher, (const void *)&imu_raw, NULL));
	}
}
void ROS2_mag_raw_Publisher_Callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		mag_raw.header.stamp.sec = ts.tv_sec;
		mag_raw.header.stamp.nanosec = ts.tv_nsec;

		mag_raw.magnetic_field.x = mag_uT.x / 1000; // send in Tesla
		mag_raw.magnetic_field.y = mag_uT.y / 1000;
		mag_raw.magnetic_field.z = mag_uT.z / 1000;
		
		RCSOFTCHECK(rcl_publish(&mag_raw_publisher, (const void *)&mag_raw, NULL));
	}
}
void TASK_microROS(void *args)
{
	// Galactic: Node wih options...
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	// Initialize and modify options (Set ROS_DOMAIN_ID)
	RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));

	// Initialize rclc support object with custom options
	rclc_support_t support;
	while(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK)
	{
		rclc_support_fini(&support);
	}

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "AMR_MicroROS_Node_v2", "", &support));

	// create publishers
	RCCHECK(rclc_publisher_init_best_effort(&encoder_raw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), "/amr/encoder_raw"));
	RCCHECK(rclc_publisher_init_default(&imu_raw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw"));
	RCCHECK(rclc_publisher_init_default(&mag_raw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag"));
	// create subscribers
	RCCHECK(rclc_subscription_init_default(&wheel_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/amr/wheel_speed"));

	// create and initialize the timers for each publisher
	rcl_timer_t encoder_raw_publisher_timer;
	const unsigned int encoder_raw_publish_period = 50; // milliseconds for 20Hz publish rate
	RCCHECK(rclc_timer_init_default(&encoder_raw_publisher_timer, &support, RCL_MS_TO_NS(encoder_raw_publish_period), ROS2_encoder_raw_Publisher_Callback));
	rcl_timer_t imu_raw_publisher_timer;
	const unsigned int imu_raw_publish_period = 10; // milliseconds for 100Hz publish rate
	RCCHECK(rclc_timer_init_default(&imu_raw_publisher_timer, &support, RCL_MS_TO_NS(imu_raw_publish_period), ROS2_imu_raw_Publisher_Callback));
	rcl_timer_t mag_raw_publisher_timer;
	const unsigned int mag_raw_publish_period = 10; // milliseconds for 100Hz publish rate
	RCCHECK(rclc_timer_init_default(&mag_raw_publisher_timer, &support, RCL_MS_TO_NS(mag_raw_publish_period), ROS2_mag_raw_Publisher_Callback));
	
	// create and allocate storage space for incoming and outgoing messages via topics
	// (only if they have a provision for specifying the same)
	encoder_raw.data.capacity = 2; // increase this wrt. the number of encoders being used
	encoder_raw.data.data = (int64_t *)calloc(encoder_raw.data.capacity, sizeof(int64_t));
	encoder_raw.data.size = encoder_raw.data.capacity;
	char frame_id_buffer[STRING_BUFFER_LEN] = "imu_link";
	imu_raw.header.frame_id.data = frame_id_buffer;
	imu_raw.header.frame_id.capacity = STRING_BUFFER_LEN;
	imu_raw.header.frame_id.size = sizeof(imu_raw.header.frame_id.data);
	mag_raw.header.frame_id.data = frame_id_buffer;
	mag_raw.header.frame_id.capacity = STRING_BUFFER_LEN;
	mag_raw.header.frame_id.size = sizeof(mag_raw.header.frame_id.data);
	wheel_speed.effort.capacity = NUMBER_OF_MOTORS;
	wheel_speed.effort.data = (double *)calloc(wheel_speed.effort.capacity, sizeof(double));
	wheel_speed.effort.size = wheel_speed.effort.capacity;

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &encoder_raw_publisher_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &imu_raw_publisher_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &mag_raw_publisher_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_speed_subscriber, &wheel_speed, &ROS2_wheel_speed_Subscription_Callback, ON_NEW_DATA));

	while(rmw_uros_ping_agent(5, 20) == RMW_RET_OK)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
	}
	esp_restart();
}

static void IRAM_ATTR Motor1_Encoder_ISR_Handler(void *arg)
{
	if (gpio_get_level(MOTOR1_ENCODER_B_GPIO) > 0)
		Motor1_Encoder_Value++;
	else
		Motor1_Encoder_Value--;
}
static void IRAM_ATTR Motor2_Encoder_ISR_Handler(void *arg)
{
	if (gpio_get_level(MOTOR2_ENCODER_B_GPIO) > 0)
		Motor2_Encoder_Value++;
	else
		Motor2_Encoder_Value--;
}
void TASK_motor_command_reception_timeout(void *arguments)
{
	while(true)
	{
		clock_gettime(CLOCK_REALTIME, &current_time_stamp);
		while((current_time_stamp.tv_sec - wheel_speed_message_time_stamp.tv_sec) < 0.200) // 200ms timeout period
		{
			clock_gettime(CLOCK_REALTIME, &current_time_stamp);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		Motor_Duty_Cycle[0] = 0;
		Motor_Duty_Cycle[1] = 0;
		Set_Motor_Speed();
		
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}
void TASK_IMU_data_acquisition(void *arg)
{
	ESP_LOGI(TAG, "Fetching raw readings...");
	uint16_t delta_T = int(1000 * IMU_DAQ_Period);
	portTickType lastWakeTime;
	portTickType loopPeriod = pdMS_TO_TICKS(delta_T);
	while(true)
	{
		bmx160.getAllData(&mag_uT, &gyro_DPS, &accel_G);

		// printf("accel_G:%.2f %.2f %.2f \t gyro_DPS:%.2f %.2f %.2f \t mag_uT:%.2f %.2f %.2f \n", accel_G.x, accel_G.y, accel_G.z, gyro_DPS.x, gyro_DPS.y, gyro_DPS.z, mag_uT.x, mag_uT.y, mag_uT.z);

		vTaskDelayUntil(&lastWakeTime, loopPeriod);
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
    };
	ledc_timer_config(&motor_timer);
	/////////////////// Channel configuration ///////////////////
	// Channel 1 //
	ledc_channel_config_t motor1a_channel;
	motor1a_channel.gpio_num   = MOTOR1_A_PWM_GPIO;
	motor1a_channel.speed_mode = PWM_HS_MODE;
	motor1a_channel.channel    = MOTOR1_A_PWM_CHANNEL;
	motor1a_channel.intr_type  = LEDC_INTR_DISABLE;
	motor1a_channel.timer_sel  = PWM_HS_TIMER;
	motor1a_channel.duty       = 0;
	motor1a_channel.hpoint     = 0;
	motor1a_channel.flags.output_invert = 0;
	ledc_channel_config(&motor1a_channel);
	// Channel 2 //
	ledc_channel_config_t motor1b_channel;
	motor1b_channel.gpio_num   = MOTOR1_B_PWM_GPIO;
	motor1b_channel.speed_mode = PWM_HS_MODE;
	motor1b_channel.channel    = MOTOR1_B_PWM_CHANNEL;
	motor1b_channel.intr_type  = LEDC_INTR_DISABLE;
	motor1b_channel.timer_sel  = PWM_HS_TIMER;
	motor1b_channel.duty       = 0;
	motor1b_channel.hpoint     = 0;
	motor1b_channel.flags.output_invert = 0;
	ledc_channel_config(&motor1b_channel);
	// Channel 3 //
	ledc_channel_config_t motor2a_channel;
	motor2a_channel.gpio_num   = MOTOR2_A_PWM_GPIO;
	motor2a_channel.speed_mode = PWM_HS_MODE;
	motor2a_channel.channel    = MOTOR2_A_PWM_CHANNEL;
	motor2a_channel.intr_type  = LEDC_INTR_DISABLE;
	motor2a_channel.timer_sel  = PWM_HS_TIMER;
	motor2a_channel.duty       = 0;
	motor2a_channel.hpoint     = 0;
	motor2a_channel.flags.output_invert = 0;
	ledc_channel_config(&motor2a_channel);
	// Channel 4 //
	ledc_channel_config_t motor2b_channel;
	motor2b_channel.gpio_num   = MOTOR2_B_PWM_GPIO;
	motor2b_channel.speed_mode = PWM_HS_MODE;
	motor2b_channel.channel    = MOTOR2_B_PWM_CHANNEL;
	motor2b_channel.intr_type  = LEDC_INTR_DISABLE;
	motor2b_channel.timer_sel  = PWM_HS_TIMER;
	motor2b_channel.duty       = 0;
	motor2b_channel.hpoint     = 0;
	motor2b_channel.flags.output_invert = 0;
	ledc_channel_config(&motor2b_channel);
	/////////////////// Fade configuration ///////////////////
	ledc_fade_func_install(0);
	Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_A_PWM_CHANNEL, 0.0);
	Set_Inverted_PWM(PWM_HS_MODE, MOTOR1_B_PWM_CHANNEL, 0.0);
	Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_A_PWM_CHANNEL, 0.0);
	Set_Inverted_PWM(PWM_HS_MODE, MOTOR2_B_PWM_CHANNEL, 0.0);
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

	gpio_set_intr_type(MOTOR1_ENCODER_A_GPIO, GPIO_INTR_POSEDGE); // don't know why this reduntant line has to be mentioned, but it all fails without it.
	gpio_set_intr_type(MOTOR2_ENCODER_A_GPIO, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);													  //install gpio isr service
	gpio_isr_handler_add(MOTOR1_ENCODER_A_GPIO, Motor1_Encoder_ISR_Handler, (void *)MOTOR1_ENCODER_A_GPIO); //Attach ISR handler to encA pin of motor 1 and 2
	gpio_isr_handler_add(MOTOR2_ENCODER_A_GPIO, Motor2_Encoder_ISR_Handler, (void *)MOTOR2_ENCODER_A_GPIO);
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
	gpio_config_t Digital_Output_Pins_Config = (gpio_config_t) // encB of motor 1 and 2
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
void Setup_ADC()
{
	////////////////////////////////// ANALOG INPUTS //////////////////////////////////
	esp_adc_cal_check_efuse(ADC_CALIBRATION_SCHEME);
	esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTENUATION, ADC_RESOLUTION_BIT, 0, &motor1_shunt_p_adc_chars);
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTENUATION, ADC_RESOLUTION_BIT, 0, &motor2_shunt_p_adc_chars);
	esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTENUATION, ADC_RESOLUTION_BIT, 0, &voltage_sensor_adc_chars);
	adc2_config_channel_atten(MOTOR1_SHUNT_P_GPIO, ADC_ATTENUATION);
	adc1_config_channel_atten(MOTOR2_SHUNT_P_GPIO, ADC_ATTENUATION);
	adc2_config_channel_atten(VOLTAGE_SENSOR_GPIO, ADC_ATTENUATION);
	ESP_LOGI("IO_config", "Analog Inputs - check");
}
void Setup_IMU()
{
	ESP_LOGI(TAG, "Setting up BMX160 via I2C");
    while(bmx160.begin() != true)
	{
		ESP_LOGE(TAG, "begin failed!");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
    ESP_LOGI(TAG, "connection successful!");
	// bmx160.setLowPower();   //disable the gyroscope and accelerometer sensor
	// bmx160.wakeUp();        //enable the gyroscope and accelerometer sensor
	// bmx160.softReset();     //reset the sensor
	/** 
	 * enum{eGyroRange_2000DPS,
	 *       eGyroRange_1000DPS,
	 *       eGyroRange_500DPS,
	 *       eGyroRange_250DPS,
	 *       eGyroRange_125DPS
	 *       }eGyroRange_t;
	 **/
	// bmx160.setGyroRange(eGyroRange_500DPS); // default 250DPS inside .h
	/*
	*  enum{eAccelRange_2G,
	*       eAccelRange_4G,
	*       eAccelRange_8G,
	*       eAccelRange_16G
	*       }eAccelRange_t;
	*/
	// bmx160.setAccelRange(eAccelRange_4G); // default 2G inside .h
}
void Setup_UART()
{
    // ******************************* UART0 config for microROS transport ******************************* //
    static size_t uart_port = UART_NUM_0;
    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(
			true,
			(void *) &uart_port,
			esp32_serial_open,
			esp32_serial_close,
			esp32_serial_write,
			esp32_serial_read
		);
	#else
		#error micro-ROS transports misconfigured
	#endif

    // ******************************* UART1 config for serial monitoring ******************************* //
	/* Please make sure that the following is set inside menuconfig:
	   1. component_config -> ESP system settings -> channel for console output -> custom UART
	   2. component_config -> ESP system settings -> UART peripheral for console output -> UART1
	   3. component_config -> ESP system settings -> UART TX -> 13 (same as UART1_TX)
	   4. component_config -> ESP system settings -> UART RX -> 14 (same as UART1_RX)
	   5. micro-ROS Settings -> UART Settings -> UART TX -> 1
	   6. micro-ROS Settings -> UART Settings -> UART RX -> 3
	 */
	// ******************************* UART1 config for GPS (optional) ******************************* //
    static size_t uart1_port = UART_NUM_1;
    const unsigned int UART1_BUFFER_SIZE = 1024;
    uart_config_t uart1_config;
    uart1_config.baud_rate = 115200; // for monitoring via uart1
    uart1_config.data_bits = UART_DATA_8_BITS;
    uart1_config.parity    = UART_PARITY_DISABLE;
    uart1_config.stop_bits = UART_STOP_BITS_1;
    uart1_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(uart1_port, &uart1_config);
    uart_set_pin(uart1_port, CONFIG_ESP_CONSOLE_UART_TX_GPIO, CONFIG_ESP_CONSOLE_UART_RX_GPIO, -1, -1);
    uart_driver_install(uart1_port, UART1_BUFFER_SIZE * 2, 0, 0, NULL, 0);
}
void Setup_CAN()
{
	// ******************************* CAN config ******************************* //
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    static const twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = (gpio_num_t) CAN_TX_GPIO,
        .rx_io = (gpio_num_t) CAN_RX_GPIO,
		.clkout_io = (gpio_num_t) -1,
		.bus_off_io = (gpio_num_t) -1,
        .tx_queue_len = 0,
        .rx_queue_len = 65,
        .alerts_enabled = TWAI_ALERT_ALL,
        .clkout_divider = 0,
		.intr_flags = 0
    };
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

extern "C" void app_main(void)
{
	Setup_PWM();
	Setup_INTR();
	Setup_Digital_IO();
	Setup_ADC();
	Setup_UART();
	Setup_CAN();
	// for some reason the microROS TASK fails to initialize from 'node' and onwards, if GPIO of CTS and RTS is configured on the fly.
	// therefore, make sure that EITHER the GPIOs have finished configuring OR the microROS task has initialized completely, once function at a time and not simultaneously.
	xTaskCreatePinnedToCore(TASK_microROS, "microROS task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
	xTaskCreate(TASK_motor_command_reception_timeout, "Check motor command reception timeout", 2048, NULL, 4, NULL);
	Setup_IMU(); // make sure that this function and following task is executed only at the end, because this function won't proceed further unless communication with IMU has established.
 	xTaskCreatePinnedToCore(TASK_IMU_data_acquisition, "IMU raw data acquisition", 2048, NULL, 5, NULL, 0);
}