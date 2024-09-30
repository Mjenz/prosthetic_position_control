// Include necessary drivers and libraries
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>
#include "mpu_6050.h"
#include "ina_219.h"
#include "data_profile.h"

// Define constants
#define STACKSIZE      1024
#define PRIORITY       7
#define WORKQ_PRIORITY 7
#define INA_WINDOW     1000
#define IMU_WINDOW     1500
#define IMU_AVG_WINDOW 150
#define POS_AVG_WINDOW 50
#define AVG_WINDOW     150
#define SPEED	       PWM_KHZ(20)
#define WINDOW_SIZE    35
#define DELAY	       25
#define WORQ_THREAD_STACK_SIZE 512
#define RPM   68
#define V_NOM 7.4
#define PI    3.14159265358979323846
#define KV    RPM / V_NOM
#define KV_SI (KV * ((2 * PI) / 60)) // radians/second/volt
#define KT    (1 / KV_SI)
#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

// makes sure there is a devicetree overlay
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

// initialize pwm
static const struct pwm_dt_spec pwm_28 = PWM_DT_SPEC_GET(DT_NODELABEL(pin28));
static uint32_t min_pulse_28 = DT_PROP(DT_NODELABEL(pin28), min_pulse);
static const struct pwm_dt_spec pwm_29 = PWM_DT_SPEC_GET(DT_NODELABEL(pin29));
static uint32_t min_pulse_29 = DT_PROP(DT_NODELABEL(pin29), min_pulse);

// initialize global vars
struct mpu_6050_data mpu_data;				// imu data, passed to mpu functions
struct ina_219_data ina_data;				// current data, passed to outside functions
const struct device *mpu_dev, *ina_dev;		// device variables
volatile static float angle = 130.0;		// setpoint variable in position PID control
volatile static float commanded_current = 0.0;	// setpoint variable in current PID control
volatile static float accel_profile;		// accel value used for direciton selection
volatile static float actual_radians;		// actual angle used in position PID control
static float point_arr[DELAY];				// array used to filter IMU data during step detection
unsigned int pwm_val;						// global variable for setting pwm 
int dir_flag = 1;							// indicates the direction of the motor based on u values
bool motion_detect = false;					// global var to signal between position control and motor control threads
int ret;									// dummy variable for zephyr required initialization
float threshold = 12.5;	    				// determined experimentally, for step detection
volatile float time_between_steps = 0.0;	// time measured between steps
volatile float cadence = 0.0;				// inverse of time_between_steps
struct adc_sequence sequence;				// struct for reading adc
uint16_t buf;								// buffer variable for functions
float filteredEncoderReading = 0.0;			// encoder value, global for use by position control

// PID coefficients for single trajectory runs, and init for variable speed runs
static volatile float Kp = 0.005, Ki = 0.0, Kd = 0.0, Kff = 0.075; // works well!
static volatile float Kp_pos = 0, Ki_pos = 0, Kd_pos = 0, Kff1_pos = 0, Kff2_pos = 0; 

// array of PID coefficents for different REAL_cropped trajectories
float Kp_pos_array[4] = {2.025, 1.7, 1.3, 1.1};
float Ki_pos_array[4] = {0.0575, 0.0575, 0.05, 0.05};
float Kd_pos_array[4] = {0.4, 0.4, 0.4, 0.03};
float Kff1_pos_array[4] = {0.5, 0.5, 0.4, 0.5};
float Kff2_pos_array[4] = {0, 0, 0, 0};

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

// Define mutex for sensor readings
K_FIFO_DEFINE(printk_fifo);
K_SEM_DEFINE(start_sem, 0, 1);
K_MUTEX_DEFINE(mpu_sensor_mutex);
K_MUTEX_DEFINE(ina_sensor_mutex);
K_MUTEX_DEFINE(pot_adc_mutex);
K_SEM_DEFINE(output_sem, 0, 1);
K_MUTEX_DEFINE(angle_mutex);
K_MUTEX_DEFINE(current_mutex);
K_MUTEX_DEFINE(step_mutex);

// Define timers for control loops
K_TIMER_DEFINE(current_timer, NULL, NULL);
K_TIMER_DEFINE(position_timer, NULL, NULL);
K_TIMER_DEFINE(imu_timer, NULL, NULL);
K_TIMER_DEFINE(motor_control_timer, NULL, NULL);
K_TIMER_DEFINE(pot_adc_timer, NULL, NULL);

// Define queue
static K_THREAD_STACK_DEFINE(imu_q_stack_area, WORQ_THREAD_STACK_SIZE);
static struct k_work_q imu_work_q = {0};

// Create queue struct
struct imu_data {
	struct k_work work;
	float data;
	int count;
} my_data;

// Helper functions

// calculate mean value of array
float mean_val(float arr[], int len)
{
	float sum = 0.0, mean = 0.0;

	for (int i = 0; i < len; ++i) {
		sum += arr[i];
	}

	mean = sum / len;
	return mean;
}

// moving average, take in new item to array
float moving_average(float *buf, int size, float new_value)
{
	float sum = 0;
	for (int i = 0; i < size - 1; i++) {
		buf[i] = buf[i + 1];
		sum += buf[i];
	}
	buf[size - 1] = new_value;
	sum += new_value;

	return sum / size;
}

// put motors at stationary (min pulse)
int base_swing(void)
{
	ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width for pin 28\n", ret);
	}

	k_msleep(700);

	ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width for pin 28\n", ret);
	}

	k_msleep(50);

	ret = pwm_set_pulse_dt(&pwm_29, min_pulse_28);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width for pin 28\n", ret);
	}

	k_msleep(700);

	ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
	if (ret < 0) {
		printk("Error %d: failed to set pulse width for pin 28\n", ret);
	}

	k_msleep(50);

	return ret;
}

// activate motors, run commanded pwm
int controlled_swing(void)
{

	if (dir_flag == -1) {
		ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
		if (ret < 0) {
			printk("Error %d: failed to set 0 pulse width for 28\n", ret);
		}

		// k_msleep(50);

		ret = pwm_set_pulse_dt(&pwm_29, pwm_val);
		if (ret < 0) {
			printk("Error %d: failed to set pulse %d width for pin 29\n", ret, pwm_val);
		}

	} else if (dir_flag == 1) {
		ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
		if (ret < 0) {
			printk("Error %d: failed to set 0 pulse width for 29\n", ret);
		}

		// k_msleep(50);

		// /// 28 CLOCKWISE (+) CURRENT
		ret = pwm_set_pulse_dt(&pwm_28, pwm_val);
		if (ret < 0) {
			printk("Error %d: failed to set pulse %d width for pin 28\n", ret, pwm_val);
		}
	} else {
		ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
		if (ret < 0) {
			printf("error %d: failed to set 0 pulse width\n", ret);
		}
		// k_msleep(50);

		ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
		if (ret < 0) {
			printf("error %d: failed to set 0 pulse width\n", ret);
		}
		// k_msleep(50);
	}

	return ret;
}

// initialize sensors
const struct device *sensors_init(const char *sensor_compat)
{

	const struct device *dev = NULL;

	if (strcmp(sensor_compat, "mpu") == 0) {
		dev = DEVICE_DT_GET_ONE(MPU_COMPAT);
	} else if (strcmp(sensor_compat, "ina") == 0) {
		dev = DEVICE_DT_GET_ONE(INA_COMPAT);
	}

	if (dev != NULL) {
		printk("\nDevice found on Teensy DTS\n");
	}

	if (device_is_ready(dev)) {
		printk("\nDevice %s is ready\n", dev->name);
	} else {
		printf("Device %s is not ready\n", dev->name);
	}

	return dev;
}

// workque callback function defenition, step/cadence detection
void cadence_detection(struct k_work *item)
{

	// init
	static int loc = 0; 			// number of data points since last peak
	static float avg;				// average of IMU values, to calculate peaks
	static bool ready = false;		// flag to indicate ready for next step

	// get the data from the queue
	struct imu_data *point = CONTAINER_OF(item, struct imu_data, work);
	float sample = point->data;
	int count = point->count;

	// save data
	for (int ii = 0; ii < DELAY - 1; ii++) {
		point_arr[ii] = point_arr[ii + 1]; // shift
	}

	// it must pass below a threshhold to signal the next step
	point_arr[DELAY - 1] = sample;
	if (point_arr[2] < 9) {
		ready = true;
	}

	// Step detection
	if (point_arr[1] > point_arr[0] && point_arr[2] < point_arr[1] && ready == true) {
		// Filter out bad steps (1/5 of a second apart, mistakes not running)
		if (count > (loc + 120)) {
			// filter out noise
			avg = mean_val(point_arr, 3);
			// if imu magnitude is big enough, it is a step
			if (avg > threshold) {
				k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);
				time_between_steps = (float)(count - loc) / 610;
				cadence = 1 / time_between_steps;
				k_mutex_unlock(&mpu_sensor_mutex);
				printk("s%f\n", cadence);	
				loc = count;
				ready = false; // reset trigger bool
			}
		}
	}
}

/// Thread Definitions

// read imu value
void mpu_sensor_read(void)
{
	const char *sensor_name = "mpu";
	mpu_dev = sensors_init(sensor_name);
	float accel_mag, accel_y;
	float smooth_data = 0.0;
	int counter = 0;

	// initialize work queue
	k_work_queue_init(&imu_work_q);
	k_work_queue_start(&imu_work_q, imu_q_stack_area, K_THREAD_STACK_SIZEOF(imu_q_stack_area),
			   WORKQ_PRIORITY,
			   NULL);
	k_work_init(&my_data, cadence_detection); // init work item

	k_sleep(K_SECONDS(2));

	while (1) {

		// start timer
		k_timer_start(&imu_timer, K_USEC(1000), K_NO_WAIT);

		int rc = process_mpu6050(mpu_dev, &mpu_data);
		if (rc != 0) {
			break;
		}

		// collect the imu values
		k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);
		accel_y = sensor_value_to_double(&mpu_data.accelerometer[1]);
		k_mutex_unlock(&mpu_sensor_mutex);
		accel_mag = sqrt(pow(accel_y, 2));

		// mov_avg = moving_average(Imu_Avg_Buf,IMU_AVG_WINDOW,accel_mag);
		smooth_data = smooth_data - (.7 * (smooth_data - (accel_mag)));

		// save data to struct
		my_data.data = smooth_data;
		my_data.count = counter;

		// submit to work queue
		k_work_submit_to_queue(&imu_work_q, &my_data.work);

		// print
		printk("m%f\n",smooth_data);

		// increment counter
		counter++;

		// check timing, wait if faster than 1ms
		k_timer_status_sync(&imu_timer);
	}
}

// read current sensor
void ina_sensor_read(void)
{
	// Runs at 200 Hz
	const char *sensor_name = "ina";
	ina_dev = sensors_init(sensor_name);

	while (1) {

		// printk("ina\n");

		// use helper function in ina_219.c
		int rc = process_ina219(ina_dev, &ina_data);

		if (rc != 0) {
			break;
		}
	}
}

// call functions to set motor
void motor_control(void)
{
	int ret;
	if (!device_is_ready(pwm_28.dev)) {
		printf("Error: PWM device %s is not ready\n", pwm_28.dev->name);
	}

	if (!device_is_ready(pwm_29.dev)) {
		printf("Error: PWM device %s is not ready\n", pwm_29.dev->name);
	}

	ret = pwm_set_pulse_dt(&pwm_29, min_pulse_29);
	if (ret < 0) {
		printf("Error %d: failed to set pulse width\n", ret);
	}

	ret = pwm_set_pulse_dt(&pwm_28, min_pulse_28);
	if (ret < 0) {
		printf("Error %d: failed to set pulse width\n", ret);
	}

	int res;
	while (1) {
		// start timer
		k_timer_start(&motor_control_timer, K_MSEC(1), K_NO_WAIT);

		// check if there was motion detected
		if (!motion_detect) {
			res = base_swing();
			if (res < 0.0) {
				break;
			}
		} else {
			res = controlled_swing();
			if (res < 0.0) {
				break;
			}
		}

		// check if timer is expired, wait if necessary
		k_timer_status_sync(&motor_control_timer);
	}
}

// read adc for encoder reading
void read_pot_adc(void)
{
	int err;

	sequence.buffer = &buf;
	sequence.buffer_size = sizeof(buf);

	float rawBuffer[WINDOW_SIZE] = {0};
	int currentIndex = 0;

	k_sleep(K_SECONDS(4));

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
		}
	}

	while (1) {
		// start timer
		k_timer_start(&pot_adc_timer, K_MSEC(1), K_NO_WAIT);

		(void)adc_sequence_init_dt(&adc_channels[0], &sequence);

		k_mutex_lock(&pot_adc_mutex, K_FOREVER);

		err = adc_read(adc_channels[0].dev, &sequence);
		if (err < 0) {
			printk("Could not read (%d)\n", err);
			continue;
		}
		k_mutex_unlock(&pot_adc_mutex);

		float sum = 0;

		rawBuffer[currentIndex] = buf;
		currentIndex = (currentIndex + 1) % WINDOW_SIZE;

		for (int i = 0; i < WINDOW_SIZE; ++i) {
			sum += rawBuffer[i];
		}

		filteredEncoderReading = sum / WINDOW_SIZE;

		// check if timer is expired, wait if necessary
		k_timer_status_sync(&pot_adc_timer);
	}
}

// current control PID loop
void current_control(void)
{
	// init control variables
	float eint = 0, ed = 0, e_prev = 0;
	float p_control = 0, int_control = 0, der_control = 0, der_filtered = 0, der_prev = 0, uff;
	float dt = 0.001;

	// init filter and anti windup parameters
	float der_filter_coef = 0.5;
	float anti_windup_max = 10.0;

	// encoder moving average window size
	int window_size = 100;

	// moving average buffer
	float adc_buffer[AVG_WINDOW] = {0};

	k_sleep(K_SECONDS(5));
	while (1) {
		// start timer
		k_timer_start(&current_timer, K_USEC(1000), K_NO_WAIT);

		// lock command current mutex
		k_mutex_lock(&current_mutex, K_FOREVER);

		// init current setpoint from pos control
		float ref_current = commanded_current * 1000.0;

		// unlock command current mutex
		k_mutex_unlock(&current_mutex);

		// lock current sensor mutex
		k_mutex_lock(&ina_sensor_mutex, K_FOREVER);

		// get actual current value
		float adcval = sensor_value_to_double(&ina_data.current) * 1000.0;

		// unlock current sensor mutex
		k_mutex_unlock(&ina_sensor_mutex);

		// filter current with MAV
		float average = moving_average(adc_buffer, window_size, adcval);

		// feedback to python script
		printk("c%f %f\n", ref_current, average);

		// calculate error
		float error = (ref_current - average);

		// proportional term
		p_control = Kp * error;

		// integral calculation
		eint += error * dt;

		// clamping anti-windup for integral term
		if (eint > anti_windup_max) {
			eint = anti_windup_max;
		} else if (eint < -anti_windup_max) {
			eint = -anti_windup_max;
		}

		// integral term
		int_control = Ki * eint;

		// derivative term
		ed = (error - e_prev) / dt;

		// filter derivative term with EMA filter
		der_filtered = der_filter_coef * der_prev + (1.0 - der_filter_coef) * ed; 
		der_control = Kd * der_filtered;

		// save for next derivative calculation
		der_prev = der_control;
		e_prev = error;

		// calculate feed forward term
		uff = ref_current * Kff;

		// calculate u
		float u = p_control + der_control + int_control + uff;

		// set direction of motor based on u value
		if (u > 0.0) {
			dir_flag = 1;
		} else {
			dir_flag = -1;
		}

		// set pwm command
		if (u >= 0.0) {
			pwm_val = (int)(u / 100.0 * SPEED);

		} else {
			pwm_val = (int)(-u / 100.0 * SPEED);
		}

		// ensure timer has expired, wait if necessary
		k_timer_status_sync(&current_timer);
	}
}

// // // Position control, no step detection, encoder filter active
// void position_control(void)
// {
//     k_sleep(K_SECONDS(5));
//     float error;
//     float ed = 0, e_prev = 0.0, eint = 0.0, D_filtered, D_prev = 0.0;
//     float P, I, D;
//     float anti_windup_max = 0.5;
//
//     float encoder, actual_angle;
//     float dt = 0.01;
//
//     float der_filter_coef = 0.8;
//
//     float uff;
//
//     int i = 0;
//
//     float offset = 0.0;
//
//     // int data_length = DATA_SAMPLES;
//     // int data_length = DATA_SAMPLES_REAL_7;
//     int data_length = DATA_SAMPLES_REAL_9;
//     // int data_length = DATA_SAMPLES_REAL_11;
//     // int data_length = DATA_SAMPLES_REAL_13;
//
//     int how_many_traj = 4;
//
//     float top = 2.0;
//     float bottom = 1.10;
//     float dec = (top-bottom)/(float)how_many_traj;
//     float high = top;
//     float low = high-dec;
//
//     int traj_num = -1;
//     float pos_filtered = 0.0;
//     // float alpha = .365;
//     float alpha = .3;
//
//     bool first = 1;
//
//     float previous_angle = 0;
//     float prev_rad = 0, current_rad = 0, next_rad = 0;
//
//     // float pos_avg_arr[POS_AVG_WINDOW] ={0};
//     // float pos_average = 0;
//
//     while(1)
//     {
//         // start timer
//         k_timer_start(&position_timer, K_MSEC(10), K_NO_WAIT);
//
//         k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);
//         float time = time_between_steps;
//         float cad = cadence;
//         k_mutex_unlock(&mpu_sensor_mutex);
//
//         // check if they are moving
//         if (cad != 0.0) {
//             // Tell motor_control to run controlled swing
//             motion_detect = true;
//
//             k_mutex_lock(&angle_mutex,K_FOREVER);
//                 // angle = Angle_Profile_STEP[i];
//                 // accel_profile = Accel_Profile_STEP[i];
//
//                 // angle = Angle_Profile_SIN[i];
//                 // accel_profile = Accel_Profile_SIN[i];
//
//                 // angle = Angle_Profile_REAL_7[i];
//                 // accel_profile = Angle_Profile_REAL_7[i]; // not accel profile but signs are
//                 the same
//
//                 angle = Angle_Profile_REAL_9[i];
//                 accel_profile = Angle_Profile_REAL_9[i]; // not accel profile but signs are the
//                 same
//
//                 // angle = Angle_Profile_REAL_11[i];
//                 // accel_profile = Angle_Profile_REAL_11[i]; // not accel profile but signs are
//                 the same
//
//                 // angle = Angle_Profile_REAL_13[i];
//                 // accel_profile = Angle_Profile_REAL_13[i]; // not accel profile but signs are
//                 the same
//
//             k_mutex_unlock(&angle_mutex);
//
//             i++; // increment counter
//             if (i == data_length)  {
//                 i = 0; // reset counter
//             }
//
//             k_mutex_lock(&pot_adc_mutex, K_FOREVER);
//                 encoder = filteredEncoderReading;
//                 actual_angle = encoder * (220.0/4095.0) - 125.5;//137
//                 actual_radians = actual_angle * PI / 180.0;
//             k_mutex_unlock(&pot_adc_mutex);
//             // filter position data
//             pos_filtered = alpha * actual_radians + (1.0-alpha) * pos_filtered;
//             // prev_rad = pos_filtered;
//             // current_rad = next_rad;
//             // next_rad = actual_radians;
//             // if (first == 1){
//             //     if (prev_rad > current_rad ){
//             //         if (current_rad < next_rad){ // check that it the bottom of a swing
//             //             offset = -current_rad; // use the first reading as the offset
//             //             first = 0;
//             //         }
//             //     }
//
//             // }
//             // pos_filtered = pos_filtered + offset; // offset to account for the offset lol
//             // pos_average = moving_average(pos_avg_arr,POS_AVG_WINDOW,actual_radians);
//             error = (angle - pos_filtered);
//             // error = (angle - actual_radians);
//
//             // Proportional term
//             P = Kp_pos*error;
//
//             // Integral calc
//             eint += error * dt;
//
//             // Clamping anti-windup for integral term
//             if (eint > anti_windup_max){
//                 eint = anti_windup_max;
//             } else if (eint < -anti_windup_max) {
//                 eint = -anti_windup_max;
//             }
//
//             // Integral term
//             I = Ki_pos*eint;
//
//             // Derivative term
//             ed = (error - e_prev) / dt;
//             D_filtered = der_filter_coef * D_prev + (1.0 - der_filter_coef) * ed; // Filter
//             derivative term with EMA filter D = Kd_pos*D_filtered;
//
//             // Save for derivative term calculation
//             e_prev = error;
//             D_prev = D;
//
//             // Calculate feed forward term
//             if(angle > 0){
//                 uff = angle * Kff1_pos;
//             } else {
//                 uff = angle * Kff2_pos;
//             }
//
//             // If the trajectory is decreasing, flip the sign of the ff term
//             if (angle < previous_angle){
//                 uff = -uff;
//             }
//
//             previous_angle = angle; // save the previous position
//
//             // Calculate U
//             float u = P + I + D + uff;
//
//             k_mutex_lock(&current_mutex,K_FOREVER);
//             commanded_current = u;
//             // commanded_current = angle; // for tuning the current control
//             k_mutex_unlock(&current_mutex);
//
//         } else {
//             motion_detect = false;
//         }
//
//         /* * * * * * * * * * * * * * * * * * *
//                     Radian DATA
//         * * * * * * * * * * * * * * * * * * */
//         // printk("a%f %f\n", angle,actual_radians);
//         printk("a%f %f\n", angle,pos_filtered);
//
//         // ensure timer has expired
//         k_timer_status_sync(&position_timer);
//     }
// }

// position control, with step detection
void position_control(void)
{
	k_sleep(K_SECONDS(5));

	// init PID controller vars
	float error;
	float ed = 0, e_prev = 0.0, eint = 0.0, D_filtered, D_prev = 0.0;
	float P, I, D;
	float encoder, actual_angle;
	float dt = 0.01;
	float uff;
	float pos_filtered = 0.0;

	// init filter and anti windup parameters
	float anti_windup_max = 0.5;
	float der_filter_coef = 0.8;
	float alpha = .3;

	// counter variable
	int i = 0;

	// init such that a traj will be selected on first run
	int data_length = 1; 

	int how_many_traj = 4;
	float previous_angle = 0;

    // Variable speed selection highest and lowest cadence values
    float top = 3.0;
	float bottom = 1.8;

	// calculate the decriment value
	float dec = (top - bottom) / (float)how_many_traj;

	// initialize first window bounds
	float high = top;
	float low = high - dec;

	// init with no traj selected
	int traj_num = -1;

	// flag for first time run
	bool first = 1;

	while (1) {
		// start timer
		k_timer_start(&position_timer, K_MSEC(10), K_NO_WAIT);

		// lock imu mutex
		k_mutex_lock(&mpu_sensor_mutex, K_FOREVER);

		// get cadence
		float cad = cadence;

		// unlock imu mutex
		k_mutex_unlock(&mpu_sensor_mutex);

		// lock encoder mutex
		k_mutex_lock(&pot_adc_mutex, K_FOREVER);

		// get arm position
		encoder = filteredEncoderReading;
		actual_angle = encoder * (220.0 / 4095.0) - 135.5; // 145.5 works but is wrong
		actual_radians = actual_angle * PI / 180.0;

		// filter position data
		pos_filtered = alpha * actual_radians + (1.0-alpha) * pos_filtered;

		// lock encoder mutex
		k_mutex_unlock(&pot_adc_mutex);

		// is there motion?
		if (cad != 0.0) {
			// tell motor_control to run controlled swing
			motion_detect = true;

			// increment counter
			i++; 

			// if at the end of traj, reset
			if (i == data_length) {
				i = 0; 
			}

			// if completed traj or first run, select trajectory to follow
			if (i == 0 || first == 1) {
				// find out how fast they are moving, assign profile based on cadence
				for (int j = 0; j < how_many_traj; j++) {
					// set top and bottom boundary
					float high_adjusted = high - (dec * j);
					float low_adjusted = low - (dec * j);
					
					// check if it falls within the range
					if (cad < (high_adjusted) && cad >= (low_adjusted)) {
						 // set traj num
						traj_num = 3 - j;
  
						// get the length of the array
						data_length = profile_lengths_REAL_cropped[traj_num]; 

                        // set the corresponding PID coefficients
						Kp_pos = Kp_pos_array[traj_num];
                        Ki_pos = Ki_pos_array[traj_num];
                        Kd_pos = Kd_pos_array[traj_num];
                        Kff1_pos = Kff1_pos_array[traj_num];
                        Kff2_pos = Kff2_pos_array[traj_num];

						break;
					} else {
						traj_num = -1;
					}
				}

				// no longer first run
				first = 0;
			}

			// if no traj selected, set arm to be stationary
			if (traj_num == -1) {
				motion_detect = 0;

			// if there is a traj selected...
			} else {
				
				// lock angle mutex
				k_mutex_lock(&angle_mutex, K_FOREVER);

				// get angle setpoint
				angle = angle_profiles_REAL_cropped[traj_num][i - 1];
				accel_profile = angle_profiles_REAL_cropped[traj_num][i - 1]; 

				// lock angle mutex
				k_mutex_unlock(&angle_mutex);

				// calculate error
				error = (angle - pos_filtered);
				
				// proportional term
				P = Kp_pos * error;

				// integral calc
				eint += error * dt;

				// clamping anti-windup for integral term
				if (eint > anti_windup_max) {
					eint = anti_windup_max;
				} else if (eint < -anti_windup_max) {
					eint = -anti_windup_max;
				}

				// integral term
				I = Ki_pos * eint;

				// derivative term
				ed = (error - e_prev) / dt;

				// filter derivative term with EMA filter
				D_filtered = der_filter_coef * D_prev + (1.0 - der_filter_coef) * ed; 
				D = Kd_pos * D_filtered;

				// save for derivative term calculation
				e_prev = error;
				D_prev = D;

				/*if the trajectory is decreasing, 
				flip the sign of the ff term and increase ff power for upwards */
				if (angle < previous_angle) {
					uff = -angle * Kff2_pos;
				} else {
					uff = angle * Kff1_pos;
				}
				previous_angle = angle;

				// calculate u
				float u = P + I + D + uff;

				// lock command current mutex
				k_mutex_lock(&current_mutex, K_FOREVER);

				// set command current
				commanded_current = u;

				// lock command current mutex
				k_mutex_unlock(&current_mutex);
			}
		} else {
			motion_detect = false;
		}

		
		// feedback to python script
		printk("a%f %f\n", angle, pos_filtered);

		// ensure timer has expired, wait if not
		k_timer_status_sync(&position_timer);
	}
}

// thread defenitions
K_THREAD_DEFINE(motor_id, STACKSIZE, motor_control, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(ina219_id, STACKSIZE, ina_sensor_read, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(mpu6050_id, STACKSIZE, mpu_sensor_read, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(pot_id, STACKSIZE, read_pot_adc, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(pos_control_id, STACKSIZE, position_control, NULL, NULL, NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(current_control_id, STACKSIZE, current_control, NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
	usb_enable(NULL);
	return 0;
}
