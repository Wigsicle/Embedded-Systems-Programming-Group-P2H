#include "pico/stdlib.h"
#include "stdio.h"
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Motor 1
#define PWM_PIN_1 2
#define DIR_PIN1_1 1
#define DIR_PIN2_1 0
#define ENCODER_PIN_1 3

// Motor 2
#define PWM_PIN_2 5
#define DIR_PIN1_2 6
#define DIR_PIN2_2 4
#define ENCODER_PIN_2 7


// Define GPIO pins for both motors
volatile int encoder_count_1 = 0;
volatile int encoder_count_2 = 0;

// Define pins for ultrasonic sensor
#define TRIG_PIN 8
#define ECHO_PIN 9

// PID Constants
const float Kp = 0.1;
const float Ki = 0.01;
const float Kd = 0.005;

// PID Variables
float error_1 = 0, integral_1 = 0, derivative_1 = 0, last_error_1 = 0;
float error_2 = 0, integral_2 = 0, derivative_2 = 0, last_error_2 = 0;

// Desired speed (in counts per loop iteration, arbitrary units)
float desired_speed = 20;

// Function prototypes
void setup_pwm(uint gpio, float freq, float duty_cycle);
void update_motor_control(int motor, float duty_cycle);
void update_pwm_duty_cycle(uint gpio, float duty_cycle);

void control_motor_speed() {
    // Calculate errors
    error_1 = desired_speed - encoder_count_1;
    error_2 = desired_speed - encoder_count_2;

    // Update integrals
    integral_1 += error_1;
    integral_2 += error_2;

    // Calculate derivatives
    derivative_1 = error_1 - last_error_1;
    derivative_2 = error_2 - last_error_2;

    // Compute PID outputs
    float output_1 = Kp*error_1 + Ki*integral_1 + Kd*derivative_1;
    float output_2 = Kp*error_2 + Ki*integral_2 + Kd*derivative_2;

    // Update motor PWM based on PID output
    update_motor_control(1, output_1);
    update_motor_control(2, output_2);

    // Store last errors
    last_error_1 = error_1;
    last_error_2 = error_2;
}

// Constants for ultrasonic sensor
#define MIN_VALID_DISTANCE 2.0
#define MAX_VALID_DISTANCE 400.0
#define MEASUREMENT_COUNT 3
#define FIXED_DELAY_MS 5
#define STOP_DISTANCE_CM 10.0 // Threshold distance to stop forward movement

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile bool measurement_complete = false;
// volatile uint64_t last_time_1 = 0, last_time_2 = 0;
volatile uint64_t pulse_width_1 = 0, pulse_width_2 = 0;

static char event_str[128];

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask; 
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf = '\0';
}


// // Function to calculate wheel speed based on pulse width
// float calculate_wheel_speed(uint64_t pulse_width) {
//     float wheel_circumference_cm = 21.0; // Correct circumference
//     if (pulse_width > 0) {
//         float pulse_duration_seconds = pulse_width / 1000000.0;
//         return (wheel_circumference_cm / pulse_duration_seconds);
//     }
//     return 0.0;
// }

// // Function to calculate total distance traveled based on encoder count
// float calculate_total_distance(int encoder_count) {
//     float wheel_circumference_cm = 21.0;  // The circumference of the wheel
//     int pulses_per_rotation = 20;         // Number of encoder pulses per complete wheel rotation
//     return (wheel_circumference_cm * (encoder_count / (float)pulses_per_rotation));
// }

// Interrupt handler for ultrasonic echo pin
void echo_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        start_time = time_us_32();
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        end_time = time_us_32();
        measurement_complete = true;
    }
}

// Function to trigger the ultrasonic sensor
void trigger_sensor() {
    gpio_put(TRIG_PIN, true);
    sleep_us(10);
    gpio_put(TRIG_PIN, false);
}

// Function to calculate distance from the ultrasonic sensor
float calculate_distance() {
    uint32_t pulse_duration = end_time - start_time;
    float distance = (pulse_duration * 0.0343) / 2;
    if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE) {
        return -1;
    }
    return distance;
}

// Comparator for qsort in median filtering
int compare_floats(const void* a, const void* b) {
    float arg1 = *(const float*)a;
    float arg2 = *(const float*)b;
    return (arg1 > arg2) - (arg1 < arg2);
}

// Function to get median distance from sensor
float get_median_distance() {
    float distances[MEASUREMENT_COUNT];
    int valid_measurements = 0;

    for (int i = 0; i < MEASUREMENT_COUNT; i++) {
        trigger_sensor();
        while (!measurement_complete) {
            tight_loop_contents();
        }
        float distance = calculate_distance();
        if (distance != -1) {
            distances[valid_measurements++] = distance;
        }
        measurement_complete = false;
        sleep_ms(FIXED_DELAY_MS);
    }

    if (valid_measurements == 0) {
        return -1;
    }
    qsort(distances, valid_measurements, sizeof(float), compare_floats);
    return distances[valid_measurements / 2];
}

void gpio_callback(uint gpio, uint32_t events) {
    printf("Callback triggered for GPIO %d with events %u\n", gpio, events);  // Confirm callback execution
    gpio_event_string(event_str, events);

    // Check which GPIO pin triggered the callback and increment the appropriate counter
    if (gpio == ENCODER_PIN_1) {
        encoder_count_1++;
        printf("Encoder 1: GPIO %d %s, Count: %d\n", gpio, event_str, encoder_count_1);
    } else if (gpio == ENCODER_PIN_2) {
        encoder_count_2++;
        printf("Encoder 2: GPIO %d %s, Count: %d\n", gpio, event_str, encoder_count_2);
    }
}


int main() {
    stdio_init_all();

    // Initialize GPIO for Motor 1 and Motor 2
    gpio_init(DIR_PIN1_1);
    gpio_set_dir(DIR_PIN1_1, GPIO_OUT);
    gpio_put(DIR_PIN1_1, 1); // Set direction
    gpio_init(DIR_PIN2_1);
    gpio_set_dir(DIR_PIN2_1, GPIO_OUT);
    gpio_put(DIR_PIN2_1, 0); // Set direction
    setup_pwm(PWM_PIN_1, 100.0f, 0.8f); // Set PWM

    gpio_init(DIR_PIN1_2);
    gpio_set_dir(DIR_PIN1_2, GPIO_OUT);
    gpio_put(DIR_PIN1_2, 1); // Set direction
    gpio_init(DIR_PIN2_2);
    gpio_set_dir(DIR_PIN2_2, GPIO_OUT);
    gpio_put(DIR_PIN2_2, 0); // Set direction
    setup_pwm(PWM_PIN_2, 100.0f, 0.8f); // Set PWM

    // Initialize encoders
    gpio_init(ENCODER_PIN_1);
    gpio_set_dir(ENCODER_PIN_1, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_1);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(ENCODER_PIN_2);
    gpio_set_dir(ENCODER_PIN_2, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_2);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Setup for Ultrasonic Sensor
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, false);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_callback);

    printf("System initialized...\n");

    // Control loop for both motors
    while (true) {
        float median_distance = get_median_distance();
        trigger_sensor();

        if (median_distance >= 0 && median_distance <= STOP_DISTANCE_CM) {
            // Stop all movements if an obstacle is detected within the threshold
            printf("Object detected within %0.2f cm. Stopping motors.\n", STOP_DISTANCE_CM);

            // Set duty cycle to 0 to stop both motors
            update_pwm_duty_cycle(PWM_PIN_1, 0.0f);
            update_pwm_duty_cycle(PWM_PIN_2, 0.0f);

        } else {
            // // Reset encoder counts for this loop iteration
            // encoder_count_1 = 0;
            // encoder_count_2 = 0;

            // // Set both motors to move forward
            // gpio_put(DIR_PIN1_1, 1);
            // gpio_put(DIR_PIN2_1, 0);
            // gpio_put(DIR_PIN1_2, 1);    
            // gpio_put(DIR_PIN2_2, 0);

            // // Simulate a fixed time loop (e.g., 100 ms)
            // sleep_ms(100);

            // Run PID control
            control_motor_speed();

            // update_pwm_duty_cycle(PWM_PIN_1, 0.5f);
            // update_pwm_duty_cycle(PWM_PIN_2, 0.5f);

            bool state1 = gpio_get(ENCODER_PIN_1);
            bool state2 = gpio_get(ENCODER_PIN_2);
            printf("Encoder Pin 1 State: %d\n", state1);
            printf("Encoder Pin 2 State: %d\n", state2);

        }
    }

    return 0;
}


// Function to set up the PWM for any motor
void setup_pwm(uint gpio, float freq, float duty_cycle) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    float clock_freq = 125000000.0f;
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);
}

void update_motor_control(int motor, float duty_cycle) {
    // Determine which motor to control and call update_pwm_duty_cycle
    if (motor == 1) {
        update_pwm_duty_cycle(PWM_PIN_1, duty_cycle);
    } else {
        update_pwm_duty_cycle(PWM_PIN_2, duty_cycle);
    }
}

void update_pwm_duty_cycle(uint gpio, float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);  // Ensure the PWM slice is enabled after updating
}
