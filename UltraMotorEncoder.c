#include "pico/stdlib.h"
#include "stdio.h"
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Define GPIO pins for both motors
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

// Define pins for ultrasonic sensor
#define TRIG_PIN 8
#define ECHO_PIN 9

// Constants for ultrasonic sensor
#define MIN_VALID_DISTANCE 2.0
#define MAX_VALID_DISTANCE 400.0
#define MEASUREMENT_COUNT 3
#define FIXED_DELAY_MS 5
#define STOP_DISTANCE_CM 10.0 // Threshold distance to stop forward movement

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile bool measurement_complete = false;
volatile int encoder_count_1 = 0;  // Encoder count for motor 1
volatile int encoder_count_2 = 0;  // Encoder count for motor 2
volatile uint64_t last_time_1 = 0, last_time_2 = 0;
volatile uint64_t pulse_width_1 = 0, pulse_width_2 = 0;

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

void update_pwm_duty_cycle(uint gpio, float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));
    pwm_set_enabled(slice_num, true);  // Ensure the PWM slice is enabled after updating
}

// Encoder interrupt handlers for both motors
void encoder_callback_1(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t current_time = time_us_64();
        if (last_time_1 != 0) {
            pulse_width_1 = current_time - last_time_1;
            printf("Time since last interrupt (Motor 1): %llu microseconds\n", pulse_width_1);
        }
        last_time_1 = current_time;
        encoder_count_1++;
    }
}

void encoder_callback_2(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t current_time = time_us_64();
        if (last_time_2 != 0) {
            pulse_width_2 = current_time - last_time_2;
            printf("Time since last interrupt (Motor 2): %llu microseconds\n", pulse_width_2);
        }
        last_time_2 = current_time;
        encoder_count_2++;
    }
}

// Function to calculate wheel speed based on pulse width
float calculate_wheel_speed(uint64_t pulse_width) {
    float wheel_circumference_cm = 31.4;
    if (pulse_width > 0) {
        float pulse_duration_seconds = pulse_width / 1000000.0;
        return (wheel_circumference_cm / pulse_duration_seconds);
    }
    return 0.0;
}

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

// Function to calculate distance
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

int main() {
    stdio_init_all();

    // Setup for Motor 1
    gpio_init(DIR_PIN1_1);
    gpio_init(DIR_PIN2_1);
    gpio_set_dir(DIR_PIN1_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2_1, GPIO_OUT);
    setup_pwm(PWM_PIN_1, 100.0f, 0.9f);
    gpio_init(ENCODER_PIN_1);
    gpio_set_dir(ENCODER_PIN_1, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_1);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_1, GPIO_IRQ_EDGE_RISE, true, &encoder_callback_1);

    // Setup for Motor 2
    gpio_init(DIR_PIN1_2);
    gpio_init(DIR_PIN2_2);
    gpio_set_dir(DIR_PIN1_2, GPIO_OUT);
    gpio_set_dir(DIR_PIN2_2, GPIO_OUT);
    setup_pwm(PWM_PIN_2, 100.0f, 0.9f);
    gpio_init(ENCODER_PIN_2);
    gpio_set_dir(ENCODER_PIN_2, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_2);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_2, GPIO_IRQ_EDGE_RISE, true, &encoder_callback_2);

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
            // Set both motors to move forward
            gpio_put(DIR_PIN1_1, 1);
            gpio_put(DIR_PIN2_1, 0);
            gpio_put(DIR_PIN1_2, 1);
            gpio_put(DIR_PIN2_2, 0);


            update_pwm_duty_cycle(PWM_PIN_1, 0.9f);
            update_pwm_duty_cycle(PWM_PIN_2, 0.9f);
        }
        // Reporting speeds and rotations for both motors
        sleep_ms(1000);
        printf("Motor 1 - Wheel Speed: %.2f cm/s, Wheel Rotations: %d\n", calculate_wheel_speed(pulse_width_1), encoder_count_1 / 20);
        printf("Motor 2 - Wheel Speed: %.2f cm/s, Wheel Rotations: %d\n", calculate_wheel_speed(pulse_width_2), encoder_count_2 / 20);
        encoder_count_1 = encoder_count_2 = 0;
        last_time_1 = last_time_2 = 0;
    }

    return 0;
}
