#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Define I2C port and pins
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// Device addresses and registers
#define ACC_ADDRESS 0x19
#define ACC_CTRL_REG1_A 0x20
#define ACC_OUT_X_L_A 0x28
#define ACC_OUT_Y_L_A 0x2A
#define ACC_OUT_Z_L_A 0x2C
#define ACC_CTRL_REG4_A 0x23

#define MAG_ADDRESS 0x1E
#define MAG_MR_REG_M 0x02
#define MAG_OUT_X_H_M 0x03
#define MAG_OUT_Y_H_M 0x05
#define MAG_OUT_Z_H_M 0x07

int raw_xa, raw_ya, raw_za;
int raw_xm, raw_ym, raw_zm;

void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, address, buffer, 2, false);
}

void i2c_read_bytes(uint8_t address, uint8_t reg, uint8_t* buffer, int length) {
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, address, buffer, length, false);
}

void setup() {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    i2c_write_byte(ACC_ADDRESS, ACC_CTRL_REG1_A, 0x57);
    i2c_write_byte(MAG_ADDRESS, MAG_MR_REG_M, 0x00);
    i2c_write_byte(ACC_ADDRESS, ACC_CTRL_REG4_A,0x00);
}

void loop() {
    uint8_t accel_data_x[2], accel_data_y[2], accel_data_z[2];
    uint8_t mag_data_x[2], mag_data_y[2], mag_data_z[2];

    // Read accelerometer data
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_X_L_A | 0x80, accel_data_x, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Y_L_A | 0x80, accel_data_y, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Z_L_A | 0x80, accel_data_z, 2);
    raw_xa = (accel_data_x[0] | (accel_data_x[1] << 8));
    raw_ya = (accel_data_y[0] | (accel_data_y[1] << 8));
    raw_za = (accel_data_z[0] | (accel_data_z[1] << 8));

    // Read magnetometer data
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_X_H_M, mag_data_x, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Y_H_M, mag_data_y, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Z_H_M, mag_data_z, 2);
    raw_xm = (int16_t)((uint16_t)mag_data_x[1] | ((uint16_t)mag_data_x[0] << 8));
    raw_ym = (int16_t)((uint16_t)mag_data_y[1] | ((uint16_t)mag_data_y[0] << 8));
    raw_zm = (int16_t)((uint16_t)mag_data_z[1] | ((uint16_t)mag_data_z[0] << 8));

    // Output raw values for reference
    printf("Accelerometer: X=%d Y=%d Z=%d\n", raw_xa, raw_ya, raw_za);
    // printf("Magnetometer: X=%d Y=%d Z=%d\n", raw_xm, raw_ym, raw_zm);

    sleep_ms(100); // Delay of 1000ms
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
