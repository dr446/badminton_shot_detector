#pragma once

void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

uint32_t readSensorRegisterMPU6050(uint8_t deviceRegister);

void print_accelerations();

void MPU6050_ISR();

void update_shot_buffer();

uint16_t get_acc_x();

uint16_t get_acc_y();

uint16_t get_acc_z();
