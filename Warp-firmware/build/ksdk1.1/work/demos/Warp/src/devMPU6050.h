void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

uint32_t readSensorRegisterMPU6050(uint8_t deviceRegister);

void print_accelerations();
