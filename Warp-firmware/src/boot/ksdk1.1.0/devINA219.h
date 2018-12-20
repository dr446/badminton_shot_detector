
void		initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

uint16_t readSensorRegisterINA219(uint8_t deviceRegister);

int16_t measure_current_INA219();
