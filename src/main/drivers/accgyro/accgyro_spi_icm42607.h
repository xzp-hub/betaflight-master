#pragma once

#include "drivers/accgyro/accgyro.h"

uint8_t icm42607SpiDetect(const extDevice_t *dev);
bool icm42607SpiAccDetect(accDev_t *acc);
bool icm42607SpiGyroDetect(gyroDev_t *gyro);
