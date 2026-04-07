/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_ICM42607)

#include "common/axis.h"
#include "common/utils.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm42607.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

#define ICM42607_RA_SIGNAL_PATH_RESET 0x02
#define ICM42607_RA_TEMP_DATA1        0x09
#define ICM42607_RA_ACCEL_DATA_X1     0x0B
#define ICM42607_RA_GYRO_DATA_X1      0x11
#define ICM42607_RA_PWR_MGMT0         0x1F
#define ICM42607_RA_GYRO_CONFIG0      0x20
#define ICM42607_RA_ACCEL_CONFIG0     0x21

#define ICM42607_PWR_MGMT0_ACCEL_LN   (3 << 0)
#define ICM42607_PWR_MGMT0_GYRO_LN    (3 << 2)

// ODR and FSR
#define ICM42607_GYRO_FSR_2000DPS     (0 << 5)
#define ICM42607_ACCEL_FSR_16G        (0 << 5)
#define ICM42607_ODR_1_6KHZ           0x05

#ifndef ICM42607_MAX_SPI_CLK_HZ
#define ICM42607_MAX_SPI_CLK_HZ 24000000
#endif

uint8_t icm42607SpiDetect(const extDevice_t *dev)
{
    // Check WHO_AM_I
    uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);

    if (whoAmI == ICM42607_WHO_AM_I_CONST) {
        // Soft reset device
        spiWriteReg(dev, ICM42607_RA_SIGNAL_PATH_RESET, 1 << 4);
        delay(100);
        return ICM_42607_SPI;
    }
    
    return MPU_NONE;
}

void icm42607GyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;
    spiSetClkDivisor(dev, spiCalculateDivider(ICM42607_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);
    
    // Data registers mapped to 0x0B for accel and 0x11 for gyro
    // Since Betaflight usually reads all 14 bytes starting from accel in mpuGyroReadSPI,
    // we must set accDataReg to 0x0B, and it will read Accel(6) + Temp(2) + Gyro(6) if consecutive.
    // Wait, the user says:
    // Temp: 0x09, 0x0A
    // Accel: 0x0B ~ 0x10
    // Gyro: 0x11 ~ 0x16
    // If mpuGyroReadSPI reads 14 bytes from accDataReg (0x0B), it will read:
    // 0x0B-0x10 (Accel), 0x11-0x16 (Gyro), and 0x17-0x18. Wait, Temp is at 0x09.
    // In typical MPU/ICM, data is Accel(6) -> Temp(2) -> Gyro(6) starting at 0x3B.
    // Here, it's Temp(2) -> Accel(6) -> Gyro(6) starting at 0x09.
    // If we want to read all 14 bytes in one SPI transaction, we should start at 0x09!
    // But Betaflight's `mpuGyroReadSPI` reads 14 bytes, expecting Accel X/Y/Z, Temp, Gyro X/Y/Z.
    // So if we start at 0x09, the order is Temp, Accel, Gyro. 
    // This will break `mpuGyroReadSPI` which expects a fixed struct order (Accel, Temp, Gyro).
    // Let's implement a custom read function for ICM-42607, similar to LSM6DSO.
    // For now, I will write the basic init.
    
    gyro->accDataReg = ICM42607_RA_ACCEL_DATA_X1;
    gyro->gyroDataReg = ICM42607_RA_GYRO_DATA_X1;

    // Power management: LN mode for Gyro and Accel
    spiWriteReg(dev, ICM42607_RA_PWR_MGMT0, ICM42607_PWR_MGMT0_ACCEL_LN | ICM42607_PWR_MGMT0_GYRO_LN);
    delay(15);

    // Gyro config: 2000 dps, 1.6kHz
    spiWriteReg(dev, ICM42607_RA_GYRO_CONFIG0, ICM42607_GYRO_FSR_2000DPS | ICM42607_ODR_1_6KHZ);
    delay(15);

    // Accel config: 16g, 1.6kHz
    spiWriteReg(dev, ICM42607_RA_ACCEL_CONFIG0, ICM42607_ACCEL_FSR_16G | ICM42607_ODR_1_6KHZ);
    delay(15);
}

static bool icm42607GyroReadSPI(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    uint8_t buffer[6];
    
    bool ack = spiReadRegMskBufRB(dev, ICM42607_RA_GYRO_DATA_X1, buffer, 6);
    if (!ack) {
        return false;
    }
    
    gyro->gyroADCRaw[X] = (int16_t)((buffer[0] << 8) | buffer[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((buffer[2] << 8) | buffer[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    return true;
}

static bool icm42607AccReadSPI(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;
    uint8_t buffer[6];
    
    bool ack = spiReadRegMskBufRB(dev, ICM42607_RA_ACCEL_DATA_X1, buffer, 6);
    if (!ack) {
        return false;
    }
    
    acc->ADCRaw[X] = (int16_t)((buffer[0] << 8) | buffer[1]);
    acc->ADCRaw[Y] = (int16_t)((buffer[2] << 8) | buffer[3]);
    acc->ADCRaw[Z] = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    return true;
}

bool icm42607SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != ICM_42607_SPI) {
        return false;
    }

    gyro->initFn = icm42607GyroInit;
    gyro->readFn = icm42607GyroReadSPI;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

void icm42607AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4; // 16G scale -> 2048 LSB/g
}

bool icm42607SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != ICM_42607_SPI) {
        return false;
    }

    acc->initFn = icm42607AccInit;
    acc->readFn = icm42607AccReadSPI;

    return true;
}

#endif // USE_GYRO_SPI_ICM42607
