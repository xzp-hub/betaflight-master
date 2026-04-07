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

/*
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_ICM42605) || defined(USE_GYRO_SPI_ICM42688P) || defined(USE_ACCGYRO_IIM42653)

#include "common/axis.h"
#include "common/utils.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

// Allows frequency to be set from the compile line EXTRA_FLAGS by adding e.g.
// -D'ICM426XX_CLOCK=12000000'. If using the configurator this simply becomes
// ICM426XX_CLOCK=12000000 in the custom settings text box.
#ifndef ICM426XX_CLOCK
// Default: 24 MHz max SPI frequency
#define ICM426XX_MAX_SPI_CLK_HZ 24000000
#else
// Use the supplied value
#define ICM426XX_MAX_SPI_CLK_HZ ICM426XX_CLOCK
#endif

#define ICM426XX_CLKIN_FREQ                         32000

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

// Fix for stalls in gyro output. See https://github.com/ArduPilot/ardupilot/pull/25332
#define ICM426XX_INTF_CONFIG1                       0x4D
#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0 (ICM42605/ICM42688P)
#define ICM42670_RA_PWR_MGMT0                       0x1F  // User Bank 0 (ICM42670-P)
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0 (ICM42605/ICM42688P)
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0 (ICM42605/ICM42688P)

// ICM42670 uses different register addresses
// From ICM-42670-L datasheet (DS-000431)
// User Bank 0 registers for ICM42670-L
#define ICM42670_RA_ACCEL_DATA_X1                   0x0B  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_GYRO_DATA_X1                    0x11  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_PWR_MGMT0                       0x1F  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_GYRO_CONFIG0                    0x20  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_ACCEL_CONFIG0                   0x21  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_TEMP_CONFIG0                    0x22  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_GYRO_CONFIG1                    0x23  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_ACCEL_CONFIG1                   0x24  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_INT_CONFIG                      0x06  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_INT_CONFIG0                     0x04  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_INT_CONFIG1                     0x05  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_INT_SOURCE0                     0x2B  // User Bank 0 (ICM42670-L)
#define ICM42670_RA_INTF_CONFIG1                    0x36  // User Bank 0 (ICM42670-L)

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

// specific to CLKIN configuration
#define ICM426XX_INTF_CONFIG5                       0x7B  // User Bank 1
#define ICM426XX_INTF_CONFIG1_CLKIN                 (1 << 2)
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK    (3 << 1)   // PIN9 mode config
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN   (2 << 1)   // PIN9 as CLKIN

typedef enum {
    ODR_CONFIG_8K = 0,
    ODR_CONFIG_4K,
    ODR_CONFIG_2K,
    ODR_CONFIG_1K,
    ODR_CONFIG_COUNT
} odrConfig_e;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible output data rates (ODRs) for ICM42605/ICM42688P/IIM42653
static uint8_t odrLUT[ODR_CONFIG_COUNT] = {  // see GYRO_ODR in section 5.6
    [ODR_CONFIG_8K] = 3,
    [ODR_CONFIG_4K] = 4,
    [ODR_CONFIG_2K] = 5,
    [ODR_CONFIG_1K] = 6,
};

// Possible output data rates (ODRs) for ICM42670-L
// ICM42670-L has different ODR values and max ODR is 1.6kHz
// 0101 (5): 1.6 kHz, 0110 (6): 800 Hz, 0111 (7): 400 Hz, 1000 (8): 200 Hz
static uint8_t odrLUT42670[ODR_CONFIG_COUNT] = {
    [ODR_CONFIG_8K] = 5,   // 1.6 kHz (max for ICM42670)
    [ODR_CONFIG_4K] = 5,   // 1.6 kHz
    [ODR_CONFIG_2K] = 5,   // 1.6 kHz
    [ODR_CONFIG_1K] = 6,   // 800 Hz
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
// actual cutoff differs slightly from those of the 42688P
static aafConfig_t aafLUT42605[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = { 21,  440,  6 }, // actually 249 Hz
    [AAF_CONFIG_536HZ]  = { 39, 1536,  4 }, // actually 524 Hz
    [AAF_CONFIG_997HZ]  = { 63, 3968,  3 }, // actually 995 Hz
    [AAF_CONFIG_1962HZ] = { 63, 3968,  3 }, // 995 Hz is the max cutoff on the 42605
};

static void setUserBank(const extDevice_t *dev, const uint8_t user_bank)
{
    spiWriteReg(dev, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

#if defined(USE_GYRO_CLKIN)
static pwmOutputPort_t pwmGyroClk = {0};

static bool initExternalClock(const extDevice_t *dev)
{
    int cfg;
    if (&gyro.gyroSensor1.gyroDev.dev == dev) {
        cfg = 0;
    } else if (&gyro.gyroSensor2.gyroDev.dev == dev) {
        cfg = 1;
    } else {
        // only gyroSensor<n> device supported
        return false;
    }
    const ioTag_t tag = gyroDeviceConfig(cfg)->clkIn;
    const IO_t io = IOGetByTag(tag);
    if (pwmGyroClk.enabled) {
       // pwm is already taken, but test for shared clkIn pin
       return pwmGyroClk.io == io;
    }

    const timerHardware_t *timer = timerAllocate(tag, OWNER_GYRO_CLKIN, RESOURCE_INDEX(cfg));
    if (!timer) {
        // Error handling: failed to allocate timer
        return false;
    }

    pwmGyroClk.io = io;
    pwmGyroClk.enabled = true;

    IOInit(io, OWNER_GYRO_CLKIN, RESOURCE_INDEX(cfg));
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

    const uint32_t clock = timerClock(timer->tim);  // Get the timer clock frequency
    const uint16_t period = clock / ICM426XX_CLKIN_FREQ;

    // Calculate duty cycle value for 50%
    const uint16_t value = period / 2;

    // Configure PWM output
    pwmOutConfig(&pwmGyroClk.channel, timer, clock, period - 1, value - 1, 0);

    // Set CCR value
    *pwmGyroClk.channel.ccr = value - 1;

    return true;
}

static void icm426xxEnableExternalClock(const extDevice_t *dev)
{
    if (initExternalClock(dev)) {
        // Switch to Bank 1 and set bits 2:1 in INTF_CONFIG5 (0x7B) to enable CLKIN on PIN9
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        uint8_t intf_config5 = spiReadRegMsk(dev, ICM426XX_INTF_CONFIG5);
        intf_config5 = (intf_config5 & ~ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK) | ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN;  // Clear & set bits 2:1 to 0b10 for CLKIN
        spiWriteReg(dev, ICM426XX_INTF_CONFIG5, intf_config5);

        // Switch to Bank 0 and set bit 2 in RTC_MODE (0x4D) to enable external CLK signal
        setUserBank(dev, ICM426XX_BANK_SELECT0);
        uint8_t rtc_mode = spiReadRegMsk(dev, ICM426XX_INTF_CONFIG1);
        rtc_mode |= ICM426XX_INTF_CONFIG1_CLKIN; // Enable external CLK signal
        spiWriteReg(dev, ICM426XX_INTF_CONFIG1, rtc_mode);
    }
}
#endif

uint8_t icm426xxSpiDetect(const extDevice_t *dev)
{
    // For ICM42670, PWR_MGMT0 is at 0x1F, for others it's at 0x4E
    // Try 0x4E first (ICM42605/ICM42688P)
    spiWriteReg(dev, ICM426XX_RA_PWR_MGMT0, 0x00);

#if defined(USE_GYRO_CLKIN)
    icm426xxEnableExternalClock(dev);
#endif

    uint8_t icmDetected = MPU_NONE;
    uint8_t attemptsRemaining = 20;
    do {
        delay(150);
        const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
        
        // 添加调试输出
        DEBUG_SET(DEBUG_GYRO_RAW, 0, whoAmI);
        
        switch (whoAmI) {
        case ICM42605_WHO_AM_I_CONST:  // 0x42
            icmDetected = ICM_42605_SPI;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 42605);
            break;
        case ICM42607_WHO_AM_I_CONST:  // 0x60
            icmDetected = ICM_42607_SPI;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 42607);
            break;
        case ICM42670_WHO_AM_I_CONST:  // 0x63 - ICM42670-L
            // Re-initialize power management with correct address for ICM42670
            spiWriteReg(dev, ICM42670_RA_PWR_MGMT0, 0x00);
            delay(10);
            icmDetected = ICM_42670_SPI;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 42670);
            break;
        case ICM42688P_WHO_AM_I_CONST:  // 0x47
            icmDetected = ICM_42688P_SPI;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 42688);
            break;
        case IIM42653_WHO_AM_I_CONST:  // 0x56
            icmDetected = IIM_42653_SPI;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 42653);
            break;
        default:
            // 未知的 WHO_AM_I
            icmDetected = MPU_NONE;
            DEBUG_SET(DEBUG_GYRO_RAW, 1, 0);
            break;
        }
        if (icmDetected != MPU_NONE) {
            break;
        }
        if (!attemptsRemaining) {
            DEBUG_SET(DEBUG_GYRO_RAW, 2, 9999);
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    DEBUG_SET(DEBUG_GYRO_RAW, 3, icmDetected);
    
    // 诊断：读取关键寄存器
    // PWR_MGMT0 (0x1F) - 应该显示传感器是否被唤醒
    uint8_t pwrMgmt0 = spiReadRegMsk(dev, 0x1F);
    DEBUG_SET(DEBUG_GYRO_RAW, 2, pwrMgmt0);
    
    return icmDetected;
}

void icm426xxAccInit(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case IIM_42653_SPI:
        acc->acc_1G = 512 * 2; // Accel scale 32g (1024 LSB/g)
        break;
    case ICM_42670_SPI:
    default:
        acc->acc_1G = 512 * 4; // Accel scale 16g (2048 LSB/g)
        break;
    }
}

bool icm426xxSpiAccDetect(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
    case ICM_42688P_SPI:
    case ICM_42670_SPI:
    case IIM_42653_SPI:
        break;
    default:
        return false;
    }

    acc->initFn = icm426xxAccInit;
    acc->readFn = mpuAccReadSPI;

    return true;
}

static aafConfig_t getGyroAafConfig(const mpuSensor_e, const aafConfig_e);

static void turnGyroAccOff(const extDevice_t *dev, mpuSensor_e sensor)
{
    uint8_t pwrAddr = (sensor == ICM_42670_SPI) ? ICM42670_RA_PWR_MGMT0 : ICM426XX_RA_PWR_MGMT0;
    spiWriteReg(dev, pwrAddr, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
    // ICM42670 requires 200us delay after power mode transition
    if (sensor == ICM_42670_SPI) {
        delayMicroseconds(200);
    }
}

// Turn on gyro and acc on in Low Noise mode
static void turnGyroAccOn(const extDevice_t *dev, mpuSensor_e sensor)
{
    uint8_t pwrAddr = (sensor == ICM_42670_SPI) ? ICM42670_RA_PWR_MGMT0 : ICM426XX_RA_PWR_MGMT0;
    spiWriteReg(dev, pwrAddr, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    // ICM42670 requires 200us delay after power mode transition
    // and 30-40ms for gyro startup, 10-20ms for accel startup
    if (sensor == ICM_42670_SPI) {
        delayMicroseconds(200);  // Wait 200us before any other register write
        delay(50);               // Wait 50ms for gyro/accel to stabilize
    } else {
        delay(1);
    }
}

void icm426xxGyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(ICM426XX_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);
    
    // ICM42670 uses different data register addresses
    const mpuSensor_e gyroModel = gyro->mpuDetectionResult.sensor;
    if (gyroModel == ICM_42670_SPI) {
        gyro->accDataReg = ICM42670_RA_ACCEL_DATA_X1;  // 0x0B for ICM42670
        gyro->gyroDataReg = ICM42670_RA_GYRO_DATA_X1;  // 0x11 for ICM42670
    } else {
        gyro->accDataReg = ICM426XX_RA_ACCEL_DATA_X1;  // 0x1F for others
        gyro->gyroDataReg = ICM426XX_RA_GYRO_DATA_X1;  // 0x25 for others
    }

    // Ensure we're on Bank 0
    setUserBank(dev, ICM426XX_BANK_SELECT0);
    
    // For ICM42670, perform a soft reset first
    if (gyroModel == ICM_42670_SPI) {
        // Soft reset: write to SIGNAL_PATH_RESET (0x02), bit 4
        spiWriteReg(dev, 0x02, 0x10);  // Soft reset device config
        delay(100);  // Wait for reset to complete
    }
    
    // Turn off ACC and GYRO so they can be configured
    turnGyroAccOff(dev, gyroModel);
    delay(10);

    // Configure gyro Anti-Alias Filter (AAF)
    // Note: ICM42670-L does NOT have AAF configuration registers in Bank 1/2
    // The AAF is fixed and cannot be configured like other ICM426xx chips
    if (gyroModel != ICM_42670_SPI) {
        aafConfig_t aafConfig = getGyroAafConfig(gyroModel, gyroConfig()->gyro_hardware_lpf);
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
        spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

        // Configure acc Anti-Alias Filter
        aafConfig = getGyroAafConfig(gyroModel, AAF_CONFIG_258HZ);
        setUserBank(dev, ICM426XX_BANK_SELECT2);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
        spiWriteReg(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

        // Switch back to Bank 0
        setUserBank(dev, ICM426XX_BANK_SELECT0);
    }
    
    // Configure gyro and acc UI Filters
    // ICM42670 uses ACCEL_CONFIG1 (0x24) for UI filter config, not separate GYRO_ACCEL_CONFIG0
    if (gyroModel == ICM_42670_SPI) {
        // For ICM42670: GYRO_UI_FILT_BW is in GYRO_CONFIG1 (0x23), ACCEL_UI_FILT_BW is in ACCEL_CONFIG1 (0x24)
        spiWriteReg(dev, ICM42670_RA_GYRO_CONFIG1, ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);
        spiWriteReg(dev, ICM42670_RA_ACCEL_CONFIG1, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY);
    } else {
        spiWriteReg(dev, ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);
    }

    // Configure interrupt pin
    uint8_t intConfigAddr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_INT_CONFIG : ICM426XX_RA_INT_CONFIG;
    spiWriteReg(dev, intConfigAddr, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    
    // Configure INT_CONFIG0 (data ready interrupt clear option)
    uint8_t intConfig0Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_INT_CONFIG0 : ICM426XX_RA_INT_CONFIG0;
    spiWriteReg(dev, intConfig0Addr, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);
    
    uint8_t intSource0Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_INT_SOURCE0 : ICM426XX_RA_INT_SOURCE0;
    spiWriteReg(dev, intSource0Addr, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    // Configure INT_CONFIG1 (interrupt timing settings)
    uint8_t intConfig1Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_INT_CONFIG1 : ICM426XX_RA_INT_CONFIG1;
    uint8_t intConfig1Value = spiReadRegMsk(dev, intConfig1Addr);
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);
    spiWriteReg(dev, intConfig1Addr, intConfig1Value);

    // Disable AFSR to prevent stalls in gyro output (not applicable for ICM42670)
    if (gyroModel != ICM_42670_SPI) {
        uint8_t intfConfig1Value = spiReadRegMsk(dev, ICM426XX_INTF_CONFIG1);
        intfConfig1Value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
        intfConfig1Value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
        spiWriteReg(dev, ICM426XX_INTF_CONFIG1, intfConfig1Value);
    }

    // Get desired output data rate
    uint8_t odrConfig;
    const unsigned decim = llog2(gyro->mpuDividerDrops + 1);
    if (gyro->gyroRateKHz && decim < ODR_CONFIG_COUNT) {
        // ICM42670 uses different ODR values
        odrConfig = (gyroModel == ICM_42670_SPI) ? odrLUT42670[decim] : odrLUT[decim];
    } else {
        odrConfig = (gyroModel == ICM_42670_SPI) ? odrLUT42670[ODR_CONFIG_1K] : odrLUT[ODR_CONFIG_1K];
        gyro->gyroRateKHz = GYRO_RATE_1_kHz;
    }

    // Configure FSR and ODR BEFORE turning on sensors
    uint8_t gyroConfig0Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_GYRO_CONFIG0 : ICM426XX_RA_GYRO_CONFIG0;
    uint8_t accelConfig0Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_ACCEL_CONFIG0 : ICM426XX_RA_ACCEL_CONFIG0;
    spiWriteReg(dev, gyroConfig0Addr, (0 << 5) | (odrConfig & 0x0F));
    delay(15);
    spiWriteReg(dev, accelConfig0Addr, (0 << 5) | (odrConfig & 0x0F));
    delay(15);

    // Turn on gyro and acc in Low Noise mode
    turnGyroAccOn(dev, gyroModel);
    
    // Final stabilization delay
    delay(100);
    
    // Ensure we're on Bank 0 for data reading
    setUserBank(dev, ICM426XX_BANK_SELECT0);

    // Debug: Verify sensor is working by reading key registers
    uint8_t pwrMgmt0Addr = (gyroModel == ICM_42670_SPI) ? ICM42670_RA_PWR_MGMT0 : ICM426XX_RA_PWR_MGMT0;
    uint8_t pwrMgmt0Check = spiReadRegMsk(dev, pwrMgmt0Addr);
    uint8_t whoAmICheck = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
    uint8_t accDataCheck = spiReadRegMsk(dev, ICM42670_RA_ACCEL_DATA_X1);
    uint8_t gyroDataCheck = spiReadRegMsk(dev, ICM42670_RA_GYRO_DATA_X1);

    DEBUG_SET(DEBUG_GYRO_RAW, 0, whoAmICheck);      // WHO_AM_I (should be 0x63)
    DEBUG_SET(DEBUG_GYRO_RAW, 1, pwrMgmt0Check);   // PWR_MGMT0 (should be 0x0F)
    DEBUG_SET(DEBUG_GYRO_RAW, 2, accDataCheck);     // ACC X high byte
    DEBUG_SET(DEBUG_GYRO_RAW, 3, gyroDataCheck);   // GYRO X high byte
}

bool icm426xxSpiGyroDetect(gyroDev_t *gyro)
{
    switch (gyro->mpuDetectionResult.sensor) {
    case ICM_42605_SPI:
    case ICM_42688P_SPI:
    case ICM_42670_SPI:
        gyro->scale = GYRO_SCALE_2000DPS;
        break;
    case IIM_42653_SPI:
        gyro->scale = GYRO_SCALE_4000DPS;
        break;
    default:
        return false;
    }

    gyro->initFn = icm426xxGyroInit;
    gyro->readFn = mpuGyroReadSPI;

    return true;
}

static aafConfig_t getGyroAafConfig(const mpuSensor_e gyroModel, const aafConfig_e config)
{
    switch (gyroModel){
    case ICM_42605_SPI:
    case ICM_42670_SPI:
        switch (config) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return aafLUT42605[AAF_CONFIG_258HZ];
        case GYRO_HARDWARE_LPF_OPTION_1:
            return aafLUT42605[AAF_CONFIG_536HZ];
        case GYRO_HARDWARE_LPF_OPTION_2:
            return aafLUT42605[AAF_CONFIG_997HZ];
        default:
            return aafLUT42605[AAF_CONFIG_258HZ];
        }

    case ICM_42688P_SPI:
    case IIM_42653_SPI:
    default:
        switch (config) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return aafLUT42688[AAF_CONFIG_258HZ];
        case GYRO_HARDWARE_LPF_OPTION_1:
            return aafLUT42688[AAF_CONFIG_536HZ];
        case GYRO_HARDWARE_LPF_OPTION_2:
            return aafLUT42688[AAF_CONFIG_997HZ];
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return aafLUT42688[AAF_CONFIG_1962HZ];
#endif
        default:
            return aafLUT42688[AAF_CONFIG_258HZ];
        }
    }
}

#endif // USE_GYRO_SPI_ICM42605 || USE_GYRO_SPI_ICM42688P || USE_ACCGYRO_IIM42653
