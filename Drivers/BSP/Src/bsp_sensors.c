// SPDX-License-Identifier: MIT
/**
 * @file
 * @brief  I2C sensor bus implementation — MPU6500, AK8963, BMP280, SPL06.
 *
 * @details
 * All I2C traffic goes through hi2c1 (CubeMX-configured).  Device addresses
 * are 7-bit; the HAL expects left-shifted form, handled by
 * bsp_sensors_hal_address().
 */
#include "bsp_sensors.h"

#include "i2c.h"
#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define BSP_SENSORS_I2C_TIMEOUT_MS 20u  /* Per-transfer I2C timeout */
#define BSP_SENSORS_I2C_RETRIES 2u      /* Probe retry count for HAL_I2C_IsDeviceReady */

/* DRDY GPIO: directly connected to MPU INT output */
#define BSP_SENSORS_DRDY_PORT GPIOA
#define BSP_SENSORS_DRDY_PIN GPIO_PIN_4

/* --- I2C device addresses (7-bit) --- */
#define BSP_SENSORS_MPU_ADDRESS_LOW 0x68u   /* AD0 = GND */
#define BSP_SENSORS_MPU_ADDRESS_HIGH 0x69u  /* AD0 = VCC */
#define BSP_SENSORS_AK8963_ADDRESS 0x0Cu    /* AK8963 mag, behind MPU bypass */
#define BSP_SENSORS_BMP280_ADDRESS 0x76u    /* BMP280 baro, SDO = GND */
#define BSP_SENSORS_SPL06_ADDRESS 0x76u     /* SPL06 baro, SDO = GND */

/* --- MPU6500 register map (subset) --- */
#define BSP_SENSORS_MPU_REG_SMPLRT_DIV 0x19u   /* Sample rate divider */
#define BSP_SENSORS_MPU_REG_CONFIG 0x1Au        /* DLPF config */
#define BSP_SENSORS_MPU_REG_GYRO_CONFIG 0x1Bu   /* Gyro FS range */
#define BSP_SENSORS_MPU_REG_ACCEL_CONFIG 0x1Cu   /* Accel FS range */
#define BSP_SENSORS_MPU_REG_ACCEL_CONFIG_2 0x1Du /* Accel DLPF */
#define BSP_SENSORS_MPU_REG_INT_PIN_CFG 0x37u   /* INT pin / bypass config */
#define BSP_SENSORS_MPU_REG_INT_ENABLE 0x38u    /* Interrupt enable */
#define BSP_SENSORS_MPU_REG_INT_STATUS 0x3Au    /* Interrupt status (read-to-clear) */
#define BSP_SENSORS_MPU_REG_ACCEL_XOUT_H 0x3Bu  /* First data register (14-byte burst) */
#define BSP_SENSORS_MPU_REG_USER_CTRL 0x6Au     /* I2C master / FIFO control */
#define BSP_SENSORS_MPU_REG_PWR_MGMT_1 0x6Bu   /* Power management / clock select */
#define BSP_SENSORS_MPU_REG_WHO_AM_I 0x75u     /* Device ID register */

#define BSP_SENSORS_MPU_WHO_AM_I_MASK 0x7Eu    /* Mask out reserved bit 0 */
#define BSP_SENSORS_MPU_WHO_AM_I_VALUE 0x70u   /* Expected WHO_AM_I for MPU6500 */
#define BSP_SENSORS_MPU_INT_STATUS_DATA_READY 0x01u /* DATA_RDY_INT bit */
#define BSP_SENSORS_MPU_CLOCK_PLL_XGYRO 0x01u  /* PLL with X gyro as clock source */
#define BSP_SENSORS_MPU_GYRO_FS_2000DPS 0x18u  /* +/-2000 deg/s, bits 4:3 */
#define BSP_SENSORS_MPU_ACCEL_FS_16G 0x18u     /* +/-16g, bits 4:3 */
#define BSP_SENSORS_MPU_ACCEL_DLPF_41HZ 0x03u  /* Accel DLPF 41 Hz bandwidth */
#define BSP_SENSORS_MPU_GYRO_DLPF_98HZ 0x02u   /* Gyro DLPF 98 Hz bandwidth */
#define BSP_SENSORS_MPU_INT_CFG_BYPASS_EN 0x02u /* I2C bypass enable (for AK8963) */
#define BSP_SENSORS_MPU_INT_CFG_ANYRD_2CLEAR 0x10u /* Clear INT on any read */

/* --- AK8963 magnetometer register map --- */
#define BSP_SENSORS_AK8963_REG_WIA 0x00u       /* Device ID (read-only) */
#define BSP_SENSORS_AK8963_REG_ST1 0x02u       /* Status 1: DRDY bit */
#define BSP_SENSORS_AK8963_REG_HXL 0x03u       /* First data register (6 bytes + ST2) */
#define BSP_SENSORS_AK8963_REG_CNTL1 0x0Au     /* Control 1: operating mode */
#define BSP_SENSORS_AK8963_REG_ST2 0x09u       /* Status 2: overflow/error (must read after data) */
#define BSP_SENSORS_AK8963_WIA_VALUE 0x48u     /* Expected WIA value for AK8963 */
#define BSP_SENSORS_AK8963_MODE_CONTINUOUS_16BIT_100HZ 0x16u /* Continuous measurement mode 2 */
#define BSP_SENSORS_AK8963_STATUS_DATA_READY 0x01u  /* DRDY bit in ST1 */
#define BSP_SENSORS_AK8963_STATUS_OVERFLOW 0x08u    /* HOFL bit in ST2 */
#define BSP_SENSORS_AK8963_STATUS_DATA_ERROR 0x04u  /* DERR bit in ST2 */

/* --- BMP280 barometer register map --- */
#define BSP_SENSORS_BMP280_REG_CHIP_ID 0xD0u     /* Device ID (read-only) */
#define BSP_SENSORS_BMP280_REG_CTRL_MEAS 0xF4u   /* ctrl_meas: osrs_t x16, osrs_p x16, normal */
#define BSP_SENSORS_BMP280_REG_CONFIG 0xF5u      /* config: standby 500ms, filter x16 */
#define BSP_SENSORS_BMP280_REG_PRESSURE_MSB 0xF7u /* First data register (6 bytes: P + T) */
#define BSP_SENSORS_BMP280_CHIP_ID 0x58u         /* Expected chip ID for BMP280 */
#define BSP_SENSORS_BMP280_CTRL_MEAS_VALUE 0x97u /* osrs_t=x16 | osrs_p=x16 | mode=normal */
#define BSP_SENSORS_BMP280_CONFIG_VALUE 0x14u    /* t_sb=500ms | filter=x16 | spi3w_en=0 */

/* --- SPL06 barometer register map --- */
#define BSP_SENSORS_SPL06_REG_PRESSURE_MSB 0x00u      /* First data register (6 bytes: P + T) */
#define BSP_SENSORS_SPL06_REG_PRESSURE_CFG 0x06u      /* Pressure rate / oversampling */
#define BSP_SENSORS_SPL06_REG_TEMPERATURE_CFG 0x07u   /* Temperature rate / oversampling */
#define BSP_SENSORS_SPL06_REG_MODE_CFG 0x08u          /* Operating mode */
#define BSP_SENSORS_SPL06_REG_INT_FIFO_CFG 0x09u      /* Interrupt / FIFO config */
#define BSP_SENSORS_SPL06_REG_CHIP_ID 0x0Du           /* Device ID (read-only) */
#define BSP_SENSORS_SPL06_CHIP_ID 0x10u               /* Expected chip ID for SPL06 */
#define BSP_SENSORS_SPL06_PRESSURE_CFG_VALUE 0x56u    /* rate=64Hz, oversample=x64 */
#define BSP_SENSORS_SPL06_TEMPERATURE_CFG_VALUE 0xD3u /* rate=64Hz, oversample=x64, ext source */
#define BSP_SENSORS_SPL06_MODE_CFG_VALUE 0x07u        /* continuous pressure + temperature */
#define BSP_SENSORS_SPL06_INT_FIFO_CFG_VALUE 0x0Cu    /* FIFO disabled, active-high INT */

typedef struct
{
    bool initialized;
    bsp_sensors_status_t status;
} bsp_sensors_context_t;

static bsp_sensors_context_t sensors_context = {0};

/** @brief Convert 7-bit I2C address to HAL's left-shifted 8-bit format. */
static uint16_t bsp_sensors_hal_address(uint8_t address)
{
    return (uint16_t)(address << 1);
}

/** @brief Read @p length bytes from @p register_address on @p device_address. */
static bool bsp_sensors_read(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length == 0u)
    {
        return false;
    }

    return HAL_I2C_Mem_Read(
               &hi2c1,
               bsp_sensors_hal_address(device_address),
               register_address,
               I2C_MEMADD_SIZE_8BIT,
               buffer,
               length,
               BSP_SENSORS_I2C_TIMEOUT_MS) == HAL_OK;
}

/** @brief Write a single byte to @p register_address on @p device_address. */
static bool bsp_sensors_write(uint8_t device_address, uint8_t register_address, uint8_t value)
{
    uint8_t data = value;

    return HAL_I2C_Mem_Write(
               &hi2c1,
               bsp_sensors_hal_address(device_address),
               register_address,
               I2C_MEMADD_SIZE_8BIT,
               &data,
               1u,
               BSP_SENSORS_I2C_TIMEOUT_MS) == HAL_OK;
}

/** @brief Probe I2C bus for a device at @p device_address (ACK check). */
static bool bsp_sensors_probe_device(uint8_t device_address)
{
    return HAL_I2C_IsDeviceReady(
               &hi2c1,
               bsp_sensors_hal_address(device_address),
               BSP_SENSORS_I2C_RETRIES,
               BSP_SENSORS_I2C_TIMEOUT_MS) == HAL_OK;
}

/** @brief Probe and verify MPU WHO_AM_I register at @p address. */
static bool bsp_sensors_probe_mpu(uint8_t address)
{
    uint8_t who_am_i = 0u;

    if (!bsp_sensors_probe_device(address))
    {
        return false;
    }

    if (!bsp_sensors_read(address, BSP_SENSORS_MPU_REG_WHO_AM_I, &who_am_i, 1u))
    {
        return false;
    }

    return (who_am_i & BSP_SENSORS_MPU_WHO_AM_I_MASK) == BSP_SENSORS_MPU_WHO_AM_I_VALUE;
}

/** @brief Reset MPU, configure gyro/accel FS, DLPF, and enable data-ready interrupt. */
static bool bsp_sensors_init_mpu(uint8_t address)
{
    if (!bsp_sensors_write(address, BSP_SENSORS_MPU_REG_PWR_MGMT_1, 0x80u))
    {
        return false;
    }

    /* --- Wait for MPU to complete internal reset (datasheet: 100ms typ, 20ms sufficient here) --- */
    HAL_Delay(20u);

    return bsp_sensors_write(address, BSP_SENSORS_MPU_REG_PWR_MGMT_1, BSP_SENSORS_MPU_CLOCK_PLL_XGYRO) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_SMPLRT_DIV, 0x00u) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_CONFIG, BSP_SENSORS_MPU_GYRO_DLPF_98HZ) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_GYRO_CONFIG, BSP_SENSORS_MPU_GYRO_FS_2000DPS) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_ACCEL_CONFIG, BSP_SENSORS_MPU_ACCEL_FS_16G) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_ACCEL_CONFIG_2, BSP_SENSORS_MPU_ACCEL_DLPF_41HZ) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_USER_CTRL, 0x00u) &&
           bsp_sensors_write(
               address,
               BSP_SENSORS_MPU_REG_INT_PIN_CFG,
               BSP_SENSORS_MPU_INT_CFG_BYPASS_EN | BSP_SENSORS_MPU_INT_CFG_ANYRD_2CLEAR) &&
           bsp_sensors_write(address, BSP_SENSORS_MPU_REG_INT_ENABLE, BSP_SENSORS_MPU_INT_STATUS_DATA_READY);
}

/** @brief Probe AK8963 magnetometer via MPU bypass and verify WIA register. */
static bool bsp_sensors_probe_ak8963(void)
{
    uint8_t device_id = 0u;

    if (!bsp_sensors_probe_device(BSP_SENSORS_AK8963_ADDRESS))
    {
        return false;
    }

    if (!bsp_sensors_read(BSP_SENSORS_AK8963_ADDRESS, BSP_SENSORS_AK8963_REG_WIA, &device_id, 1u))
    {
        return false;
    }

    return device_id == BSP_SENSORS_AK8963_WIA_VALUE;
}

/** @brief Set AK8963 to continuous measurement mode (16-bit, 100 Hz). */
static bool bsp_sensors_init_ak8963(void)
{
    return bsp_sensors_write(BSP_SENSORS_AK8963_ADDRESS, BSP_SENSORS_AK8963_REG_CNTL1, 0x00u) &&
           bsp_sensors_write(
               BSP_SENSORS_AK8963_ADDRESS,
               BSP_SENSORS_AK8963_REG_CNTL1,
               BSP_SENSORS_AK8963_MODE_CONTINUOUS_16BIT_100HZ);
}

/** @brief Probe BMP280 barometer and verify chip ID. */
static bool bsp_sensors_probe_bmp280(void)
{
    uint8_t chip_id = 0u;

    if (!bsp_sensors_probe_device(BSP_SENSORS_BMP280_ADDRESS))
    {
        return false;
    }

    if (!bsp_sensors_read(BSP_SENSORS_BMP280_ADDRESS, BSP_SENSORS_BMP280_REG_CHIP_ID, &chip_id, 1u))
    {
        return false;
    }

    return chip_id == BSP_SENSORS_BMP280_CHIP_ID;
}

/** @brief Configure BMP280 for oversampled normal mode (x16, 500ms standby). */
static bool bsp_sensors_init_bmp280(void)
{
    return bsp_sensors_write(
               BSP_SENSORS_BMP280_ADDRESS,
               BSP_SENSORS_BMP280_REG_CTRL_MEAS,
               BSP_SENSORS_BMP280_CTRL_MEAS_VALUE) &&
           bsp_sensors_write(
               BSP_SENSORS_BMP280_ADDRESS,
               BSP_SENSORS_BMP280_REG_CONFIG,
               BSP_SENSORS_BMP280_CONFIG_VALUE);
}

/** @brief Probe SPL06 barometer and verify chip ID. */
static bool bsp_sensors_probe_spl06(void)
{
    uint8_t chip_id = 0u;

    if (!bsp_sensors_probe_device(BSP_SENSORS_SPL06_ADDRESS))
    {
        return false;
    }

    if (!bsp_sensors_read(BSP_SENSORS_SPL06_ADDRESS, BSP_SENSORS_SPL06_REG_CHIP_ID, &chip_id, 1u))
    {
        return false;
    }

    return chip_id == BSP_SENSORS_SPL06_CHIP_ID;
}

/** @brief Configure SPL06 for continuous mode, 64 Hz, oversample x64. */
static bool bsp_sensors_init_spl06(void)
{
    return bsp_sensors_write(
               BSP_SENSORS_SPL06_ADDRESS,
               BSP_SENSORS_SPL06_REG_PRESSURE_CFG,
               BSP_SENSORS_SPL06_PRESSURE_CFG_VALUE) &&
           bsp_sensors_write(
               BSP_SENSORS_SPL06_ADDRESS,
               BSP_SENSORS_SPL06_REG_TEMPERATURE_CFG,
               BSP_SENSORS_SPL06_TEMPERATURE_CFG_VALUE) &&
           bsp_sensors_write(
               BSP_SENSORS_SPL06_ADDRESS,
               BSP_SENSORS_SPL06_REG_INT_FIFO_CFG,
               BSP_SENSORS_SPL06_INT_FIFO_CFG_VALUE) &&
           bsp_sensors_write(
               BSP_SENSORS_SPL06_ADDRESS,
               BSP_SENSORS_SPL06_REG_MODE_CFG,
               BSP_SENSORS_SPL06_MODE_CFG_VALUE);
}

/** @brief Decode big-endian 16-bit signed value (MPU byte order). */
static int16_t bsp_sensors_read_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/** @brief Decode little-endian 16-bit signed value (AK8963 byte order). */
static int16_t bsp_sensors_read_le16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

/** @brief Sign-extend a 24-bit raw value to 32 bits (BMP280/SPL06 data format). */
static int32_t bsp_sensors_sign_extend_24(uint32_t value)
{
    if ((value & 0x00800000u) != 0u) /* bit 23 set: negative */
    {
        value |= 0xFF000000u;
    }

    return (int32_t)value;
}

void bsp_sensors_init(void)
{
    uint8_t imu_address = 0u;

    memset(&sensors_context, 0, sizeof(sensors_context));

    /* --- Probe IMU: try AD0=high first (AD0=low is the fallback) --- */
    if (bsp_sensors_probe_mpu(BSP_SENSORS_MPU_ADDRESS_HIGH))
    {
        imu_address = BSP_SENSORS_MPU_ADDRESS_HIGH;
    }
    else if (bsp_sensors_probe_mpu(BSP_SENSORS_MPU_ADDRESS_LOW))
    {
        imu_address = BSP_SENSORS_MPU_ADDRESS_LOW;
    }

    if (imu_address != 0u)
    {
        sensors_context.status.imu_address = imu_address;
        sensors_context.status.imu_present = bsp_sensors_init_mpu(imu_address);
    }

    /* --- Probe magnetometer (AK8963 behind MPU I2C bypass) --- */
    if (sensors_context.status.imu_present && bsp_sensors_probe_ak8963())
    {
        sensors_context.status.magnetometer_present = bsp_sensors_init_ak8963();
    }

    /* --- Probe barometer: try BMP280 first, fall back to SPL06 --- */
    if (bsp_sensors_probe_bmp280())
    {
        sensors_context.status.barometer_present = bsp_sensors_init_bmp280();
        sensors_context.status.barometer_type = sensors_context.status.barometer_present
                                                  ? BSP_SENSORS_BAROMETER_BMP280
                                                  : BSP_SENSORS_BAROMETER_NONE;
    }
    else if (bsp_sensors_probe_spl06())
    {
        sensors_context.status.barometer_present = bsp_sensors_init_spl06();
        sensors_context.status.barometer_type = sensors_context.status.barometer_present
                                                  ? BSP_SENSORS_BAROMETER_SPL06
                                                  : BSP_SENSORS_BAROMETER_NONE;
    }

    sensors_context.initialized = true;
}

bool bsp_sensors_is_initialized(void)
{
    return sensors_context.initialized;
}

void bsp_sensors_get_status(bsp_sensors_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    *status = sensors_context.status;
}

bool bsp_sensors_is_data_ready(void)
{
    uint8_t status = 0u;

    if (!sensors_context.initialized || !sensors_context.status.imu_present)
    {
        return false;
    }

    if (HAL_GPIO_ReadPin(BSP_SENSORS_DRDY_PORT, BSP_SENSORS_DRDY_PIN) != GPIO_PIN_SET)
    {
        return false;
    }

    if (!bsp_sensors_read(sensors_context.status.imu_address, BSP_SENSORS_MPU_REG_INT_STATUS, &status, 1u))
    {
        return false;
    }

    return (status & BSP_SENSORS_MPU_INT_STATUS_DATA_READY) != 0u;
}

bool bsp_sensors_read_imu_raw(
    bsp_sensors_axis3i16_t *accelerometer,
    bsp_sensors_axis3i16_t *gyroscope,
    int16_t *temperature_raw)
{
    uint8_t raw_data[14] = {0};

    if (accelerometer == NULL || gyroscope == NULL || temperature_raw == NULL)
    {
        return false;
    }

    if (!sensors_context.initialized || !sensors_context.status.imu_present)
    {
        return false;
    }

    if (!bsp_sensors_read(sensors_context.status.imu_address, BSP_SENSORS_MPU_REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data)))
    {
        return false;
    }

    accelerometer->x = bsp_sensors_read_be16(&raw_data[0]);
    accelerometer->y = bsp_sensors_read_be16(&raw_data[2]);
    accelerometer->z = bsp_sensors_read_be16(&raw_data[4]);
    *temperature_raw = bsp_sensors_read_be16(&raw_data[6]);
    gyroscope->x = bsp_sensors_read_be16(&raw_data[8]);
    gyroscope->y = bsp_sensors_read_be16(&raw_data[10]);
    gyroscope->z = bsp_sensors_read_be16(&raw_data[12]);

    return true;
}

bool bsp_sensors_read_magnetometer_raw(bsp_sensors_axis3i16_t *magnetometer)
{
    uint8_t raw_data[7] = {0};

    if (magnetometer == NULL)
    {
        return false;
    }

    if (!sensors_context.initialized || !sensors_context.status.magnetometer_present)
    {
        return false;
    }

    if (!bsp_sensors_read(BSP_SENSORS_AK8963_ADDRESS, BSP_SENSORS_AK8963_REG_ST1, raw_data, sizeof(raw_data)))
    {
        return false;
    }

    if ((raw_data[0] & BSP_SENSORS_AK8963_STATUS_DATA_READY) == 0u)
    {
        return false;
    }

    if ((raw_data[6] & (BSP_SENSORS_AK8963_STATUS_OVERFLOW | BSP_SENSORS_AK8963_STATUS_DATA_ERROR)) != 0u)
    {
        return false;
    }

    magnetometer->x = bsp_sensors_read_le16(&raw_data[1]);
    magnetometer->y = bsp_sensors_read_le16(&raw_data[3]);
    magnetometer->z = bsp_sensors_read_le16(&raw_data[5]);

    return true;
}

bool bsp_sensors_read_barometer_raw(bsp_sensors_baro_raw_t *barometer)
{
    uint8_t raw_data[6] = {0};

    if (barometer == NULL)
    {
        return false;
    }

    if (!sensors_context.initialized || !sensors_context.status.barometer_present)
    {
        return false;
    }

    if (sensors_context.status.barometer_type == BSP_SENSORS_BAROMETER_BMP280)
    {
        if (!bsp_sensors_read(BSP_SENSORS_BMP280_ADDRESS, BSP_SENSORS_BMP280_REG_PRESSURE_MSB, raw_data, sizeof(raw_data)))
        {
            return false;
        }

        /* BMP280: 20-bit unsigned, MSB-aligned in 3 bytes (shift right 4 bits) */
        barometer->pressure = (int32_t)((((uint32_t)raw_data[0]) << 12) |
                                        (((uint32_t)raw_data[1]) << 4) |
                                        ((uint32_t)raw_data[2] >> 4));
        barometer->temperature = (int32_t)((((uint32_t)raw_data[3]) << 12) |
                                           (((uint32_t)raw_data[4]) << 4) |
                                           ((uint32_t)raw_data[5] >> 4));
        return true;
    }

    if (sensors_context.status.barometer_type == BSP_SENSORS_BAROMETER_SPL06)
    {
        if (!bsp_sensors_read(BSP_SENSORS_SPL06_ADDRESS, BSP_SENSORS_SPL06_REG_PRESSURE_MSB, raw_data, sizeof(raw_data)))
        {
            return false;
        }

        /* SPL06: 24-bit signed, big-endian (sign-extend before use) */
        barometer->pressure = bsp_sensors_sign_extend_24(
            ((uint32_t)raw_data[0] << 16) |
            ((uint32_t)raw_data[1] << 8) |
            raw_data[2]);
        barometer->temperature = bsp_sensors_sign_extend_24(
            ((uint32_t)raw_data[3] << 16) |
            ((uint32_t)raw_data[4] << 8) |
            raw_data[5]);
        return true;
    }

    return false;
}
