/****************************************************************************
Created by Mr. Shivam Dhamesha, the library for MIKROE Pressure Click 20 (ICP20100) has been modified. 
The goal was to ensure compatibility with the standard HAL library of STM32, 
leading to the complete porting of the library for STM32.



Modded for SPI 
For I2C ¯\_(ツ)_/¯
****************************************************************************/

/*!
 * @file pressure20.h
 * @brief This file contains API for Pressure 20 Click Driver.
 */

#ifndef PRESSURE20_H
#define PRESSURE20_H

#ifdef __cplusplus
extern "C"{
#endif

#ifdef PREINIT_SUPPORTED
#include "preinit.h"
#endif

#include "stm32fxxx_hal.h"

#define PRESSURE20_REG_TRIM1_MSB                0x05
#define PRESSURE20_REG_TRIM2_LSB                0x06
#define PRESSURE20_REG_TRIM2_MSB                0x07
#define PRESSURE20_REG_DEVICE_ID                0x0C
#define PRESSURE20_REG_IO_DRIVE_STRENGTH        0x0D
#define PRESSURE20_REG_OTP_CONFIG1              0xAC
#define PRESSURE20_REG_OTP_MR_LSB               0xAD
#define PRESSURE20_REG_OTP_MR_MSB               0xAE
#define PRESSURE20_REG_OTP_MRA_LSB              0xAF
#define PRESSURE20_REG_OTP_MRA_MSB              0xB0
#define PRESSURE20_REG_OTP_MRB_LSB              0xB1
#define PRESSURE20_REG_OTP_MRB_MSB              0xB2
#define PRESSURE20_REG_OTP_ADDRESS_REG          0xB5
#define PRESSURE20_REG_OTP_COMMAND_REG          0xB6
#define PRESSURE20_REG_OTP_RDATA                0xB8
#define PRESSURE20_REG_OTP_STATUS               0xB9
#define PRESSURE20_REG_OTP_DBG2                 0xBC
#define PRESSURE20_REG_MASTER_LOCK              0xBE
#define PRESSURE20_REG_OTP_STATUS2              0xBF
#define PRESSURE20_REG_MODE_SELECT              0xC0
#define PRESSURE20_REG_INTERRUPT_STATUS         0xC1
#define PRESSURE20_REG_INTERRUPT_MASK           0xC2
#define PRESSURE20_REG_FIFO_CONFIG              0xC3
#define PRESSURE20_REG_FIFO_FILL                0xC4
#define PRESSURE20_REG_SPI_MODE                 0xC5
#define PRESSURE20_REG_PRESS_ABS_LSB            0xC7
#define PRESSURE20_REG_PRESS_ABS_MSB            0xC8
#define PRESSURE20_REG_PRESS_DELTA_LSB          0xC9
#define PRESSURE20_REG_PRESS_DELTA_MSB          0xCA
#define PRESSURE20_REG_DEVICE_STATUS            0xCD
#define PRESSURE20_REG_I3C_INFO                 0xCE
#define PRESSURE20_REG_VERSION                  0xD3
#define PRESSURE20_REG_DUMMY                    0xEE
#define PRESSURE20_REG_PRESS_DATA_0             0xFA
#define PRESSURE20_REG_PRESS_DATA_1             0xFB
#define PRESSURE20_REG_PRESS_DATA_2             0xFC
#define PRESSURE20_REG_TEMP_DATA_0              0xFD
#define PRESSURE20_REG_TEMP_DATA_1              0xFE
#define PRESSURE20_REG_TEMP_DATA_2              0xFF
#define PRESSURE20_TRIM1_MSB_PEFE_OFFSET_MASK   0x3F
#define PRESSURE20_TRIM2_LSB_HFOSC_MASK         0x7F
#define PRESSURE20_TRIM2_MSB_PEFE_GAIN_MASK     0x70
#define PRESSURE20_TRIM2_MSB_BG_PTAT_MASK       0x0F
#define PRESSURE20_OTP_ADDRESS_OFFSET           0xF8
#define PRESSURE20_OTP_ADDRESS_GAIN             0xF9
#define PRESSURE20_OTP_ADDRESS_HFOSC            0xFA
#define PRESSURE20_OTP_COMMAND_READ_ACTION      0x10
#define PRESSURE20_OTP_MRA_LSB                  0x04
#define PRESSURE20_OTP_MRA_MSB                  0x04
#define PRESSURE20_OTP_MRB_LSB                  0x21
#define PRESSURE20_OTP_MRB_MSB                  0x20
#define PRESSURE20_OTP_MR_LSB                   0x10
#define PRESSURE20_OTP_MR_MSB                   0x80
#define PRESSURE20_OTP_WRITE_SWITCH             0x02
#define PRESSURE20_OTP_ENABLE                   0x01
#define PRESSURE20_OTP_DISABLE                  0x00
#define PRESSURE20_OTP_RESET_SET                0x80
#define PRESSURE20_OTP_RESET_CLEAR              0x00
#define PRESSURE20_BUSY                         0x01
#define PRESSURE20_BOOT_UP_STATUS               0x01
#define PRESSURE20_MASTER_LOCK                  0x00
#define PRESSURE20_MASTER_UNLOCK                0x1F
#define PRESSURE20_MEAS_CONFIG_MODE0_ODR_25HZ   0x00
#define PRESSURE20_MEAS_CONFIG_MODE1_ODR_120HZ  0x20
#define PRESSURE20_MEAS_CONFIG_MODE2_ODR_40HZ   0x40
#define PRESSURE20_MEAS_CONFIG_MODE3_ODR_2HZ    0x60
#define PRESSURE20_MEAS_CONFIG_MODE4            0x80
#define PRESSURE20_FORCED_MEAS_TRIGGER          0x10
#define PRESSURE20_MEAS_MODE_TRIGGER            0x00
#define PRESSURE20_MEAS_MODE_CONTINUOUS         0x08
#define PRESSURE20_POWER_MODE_NORMAL            0x00
#define PRESSURE20_POWER_MODE_ACTIVE            0x04
#define PRESSURE20_FIFO_READOUT_MODE_PRESS_1ST  0x00
#define PRESSURE20_FIFO_READOUT_MODE_TEMP_ONLY  0x01
#define PRESSURE20_FIFO_READOUT_MODE_TEMP_1ST   0x02
#define PRESSURE20_FIFO_READOUT_MODE_PRESS_ONLY 0x03
#define PRESSURE20_MODE_SYNC_STATUS             0x01
#define PRESSURE20_FIFO_FLUSH                   0x80
#define PRESSURE20_FIFO_EMPTY                   0x40
#define PRESSURE20_FIFO_FULL                    0x20
#define PRESSURE20_FIFO_LEVEL_EMPTY             0x00
#define PRESSURE20_FIFO_LEVEL_1                 0x01
#define PRESSURE20_FIFO_LEVEL_2                 0x02
#define PRESSURE20_FIFO_LEVEL_3                 0x03
#define PRESSURE20_FIFO_LEVEL_4                 0x04
#define PRESSURE20_FIFO_LEVEL_5                 0x05
#define PRESSURE20_FIFO_LEVEL_6                 0x06
#define PRESSURE20_FIFO_LEVEL_7                 0x07
#define PRESSURE20_FIFO_LEVEL_8                 0x08
#define PRESSURE20_FIFO_LEVEL_9                 0x09
#define PRESSURE20_FIFO_LEVEL_10                0x0A
#define PRESSURE20_FIFO_LEVEL_11                0x0B
#define PRESSURE20_FIFO_LEVEL_12                0x0C
#define PRESSURE20_FIFO_LEVEL_13                0x0D
#define PRESSURE20_FIFO_LEVEL_14                0x0E
#define PRESSURE20_FIFO_LEVEL_15                0x0F
#define PRESSURE20_FIFO_LEVEL_FULL              0x10
#define PRESSURE20_INT_STAT_PRESS_DELTA         0x40
#define PRESSURE20_INT_STAT_PRESS_ABS           0x20
#define PRESSURE20_INT_STAT_FIFO_WMK_LOW        0x08
#define PRESSURE20_INT_STAT_FIFO_WMK_HIGH       0x04
#define PRESSURE20_INT_STAT_FIFO_UNDERFLOW      0x02
#define PRESSURE20_INT_STAT_FIFO_OVERFLOW       0x01
#define PRESSURE20_INT_MASK_RESERVED            0x80
#define PRESSURE20_INT_MASK_PRESS_DELTA         0x40
#define PRESSURE20_INT_MASK_PRESS_ABS           0x20
#define PRESSURE20_INT_MASK_FIFO_WMK_LOW        0x08
#define PRESSURE20_INT_MASK_FIFO_WMK_HIGH       0x04
#define PRESSURE20_INT_MASK_FIFO_UNDERFLOW      0x02
#define PRESSURE20_INT_MASK_FIFO_OVERFLOW       0x01
#define PRESSURE20_INT_MASK_ALL                 0xFF
#define PRESSURE20_DEVICE_ID                    0x63
#define PRESSURE20_PRESSURE_RES_RAW             0x020000ul
#define PRESSURE20_PRESSURE_RES_MBAR            400
#define PRESSURE20_PRESSURE_OFFSET_MBAR         700
#define PRESSURE20_TEMPERATURE_RES_RAW          0x040000ul
#define PRESSURE20_TEMPERATURE_RES_C            65
#define PRESSURE20_TEMPERATURE_OFFSET_C         25
#define PRESSURE20_SPI_READ_REG                 0x3C
#define PRESSURE20_SPI_WRITE_REG                0x33
#define PRESSURE20_DEVICE_ADDRESS_0             0x63
#define PRESSURE20_DEVICE_ADDRESS_1             0x64

/**
 * ENUM AND THE DATA STRUCT DEFINITION
 * 
*/

typedef enum
{
    PRESSURE20_OK = 0,
    PRESSURE20_ERROR = -1

} pressure20_return_value_t;

typedef struct 
{
    I2C_HandleTypeDef *hi2c;  /**< I2C handle */
    SPI_HandleTypeDef *hspi;  /**< SPI handle */
    uint8_t i2c_address;      /**< I2C address */
} pressure20_t;

typedef struct
{
    uint8_t reg_addr;
    uint8_t data_len;
    uint8_t *data_buf;

} pressure20_read_data_t;

typedef struct
{
    uint8_t reg_addr;
    uint8_t data_len;
    uint8_t *data_buf;

} pressure20_write_data_t;

#define PRESSURE20_MAP_MIKROBUS(cfg, mikrobus)  \
    cfg.hi2c = &hi2c##mikrobus

void pressure20_cfg_setup ( pressure20_t *cfg );
void pressure20_init ( pressure20_t *ctx, pressure20_t *cfg );
void pressure20_i2c_write ( pressure20_t *ctx, pressure20_write_data_t *data_in );
void pressure20_i2c_read ( pressure20_t *ctx, pressure20_read_data_t *data_out );
void pressure20_spi_write ( pressure20_t *ctx, pressure20_write_data_t *data_in );
void pressure20_spi_read ( pressure20_t *ctx, pressure20_read_data_t *data_out );
void pressure20_default_cfg ( pressure20_t *ctx );

#ifdef __cplusplus
}
#endif
#endif // PRESSURE20_H

// Implementation file pressure20.c

#include "pressure20.h"

void pressure20_cfg_setup ( pressure20_t *cfg )
{
    // Default I2C address
    cfg->i2c_address = PRESSURE20_DEVICE_ADDRESS_0;
}

void pressure20_init ( pressure20_t *ctx, pressure20_t *cfg )
{
    // Copy configuration
    ctx->hi2c = cfg->hi2c;
    ctx->hspi = cfg->hspi;
    ctx->i2c_address = cfg->i2c_address;
}

void pressure20_i2c_write ( pressure20_t *ctx, pressure20_write_data_t *data_in )
{
    HAL_I2C_Mem_Write(ctx->hi2c, ctx->i2c_address, data_in->reg_addr, I2C_MEMADD_SIZE_8BIT, data_in->data_buf, data_in->data_len, HAL_MAX_DELAY);
}

void pressure20_i2c_read ( pressure20_t *ctx, pressure20_read_data_t *data_out )
{
    HAL_I2C_Mem_Read(ctx->hi2c, ctx->i2c_address, data_out->reg_addr, I2C_MEMADD_SIZE_8BIT, data_out->data_buf, data_out->data_len, HAL_MAX_DELAY);
}

void pressure20_spi_write ( pressure20_t *ctx, pressure20_write_data_t *data_in )
{
    uint8_t reg_addr = data_in->reg_addr | PRESSURE20_SPI_WRITE_REG;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // using PA4 for the chip select
    HAL_SPI_Transmit(ctx->hspi, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(ctx->hspi, data_in->data_buf, data_in->data_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  
}

void pressure20_spi_read ( pressure20_t *ctx, pressure20_read_data_t *data_out )
{
    uint8_t reg_addr = data_out->reg_addr | PRESSURE20_SPI_READ_REG;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 
    HAL_SPI_Transmit(ctx->hspi, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(ctx->hspi, data_out->data_buf, data_out->data_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   
}

void pressure20_default_cfg ( pressure20_t *ctx )
{
    // Set default configuration
    uint8_t data = 0x00;

    // Example of writing default configuration
    pressure20_write_data_t write_data;
    write_data.reg_addr = PRESSURE20_REG_MODE_SELECT;
    write_data.data_len = 1;
    write_data.data_buf = &data;

    pressure20_i2c_write(ctx, &write_data);
}
