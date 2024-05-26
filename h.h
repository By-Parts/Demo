/****************************************************************************
Created by Mr. Shivam Dhamesha, the library for MIKROE Pressure Click 20 (ICP20100) has been modified. 
The goal was to ensure compatibility with the standard HAL library of STM32, 
leading to the complete porting of the library for STM32.
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


#include "drv_digital_out.h"
#include "drv_digital_in.h"
#include "drv_i2c_master.h"
#include "drv_spi_master.h"
#include "spi_specifics.h"

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
#define PRESSURE20_SET_DATA_SAMPLE_EDGE         SET_SPI_DATA_SAMPLE_EDGE
#define PRESSURE20_SET_DATA_SAMPLE_MIDDLE       SET_SPI_DATA_SAMPLE_MIDDLE


typedef enum
{
    PRESSURE20_HAL_SEL_SPI,             /**< SPI driver descriptor. */
    PRESSURE20_HAL_SEL_I2C              /**< I2C driver descriptor. */

} pressure20_hal_t;

typedef HAL_StatusTypeDef ( *pressure20_master_io_t )( struct pressure20_s*, uint8_t, uint8_t*, uint8_t ); /**< Driver serial interface. */

typedef struct pressure20_s
{
    GPIO_TypeDef*  int_pin;             /**< Interrupt pin. */

    I2C_HandleTypeDef  i2c;             /**< I2C driver object. */
    SPI_HandleTypeDef  spi;             /**< SPI driver object. */

    uint8_t     slave_address;          /**< Device slave address (used for I2C driver). */
    GPIO_TypeDef*  chip_select;         /**< Chip select pin descriptor (used for SPI driver). */
    pressure20_hal_t  hal_sel;          /**< Master driver interface selector. */

    pressure20_master_io_t  write_f;    /**< Master write function. */
    pressure20_master_io_t  read_f;     /**< Master read function. */

} pressure20_t;

typedef struct
{
    GPIO_TypeDef*  scl;                 /**< Clock pin descriptor for I2C driver. */
    GPIO_TypeDef*  sda;                 /**< Bidirectional data pin descriptor for I2C driver. */
    GPIO_TypeDef*  miso;                /**< Master input - slave output pin descriptor for SPI driver. */
    GPIO_TypeDef*  mosi;                /**< Master output - slave input pin descriptor for SPI driver. */
    GPIO_TypeDef*  sck;                 /**< Clock pin descriptor for SPI driver. */
    GPIO_TypeDef*  cs;                  /**< Chip select pin descriptor for SPI driver. */
    GPIO_TypeDef*  int_pin;             /**< Interrupt pin. */

    uint32_t  i2c_speed;                /**< I2C serial speed. */
    uint8_t   i2c_address;              /**< I2C slave address. */
    uint32_t  spi_speed;                /**< SPI serial speed. */
    uint32_t  spi_mode;                 /**< SPI master mode. */
    uint32_t  cs_polarity;              /**< Chip select pin polarity. */

    pressure20_hal_t  hal_sel;          /**< Master driver interface selector. */

} pressure20_cfg_t;

typedef enum
{
    PRESSURE20_OK = HAL_OK,
    PRESSURE20_ERROR = HAL_ERROR

} pressure20_return_value_t;


/*!
 * @addtogroup pressure20 Pressure 20 Click Driver
 * @brief API for configuring and manipulating Pressure 20 Click driver.
 * @{
 */

/**
 * @brief Pressure 20 configuration object setup function.
 * @details This function initializes click configuration structure to initial
 * values.
 * @param[out] cfg : Click configuration structure.
 * See #pressure20_cfg_t object definition for detailed explanation.
 * @return Nothing.
 * @note The all used pins will be set to unconnected state.
 */
void pressure20_cfg_setup ( pressure20_cfg_t *cfg );

/**
 * @brief Pressure 20 driver interface setup function.
 * @details This function sets a serial driver interface which will be used
 * further in the click driver.
 * @param[out] cfg : Click configuration structure.
 * See #pressure20_cfg_t object definition for detailed explanation.
 * @param[in] drv_sel : Driver interface selection.
 * See #pressure20_drv_t object definition for detailed explanation.
 * @return Nothing.
 * @note This driver selection should be call before init function to configure
 * the driver to work with the serial interface which is consistent with the
 * real state of the hardware. If this function is not called, the default
 * driver interface will be set.
 */
void pressure20_drv_interface_selection ( pressure20_cfg_t *cfg, pressure20_drv_t drv_sel );

/**
 * @brief Pressure 20 initialization function.
 * @details This function initializes all necessary pins and peripherals used
 * for this click board.
 * @param[out] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] cfg : Click configuration structure.
 * See #pressure20_cfg_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_init ( pressure20_t *ctx, pressure20_cfg_t *cfg );

/**
 * @brief Pressure 20 default configuration function.
 * @details This function executes a default configuration of Pressure 20
 * click board.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note This function can consist any necessary configuration or setting to put
 * device into operating mode.
 */
err_t pressure20_default_cfg ( pressure20_t *ctx );

/**
 * @brief Pressure 20 data writing function.
 * @details This function writes a desired number of data bytes starting from
 * the selected register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[in] data_in : Data to be written.
 * @param[in] len : Number of bytes to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_generic_write ( pressure20_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len );

/**
 * @brief Pressure 20 data reading function.
 * @details This function reads a desired number of data bytes starting from
 * the selected register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] data_out : Output read data.
 * @param[in] len : Number of bytes to be read.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_generic_read ( pressure20_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len );

/**
 * @brief Pressure 20 write register function.
 * @details This function writes a desired data to the selected register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] reg : Register address.
 * @param[in] data_in : Data to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_write_register ( pressure20_t *ctx, uint8_t reg, uint8_t data_in );

/**
 * @brief Pressure 20 read register function.
 * @details This function reads data from the selected register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] reg : Register address.
 * @param[out] data_out : Output read data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_read_register ( pressure20_t *ctx, uint8_t reg, uint8_t *data_out );

/**
 * @brief Pressure 20 check communication function.
 * @details This function checks the communication by reading and verifying the device ID.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_check_communication ( pressure20_t *ctx );

/**
 * @brief Pressure 20 write mode select function.
 * @details This function writes data to the mode select register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] mode_select : Data to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_write_mode_select ( pressure20_t *ctx, uint8_t mode_select );

/**
 * @brief Pressure 20 read otp data function.
 * @details This function reads the OTP data from the selected register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[in] otp_address : OTP register address.
 * @param[out] data_out : Output read data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_read_otp_data ( pressure20_t *ctx, uint8_t otp_address, uint8_t *data_out );

/**
 * @brief Pressure 20 read data function.
 * @details This function reads the pressure [mBar] and temperature [Celsius] data.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @param[out] pressure : Pressure in mBar.
 * @param[out] temperature : Temperature in Celsius.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_read_data ( pressure20_t *ctx, float *pressure, float *temperature );

/**
 * @brief Pressure 20 get int pin function.
 * @details This function returns the INT pin logic state.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @return Pin logic state.
 * @note None.
 */
uint8_t pressure20_get_int_pin ( pressure20_t *ctx );

/**
 * @brief Pressure 20 clear interrupts function.
 * @details This function reads and clears the interrupt status register.
 * @param[in] ctx : Click context object.
 * See #pressure20_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 * See #err_t definition for detailed explanation.
 * @note None.
 */
err_t pressure20_clear_interrupts ( pressure20_t *ctx );

#ifdef __cplusplus
}
#endif
#endif // PRESSURE20_H

/*! @} */ // pressure20

// ------------------------------------------------------------------------ END
