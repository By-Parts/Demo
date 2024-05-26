#include "pressure20.h"

/**
 * @brief Dummy data.
 * @details Definition of dummy data.
 */
#define DUMMY             0x00

static HAL_StatusTypeDef pressure20_spi_write(pressure20_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len);
static HAL_StatusTypeDef pressure20_spi_read(pressure20_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len);

void pressure20_cfg_setup(pressure20_cfg_t *cfg) 
{
    cfg->sck  = GPIO_PIN_NC;
    cfg->miso = GPIO_PIN_NC;
    cfg->mosi = GPIO_PIN_NC;
    cfg->cs   = GPIO_PIN_NC;
    cfg->int_pin = GPIO_PIN_NC;

    cfg->spi_speed   = 100000;
    cfg->spi_mode    = SPI_MODE_0;
    cfg->cs_polarity = GPIO_PIN_RESET;

    cfg->hal_sel = PRESSURE20_HAL_SEL_SPI;
}

void pressure20_drv_interface_selection(pressure20_cfg_t *cfg, pressure20_hal_t hal_sel) 
{
    cfg->hal_sel = hal_sel;
}

HAL_StatusTypeDef pressure20_init(pressure20_t *ctx, pressure20_cfg_t *cfg) 
{
    ctx->hal_sel = cfg->hal_sel;
    ctx->spi.Instance = cfg->spi.Instance;
    ctx->spi.Init.Mode = SPI_MODE_MASTER;
    ctx->spi.Init.Direction = SPI_DIRECTION_2LINES;
    ctx->spi.Init.DataSize = SPI_DATASIZE_8BIT;
    ctx->spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    ctx->spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    ctx->spi.Init.NSS = SPI_NSS_SOFT;
    ctx->spi.Init.BaudRatePrescaler = cfg->spi_speed;
    ctx->spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    ctx->spi.Init.TIMode = SPI_TIMODE_DISABLE;
    ctx->spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    ctx->spi.Init.CRCPolynomial = 7;
    ctx->spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    ctx->spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

    if (HAL_SPI_Init(&(ctx->spi)) != HAL_OK) 
    {
        return HAL_ERROR;
    }

    ctx->read_f = pressure20_spi_read;
    ctx->write_f = pressure20_spi_write;
    return HAL_OK;
}

HAL_StatusTypeDef pressure20_default_cfg(pressure20_t *ctx) 
{
    HAL_StatusTypeDef error_flag = HAL_OK;
    uint8_t reg_data;

    if (HAL_ERROR == pressure20_check_communication(ctx))
    {
        return HAL_ERROR;
    }

    error_flag |= pressure20_write_mode_select(ctx, PRESSURE20_POWER_MODE_NORMAL);
    HAL_Delay(5);
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_FIFO_FILL, &PRESSURE20_FIFO_FLUSH, 1);
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_FIFO_CONFIG, &PRESSURE20_FIFO_LEVEL_EMPTY, 1);
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_INTERRUPT_MASK, &PRESSURE20_INT_MASK_ALL, 1);
    error_flag |= pressure20_clear_interrupts(ctx);
    error_flag |= pressure20_spi_read(ctx, PRESSURE20_REG_OTP_STATUS2, &reg_data, 1);

    if (PRESSURE20_BOOT_UP_STATUS != (reg_data & PRESSURE20_BOOT_UP_STATUS))
    {
        error_flag |= pressure20_write_mode_select(ctx, PRESSURE20_POWER_MODE_ACTIVE);
        HAL_Delay(5); 
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_MASTER_LOCK, &PRESSURE20_MASTER_UNLOCK, 1);
        uint8_t otp_config = PRESSURE20_OTP_WRITE_SWITCH | PRESSURE20_OTP_ENABLE;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_CONFIG1, &otp_config, 1);
        HAL_Delay(1); 
        uint8_t otp_reset_set = PRESSURE20_OTP_RESET_SET;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_DBG2, &otp_reset_set, 1);
        HAL_Delay(1); 
        uint8_t otp_reset_clear = PRESSURE20_OTP_RESET_CLEAR;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_DBG2, &otp_reset_clear, 1);
        HAL_Delay(1); 
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MRA_LSB, &PRESSURE20_OTP_MRA_LSB, 1);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MRA_MSB, &PRESSURE20_OTP_MRA_MSB, 1);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MRB_LSB, &PRESSURE20_OTP_MRB_LSB, 1);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MRB_MSB, &PRESSURE20_OTP_MRB_MSB, 1);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MR_LSB, &PRESSURE20_OTP_MR_LSB, 1);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_MR_MSB, &PRESSURE20_OTP_MR_MSB, 1);

        uint8_t offset, gain, hfosc;
        error_flag |= pressure20_read_otp_data(ctx, PRESSURE20_OTP_ADDRESS_OFFSET, &offset);
        error_flag |= pressure20_read_otp_data(ctx, PRESSURE20_OTP_ADDRESS_GAIN, &gain);
        error_flag |= pressure20_read_otp_data(ctx, PRESSURE20_OTP_ADDRESS_HFOSC, &hfosc);

        uint8_t otp_disable = PRESSURE20_OTP_DISABLE;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_CONFIG1, &otp_disable, 1);

        uint8_t trim1_msb = (offset & PRESSURE20_TRIM1_MSB_PEFE_OFFSET_MASK);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_TRIM1_MSB, &trim1_msb, 1);
        error_flag |= pressure20_spi_read(ctx, PRESSURE20_REG_TRIM2_MSB, &reg_data, 1);
        reg_data = (reg_data & (~PRESSURE20_TRIM2_MSB_PEFE_GAIN_MASK)) | ((gain << 4) & PRESSURE20_TRIM2_MSB_PEFE_GAIN_MASK);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_TRIM2_MSB, &reg_data, 1);
        uint8_t trim2_lsb = (hfosc & PRESSURE20_TRIM2_LSB_HFOSC_MASK);
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_TRIM2_LSB, &trim2_lsb, 1);

        uint8_t master_lock = PRESSURE20_MASTER_LOCK;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_MASTER_LOCK, &master_lock, 1);

        error_flag |= pressure20_write_mode_select(ctx, PRESSURE20_POWER_MODE_NORMAL);
        HAL_Delay(5); 
        uint8_t boot_up_status = PRESSURE20_BOOT_UP_STATUS;
        error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_OTP_STATUS2, &boot_up_status, 1);
    }

    uint8_t meas_mode = PRESSURE20_MEAS_MODE_CONTINUOUS | PRESSURE20_MEAS_CONFIG_MODE0_ODR_25HZ;
    error_flag |= pressure20_write_mode_select(ctx, meas_mode);
    HAL_Delay(5); 

    uint8_t fifo_flush = PRESSURE20_FIFO_FLUSH;
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_FIFO_FILL, &fifo_flush, 1);

    do 
    {
        HAL_Delay(100);
        error_flag |= pressure20_spi_read(ctx, PRESSURE20_REG_FIFO_FILL, &reg_data, 1);
    } while (PRESSURE20_FIFO_FULL != (reg_data & PRESSURE20_FIFO_FULL));

    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_FIFO_FILL, &fifo_flush, 1);

    uint8_t fifo_config = PRESSURE20_FIFO_LEVEL_1 << 4;
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_FIFO_CONFIG, &fifo_config, 1);

    uint8_t int_mask = (~PRESSURE20_INT_MASK_FIFO_WMK_HIGH);
    error_flag |= pressure20_spi_write(ctx, PRESSURE20_REG_INTERRUPT_MASK, &int_mask, 1);
    error_flag |= pressure20_clear_interrupts(ctx);
    
    return error_flag;
}

HAL_StatusTypeDef pressure20_generic_write(pressure20_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len)
{
    return ctx->write_f(ctx, reg, data_in, len);
}

HAL_StatusTypeDef pressure20_generic_read(pressure20_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len)
{
    return ctx->read_f(ctx, reg, data_out, len);
}

HAL_StatusTypeDef pressure20_write_register(pressure20_t *ctx, uint8_t reg, uint8_t data_in)
{
    return pressure20_generic_write(ctx, reg, &data_in, 1);
}

HAL_StatusTypeDef pressure20_read_register(pressure20_t *ctx, uint8_t reg, uint8_t *data_out)
{
    return pressure20_generic_read(ctx, reg, data_out, 1);
}

HAL_StatusTypeDef pressure20_check_communication(pressure20_t *ctx)
{
    HAL_StatusTypeDef status;
    uint8_t temp;

    status = pressure20_read_register(ctx, PRESSURE20_REG_DEVICE_ID, &temp);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    if (PRESSURE20_DEVICE_ID != temp)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef pressure20_write_mode_select(pressure20_t *ctx, uint8_t mode_select)
{
    return pressure20_write_register(ctx, PRESSURE20_REG_MODE_SELECT, mode_select);
}

HAL_StatusTypeDef pressure20_read_otp_data(pressure20_t *ctx, uint8_t otp_address, uint8_t *data_out)
{
    HAL_StatusTypeDef status;
    uint8_t command = PRESSURE20_OTP_COMMAND_READ_ACTION;

    status = pressure20_write_register(ctx, PRESSURE20_REG_OTP_ADDRESS_REG, otp_address);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    status = pressure20_write_register(ctx, PRESSURE20_REG_OTP_COMMAND_REG, command);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    status = pressure20_read_register(ctx, PRESSURE20_REG_OTP_STATUS, data_out);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    if (PRESSURE20_OTP_ACTION_STATUS_DONE != (*data_out & PRESSURE20_OTP_ACTION_STATUS_DONE))
    {
        return HAL_ERROR;
    }

    status = pressure20_read_register(ctx, PRESSURE20_REG_OTP_RDATA, data_out);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef pressure20_read_data(pressure20_t *ctx, float *pressure, float *temperature)
{
    HAL_StatusTypeDef status;
    uint8_t press_data[3];
    uint8_t temp_data[3];
    int32_t press_raw;
    int32_t temp_raw;

    status = pressure20_spi_read(ctx, PRESSURE20_REG_PRESS_DATA_0, press_data, 3);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    status = pressure20_spi_read(ctx, PRESSURE20_REG_TEMP_DATA_0, temp_data, 3);
    if (HAL_OK != status)
    {
        return HAL_ERROR;
    }

    press_raw = (int32_t)press_data[2];
    press_raw |= (int32_t)press_data[1] << 8;
    press_raw |= (int32_t)press_data[0] << 16;
    if (press_raw & 0x800000)
    {
        press_raw |= 0xFF000000;
    }

    *pressure = (float)press_raw / 4096.0;

    temp_raw = (int32_t)temp_data[2];
    temp_raw |= (int32_t)temp_data[1] << 8;
    temp_raw |= (int32_t)temp_data[0] << 16;
    if (temp_raw & 0x800000)
    {
        temp_raw |= 0xFF000000;
    }

    *temperature = (float)temp_raw / 5120.0;

    return HAL_OK;
}

uint8_t pressure20_get_int_pin(pressure20_t *ctx)
{
    return HAL_GPIO_ReadPin(ctx->int_pin.port, ctx->int_pin.pin);
}

HAL_StatusTypeDef pressure20_clear_interrupts(pressure20_t *ctx)
{
    uint8_t int_status;
    return pressure20_spi_read(ctx, PRESSURE20_REG_INTERRUPT_STATUS, &int_status, 1);
}

static HAL_StatusTypeDef pressure20_spi_write(pressure20_t *ctx, uint8_t reg, uint8_t *data_in, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[1 + len];

    tx_buf[0] = reg;
    memcpy(&tx_buf[1], data_in, len);

    HAL_GPIO_WritePin(ctx->cs.port, ctx->cs.pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&ctx->spi, tx_buf, 1 + len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ctx->cs.port, ctx->cs.pin, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef pressure20_spi_read(pressure20_t *ctx, uint8_t reg, uint8_t *data_out, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[1];
    uint8_t rx_buf[len];

    tx_buf[0] = reg | 0x80;

    HAL_GPIO_WritePin(ctx->cs.port, ctx->cs.pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&ctx->spi, tx_buf, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(ctx->cs.port, ctx->cs.pin, GPIO_PIN_SET);
        return status;
    }

    status = HAL_SPI_Receive(&ctx->spi, rx_buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ctx->cs.port, ctx->cs.pin, GPIO_PIN_SET);

    memcpy(data_out, rx_buf, len);

    return status;
}
