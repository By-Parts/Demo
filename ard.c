#include "main.h"
#include "ICP201xx.h"
#include <math.h>
#include <stdio.h>

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  ICP201xx icp_sensor;
  icp_sensor.i2c = &hi2c1;
  icp_sensor.i2c_address = ICP201xx_I2C_ADDRESS;
  icp_sensor.clk_freq = I2C_DEFAULT_CLOCK;

  if (ICP201xx_begin(&icp_sensor) != 0)
  {
    printf("ICP201xx initialization failed\n");
    while(1);
  }

  while (1)
  {
    float pressure_kP, temperature_C;
    if (ICP201xx_getData(&icp_sensor, &pressure_kP, &temperature_C) == 0)
    {
      printf("Pressure(kP): %.3f\n", pressure_kP);
      printf("Temp(C): %.1f\n", temperature_C);
    }
    HAL_Delay(1000);
  }
}

void SystemClock_Config(void)
{
  // System clock configuration code...
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // GPIO initialization code...
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;  // 400kHz I2C speed
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  while(1)
  {
    // Error handling code...
  }
}







#include "ICP201xx.h"
#include "main.h"

int ICP201xx_begin(ICP201xx *sensor)
{
  uint8_t who_am_i, icp_version;
  if (HAL_I2C_IsDeviceReady(sensor->i2c, sensor->i2c_address, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }

  // Initialize ICP201xx
  inv_icp201xx_serif_t icp_serif = {
    .context = sensor,
    .read_reg = i2c_read,
    .write_reg = i2c_write,
    .if_mode = ICP201XX_IF_I2C,
    .max_read = 2048,
    .max_write = 2048
  };

  int rc = inv_icp201xx_init(&sensor->icp_device, &icp_serif);
  if (rc != INV_ERROR_SUCCESS)
  {
    return rc;
  }

  rc = inv_icp201xx_soft_reset(&sensor->icp_device);
  if (rc != INV_ERROR_SUCCESS)
  {
    return rc;
  }

  rc = inv_icp201xx_get_devid_version(&sensor->icp_device, &who_am_i, &icp_version);
  if (rc != 0 || who_am_i != EXPECTED_DEVICE_ID)
  {
    return -2;
  }

  rc = inv_icp201xx_OTP_bootup_cfg(&sensor->icp_device);
  return rc;
}

int ICP201xx_getData(ICP201xx *sensor, float *pressure, float *temperature)
{
  uint8_t fifo_packets, fifo_data[6];
  int32_t data_press, data_temp;

  if (inv_icp201xx_get_fifo_count(&sensor->icp_device, &fifo_packets) != 0)
  {
    return -2;
  }

  if (fifo_packets)
  {
    inv_icp201xx_get_fifo_data(&sensor->icp_device, 1, fifo_data);
    inv_icp201xx_process_raw_data(&sensor->icp_device, 1, fifo_data, &data_press, &data_temp);

    *pressure = ((float)data_press * 40 / 131072) + 70;
    *pressure = round(*pressure * 1000.0) / 1000.0;
    *temperature = ((float)data_temp * 65 / 262144) + 25;
    *temperature = round(*temperature * 10.0) / 10.0;

    return 0;
  }
  return -1;
}

static int i2c_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  ICP201xx* sensor = (ICP201xx*)ctx;
  uint8_t data[wlen + 1];
  data[0] = reg;
  memcpy(&data[1], wbuffer, wlen);

  if (HAL_I2C_Master_Transmit(sensor->i2c, sensor->i2c_address, data, wlen + 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

static int i2c_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  ICP201xx* sensor = (ICP201xx*)ctx;
  if (HAL_I2C_Master_Transmit(sensor->i2c, sensor->i2c_address, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }
  if (HAL_I2C_Master_Receive(sensor->i2c, sensor->i2c_address, rbuffer, rlen, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }
  return 0;
}




#ifndef ICP201XX_H
#define ICP201XX_H

#include "stm32l4xx_hal.h"
#include "inv_icp201xx.h"

#define ICP201xx_I2C_ADDRESS 0x63
#define I2C_DEFAULT_CLOCK 400000

typedef struct
{
  I2C_HandleTypeDef *i2c;
  SPI_HandleTypeDef *spi;
  uint8_t i2c_address;
  uint8_t spi_cs;
  bool use_spi;
  uint32_t clk_freq;
  inv_icp201xx_t icp_device;
} ICP201xx;

int ICP201xx_begin(ICP201xx *sensor);
int ICP201xx_getData(ICP201xx *sensor, float *pressure, float *temperature);

#endif // ICP201XX_H




#include "stm32l4xx_hal.h"

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    // Initialization Error
    Error_Handler();
  }
}




#include "ICP201xx.h"

// Define the SPI CS pin
#define SPI_CS_PIN GPIO_PIN_4
#define SPI_CS_GPIO_PORT GPIOA

// ICP201xx instance
ICP201xx icp;

// SPI write function
static int spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)wbuffer, wlen, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET);
  return 0;
}

// SPI read function
static int spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  reg |= 0x80; // Set read bit
  HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, rbuffer, rlen, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET);
  return 0;
}

// Initialize the ICP201xx
int ICP201xx_Init(ICP201xx * icp) {
  inv_icp201xx_serif_t icp_serif;

  icp_serif.context = (void*)icp;
  icp_serif.if_mode = ICP201XX_IF_4_WIRE_SPI;
  icp_serif.read_reg = spi_read;
  icp_serif.write_reg = spi_write;
  icp_serif.max_read = 2048;
  icp_serif.max_write = 2048;

  int rc = inv_icp201xx_init(&icp->icp_device, &icp_serif);
  if (rc != INV_ERROR_SUCCESS) {
    return rc;
  }

  rc = inv_icp201xx_soft_reset(&icp->icp_device);
  if (rc != INV_ERROR_SUCCESS) {
    return rc;
  }

  uint8_t who_am_i;
  uint8_t icp_version;
  rc = inv_icp201xx_get_devid_version(&icp->icp_device, &who_am_i, &icp_version);
  if (rc != 0 || who_am_i != EXPECTED_DEVICE_ID) {
    return -1;
  }

  rc = inv_icp201xx_OTP_bootup_cfg(&icp->icp_device);
  if (rc != 0) {
    return rc;
  }

  return 0;
}




#include "main.h"
#include "ICP201xx.h"

// Function prototypes
void SystemClock_Config(void);
void Error_Handler(void);

int main(void) {
  // Initialize the HAL Library
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize all configured peripherals
  MX_GPIO_Init();
  MX_SPI1_Init();

  // Initialize the ICP201xx sensor
  int ret = ICP201xx_Init(&icp);
  if (ret != 0) {
    // Initialization failed
    Error_Handler();
  }

  // Start the sensor
  ret = ICP201xx_start(&icp);
  if (ret != 0) {
    // Start failed
    Error_Handler();
  }

  float pressure_kp = 0;
  float temperature_C = 0;

  // Main loop
  while (1) {
    if (ICP201xx_getData(&icp, &pressure_kp, &temperature_C) == 0) {
      // Process the data
      printf("Pressure(kP): %.2f, Temp(C): %.2f\n", pressure_kp, temperature_C);
    }

    HAL_Delay(1000);
  }
}





void Error_Handler(void) {
  // User can add his own implementation to report the HAL error return state
  while (1) {
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}




#include "main.h"
#include "stm32l4xx_hal.h"

// Define the SPI handle
SPI_HandleTypeDef hspi1;

// Define the ICP201xx instance
typedef struct {
    inv_icp201xx_device icp_device;
} ICP201xx;

ICP201xx icp;

// Define the SPI CS pin
#define SPI_CS_PIN GPIO_PIN_4
#define SPI_CS_GPIO_PORT GPIOA

// SPI write function
static int spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
    HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)wbuffer, wlen, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET);
    return 0;
}

// SPI read function
static int spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
    reg |= 0x80; // Set read bit
    HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, rbuffer, rlen, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET);
    return 0;
}

// ICP201xx initialization function
int ICP201xx_Init(ICP201xx * icp) {
    inv_icp201xx_serif_t icp_serif;

    icp_serif.context = (void*)icp;
    icp_serif.if_mode = ICP201XX_IF_4_WIRE_SPI;
    icp_serif.read_reg = spi_read;
    icp_serif.write_reg = spi_write;
    icp_serif.max_read = 2048;
    icp_serif.max_write = 2048;

    int rc = inv_icp201xx_init(&icp->icp_device, &icp_serif);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    rc = inv_icp201xx_soft_reset(&icp->icp_device);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    uint8_t who_am_i;
    uint8_t icp_version;
    rc = inv_icp201xx_get_devid_version(&icp->icp_device, &who_am_i, &icp_version);
    if (rc != 0 || who_am_i != EXPECTED_DEVICE_ID) {
        return -1;
    }

    rc = inv_icp201xx_OTP_bootup_cfg(&icp->icp_device);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

// Function to read pressure and temperature
int ICP201xx_getData(ICP201xx * icp, float * pressure_kp, float * temperature_C) {
    uint8_t data[4];
    int rc = spi_read(icp, 0xFA, data, 4);
    if (rc != 0) {
        return rc;
    }

    uint16_t pressure_raw = (data[0] << 8) | data[1];
    uint16_t temperature_raw = (data[2] << 8) | data[3];

    *pressure_kp = (float)pressure_raw / 100.0;
    *temperature_C = (float)temperature_raw / 100.0;

    return 0;
}

// SPI1 initialization function
void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }
}

// Error handler function
void Error_Handler(void) {
    while (1) {
    }
}

// System clock configuration function
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

// GPIO initialization function
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET);

    // Configure GPIO pin : SPI_CS_PIN
    GPIO_InitStruct.Pin = SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI_CS_GPIO_PORT, &GPIO_InitStruct);
}

int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_SPI1_Init();

    // Initialize the ICP201xx sensor
    int ret = ICP201xx_Init(&icp);
    if (ret != 0) {
        // Initialization failed
        Error_Handler();
    }

    float pressure_kp = 0;
    float temperature_C = 0;

    // Main loop
    while (1) {
        if (ICP201xx_getData(&icp, &pressure_kp, &temperature_C) == 0) {
            // Process the data
            printf("Pressure(kP): %.2f, Temp(C): %.2f\n", pressure_kp, temperature_C);
        }

        HAL_Delay(1000);
    }
}






// Define register addresses
#define MODE_SELECT_REG      0xC0
#define INTERRUPT_MASK_REG   0xC2
#define FIFO_CONFIG_REG      0xC3
#define INTERRUPT_STATUS_REG 0xC1
#define FIFO_BASE_REG        0xFA

// Set Mode 0
writeRegister(MODE_SELECT_REG, 0x08);

// Enable FIFO Overflow Interrupt
writeRegister(INTERRUPT_MASK_REG, 0xFE);

// Set FIFO Watermark High to 1
writeRegister(FIFO_CONFIG_REG, 0x10);

// On interrupt
uint8_t interrupt_status = readRegister(INTERRUPT_STATUS_REG);
if (interrupt_status & FIFO_OVERFLOW_INTERRUPT) {
    // Read 96 bytes (16 packets)
    uint8_t data[96];
    readRegisters(FIFO_BASE_REG, data, 96);

    // Clear interrupt status
    writeRegister(INTERRUPT_STATUS_REG, interrupt_status);
}





#include "main.h"

// Define register addresses
#define MODE_SELECT_REG      0xC0
#define INTERRUPT_MASK_REG   0xC2
#define FIFO_CONFIG_REG      0xC3
#define INTERRUPT_STATUS_REG 0xC1
#define FIFO_BASE_REG        0xFA

SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t size);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    // Set Mode 0
    writeRegister(MODE_SELECT_REG, 0x08);

    // Enable FIFO Overflow Interrupt
    writeRegister(INTERRUPT_MASK_REG, 0xFE);

    // Set FIFO Watermark High to 1
    writeRegister(FIFO_CONFIG_REG, 0x10);

    while (1)
    {
        uint8_t interrupt_status = readRegister(INTERRUPT_STATUS_REG);
        if (interrupt_status & 0x02) // Check FIFO overflow interrupt bit
        {
            // Read 96 bytes (16 packets)
            uint8_t data[96];
            readRegisters(FIFO_BASE_REG, data, 96);

            // Clear interrupt status
            writeRegister(INTERRUPT_STATUS_REG, interrupt_status);
        }
    }
}

void writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = {reg, value};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

uint8_t readRegister(uint8_t reg)
{
    uint8_t txData = reg | 0x80;
    uint8_t rxData;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
    return rxData;
}

void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t size)
{
    uint8_t txData = reg | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    while(1)
    {
        // Stay here
    }
}





#include "main.h"

// Define register addresses
#define MODE_SELECT_REG        0xC0
#define INTERRUPT_MASK_REG     0xC2
#define FIFO_CONFIG_REG        0xC3
#define INTERRUPT_STATUS_REG   0xC1
#define FIFO_BASE_REG          0xFA
#define DEVICE_STATUS_REG      0xCD

SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t size);
uint8_t checkModeSyncStatus(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    // Wait for MODE_SYNC_STATUS bit to be set
    while (!checkModeSyncStatus());

    // Set Mode 0
    writeRegister(MODE_SELECT_REG, 0x08);

    // Enable FIFO Overflow Interrupt
    writeRegister(INTERRUPT_MASK_REG, 0xFE);

    // Set FIFO Watermark High to 1
    writeRegister(FIFO_CONFIG_REG, 0x10);

    while (1)
    {
        uint8_t interrupt_status = readRegister(INTERRUPT_STATUS_REG);
        if (interrupt_status & 0x02) // Check FIFO overflow interrupt bit
        {
            // Read 96 bytes (16 packets)
            uint8_t data[96];
            readRegisters(FIFO_BASE_REG, data, 96);

            // Clear interrupt status
            writeRegister(INTERRUPT_STATUS_REG, interrupt_status);
        }
    }
}

uint8_t checkModeSyncStatus(void)
{
    uint8_t status = readRegister(DEVICE_STATUS_REG);
    return status & 0x01; // Check MODE_SYNC_STATUS bit
}

void writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = {reg, value};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

uint8_t readRegister(uint8_t reg)
{
    uint8_t txData = reg | 0x80;
    uint8_t rxData;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
    return rxData;
}

void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t size)
{
    uint8_t txData = reg | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    while(1)
    {
        // Stay here
    }
}
