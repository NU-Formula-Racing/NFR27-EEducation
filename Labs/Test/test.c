#include "test.h"
#include "can_lib.h"
#include <stdio.h>
#include <stdlib.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern SMBUS_HandleTypeDef hsmbus1;
extern SPI_HandleTypeDef hspi1;

int __io_putchar(int ch) {
  ITM_SendChar((uint32_t)ch);
  return ch;
}

// Latest sensor readings. test_imu() / test_adc() fill these in and the CAN
// messages built in test_can() send whatever is here.
static int32_t adc_code;    // raw 24-bit result, sign-extended
static int16_t adc_mv;      // adc_code scaled to millivolts
static int16_t imu_acc[3];  // raw, 4096 LSB/g (+/-8 g)
static int16_t imu_gyr[3];  // raw, 16.4 LSB/dps (+/-2000 dps)
static int16_t imu_temp;    // raw, degC = raw / 512 + 23

// ---------------------------------------------------------------- board
void test_led(void) {
  HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
  HAL_Delay(500);
}

// ---------------------------------------------------------------- adc

// MCP3561R (U3) on SPI1. CH0 is FSR_SENSE, measured against AGND using the
// internal 2.4 V reference. Every SPI transaction starts with a command byte,
// and the chip shifts its STATUS byte back while that byte goes out.
#define MCP_DEV_ADDR 0x1  // factory default for MCP3561R-E/ST
#define MCP_VREF_MV 2400

#define MCP_CMD(reg, type) ((MCP_DEV_ADDR << 6) | ((reg) << 2) | (type))
#define MCP_FAST 0x0  // fast command, reg field holds the command code
#define MCP_STATIC_READ 0x1
#define MCP_INC_WRITE 0x2
#define MCP_INC_READ 0x3

#define MCP_REG_ADCDATA 0x0
#define MCP_REG_CONFIG0 0x1
#define MCP_REG_RESERVED_E 0xE  // low nibble = chip ID
#define MCP_FAST_RESET 0xE

#define MCP_CHIP_ID 0xC

// STATUS = 0 0 A1 A0 !A0 DR CRCCFG POR, the last three active low
#define MCP_STATUS_ADDR(s) (((s) >> 3) & 0x7)
#define MCP_STATUS_ADDR_EXPECTED (((MCP_DEV_ADDR) << 1) | (~(MCP_DEV_ADDR) & 1))
#define MCP_STATUS_DR 0x04

// CONFIG0..MUX, written in one incremental write
static const uint8_t mcp_config[] = {
    0xE3,  // CONFIG0: internal VREF, internal clock (MCLK not driven), conversion mode
    0x0C,  // CONFIG1: no prescaler, OSR 256
    0x8B,  // CONFIG2: boost 1x, gain 1x, auto-zero off (default)
    0xC0,  // CONFIG3: continuous conversion, 24-bit data
    0x06,  // IRQ: IRQ pin push-pull idle high, fast commands on, no start IRQ
    0x08,  // MUX: VIN+ = CH0, VIN- = AGND
};
// IRQ bits 6:4 are read-only flags, so ignore them on readback
static const uint8_t mcp_readback_mask[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF};

static HAL_StatusTypeDef mcp_xfer(uint8_t *tx, uint8_t *rx, uint16_t len) {
  return HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 10);
}

static int mcp_status_ok(uint8_t status) {
  return MCP_STATUS_ADDR(status) == MCP_STATUS_ADDR_EXPECTED;
}

static int adc_init(void) {
  uint8_t tx[1 + sizeof(mcp_config)] = {0};
  uint8_t rx[1 + sizeof(mcp_config)] = {0};

  tx[0] = MCP_CMD(MCP_FAST_RESET, MCP_FAST);
  if (mcp_xfer(tx, rx, 1) != HAL_OK) {
    printf("ADC: SPI transfer failed (err 0x%lx)\r\n", HAL_SPI_GetError(&hspi1));
    return -1;
  }
  HAL_Delay(1);

  tx[0] = MCP_CMD(MCP_REG_RESERVED_E, MCP_STATIC_READ);
  tx[1] = tx[2] = 0;
  mcp_xfer(tx, rx, 3);
  if (!mcp_status_ok(rx[0])) {
    printf("ADC: no response (status 0x%02x), check U3 power/soldering\r\n",
           rx[0]);
    return -1;
  }
  if ((rx[2] & 0x0F) != MCP_CHIP_ID) {
    printf("ADC: unexpected chip id 0x%02x%02x\r\n", rx[1], rx[2]);
    return -1;
  }

  tx[0] = MCP_CMD(MCP_REG_CONFIG0, MCP_INC_WRITE);
  for (uint32_t i = 0; i < sizeof(mcp_config); i++) {
    tx[1 + i] = mcp_config[i];
  }
  mcp_xfer(tx, rx, sizeof(tx));

  tx[0] = MCP_CMD(MCP_REG_CONFIG0, MCP_INC_READ);
  for (uint32_t i = 0; i < sizeof(mcp_config); i++) {
    tx[1 + i] = 0;
  }
  mcp_xfer(tx, rx, sizeof(tx));
  for (uint32_t i = 0; i < sizeof(mcp_config); i++) {
    if ((rx[1 + i] & mcp_readback_mask[i]) !=
        (mcp_config[i] & mcp_readback_mask[i])) {
      printf("ADC: config readback mismatch at reg %lu: 0x%02x != 0x%02x\r\n",
             MCP_REG_CONFIG0 + i, rx[1 + i], mcp_config[i]);
      return -1;
    }
  }

  printf("ADC: MCP3561R ready\r\n");
  return 0;
}

// Reads the latest CH0 conversion and prints it. The chip converts
// continuously, so a read without a fresh result returns the previous one.
void test_adc(void) {
  static uint8_t ready = 0;

  if (!ready) {
    if (adc_init() != 0) {
      return;
    }
    ready = 1;
  }

  uint8_t tx[4] = {MCP_CMD(MCP_REG_ADCDATA, MCP_STATIC_READ), 0, 0, 0};
  uint8_t rx[4] = {0};
  if (mcp_xfer(tx, rx, sizeof(tx)) != HAL_OK || !mcp_status_ok(rx[0])) {
    printf("ADC: read failed (status 0x%02x), re-initializing\r\n", rx[0]);
    ready = 0;
    return;
  }

  adc_code = (int32_t)(((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) |
                       ((uint32_t)rx[3] << 8)) >> 8;
  adc_mv = (int16_t)(((int64_t)adc_code * MCP_VREF_MV) >> 23);

  printf("ADC: CH0 code=%ld %d mV%s IRQ=%d\r\n", adc_code, adc_mv,
         (rx[0] & MCP_STATUS_DR) ? " (stale)" : "",
         HAL_GPIO_ReadPin(ADC_IRQ_GPIO_Port, ADC_IRQ_Pin));
}

// ---------------------------------------------------------------- imu

// BMI323 (U4) on I2C1. Registers are 16 bits wide and little-endian, and
// every I2C read returns two dummy bytes before the data.
//
// I2C1 is generated as an SMBus device, and the SMBus HAL only offers
// interrupt-driven transfers (no I2C1 IRQ is enabled), so these helpers drive
// the I2C1 registers directly on top of the timing CubeMX already set up.
#define IMU_I2C (hsmbus1.Instance)
#define IMU_I2C_TIMEOUT_MS 5

#define BMI_ADDR_SDO_LOW 0x68
#define BMI_ADDR_SDO_HIGH 0x69
#define BMI_CHIP_ID 0x43

#define BMI_REG_CHIP_ID 0x00
#define BMI_REG_ERR 0x01
#define BMI_REG_ACC_X 0x03  // ACC X/Y/Z, GYR X/Y/Z, TEMP follow in order
#define BMI_REG_ACC_CONF 0x20
#define BMI_REG_GYR_CONF 0x21
#define BMI_REG_CMD 0x7E

#define BMI_CMD_SOFT_RESET 0xDEAF
#define BMI_ACC_CONF 0x4028  // normal mode, +/-8 g, 100 Hz
#define BMI_GYR_CONF 0x4048  // normal mode, +/-2000 dps, 100 Hz
#define BMI_ERR_CONF 0x0060  // acc_conf_err | gyr_conf_err
#define BMI_INVALID ((int16_t)0x8000)

static uint8_t imu_addr;

// Resets the peripheral so a stuck transfer can't block the next one
static void i2c_recover(void) {
  IMU_I2C->CR1 &= ~I2C_CR1_PE;
  (void)IMU_I2C->CR1;
  IMU_I2C->CR1 |= I2C_CR1_PE;
}

static int i2c_wait(uint32_t flag) {
  uint32_t start = HAL_GetTick();
  while (!(IMU_I2C->ISR & flag)) {
    if (IMU_I2C->ISR & I2C_ISR_NACKF) {
      // hardware sends STOP by itself after a NACK
      while (!(IMU_I2C->ISR & I2C_ISR_STOPF) &&
             (HAL_GetTick() - start) <= IMU_I2C_TIMEOUT_MS) {
      }
      IMU_I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
      return -1;
    }
    if ((HAL_GetTick() - start) > IMU_I2C_TIMEOUT_MS) {
      i2c_recover();
      return -1;
    }
  }
  return 0;
}

static void i2c_start(uint8_t addr, uint8_t nbytes, uint8_t read,
                      uint8_t autoend) {
  IMU_I2C->CR2 = ((uint32_t)(addr << 1) & I2C_CR2_SADD) |
                 ((uint32_t)nbytes << I2C_CR2_NBYTES_Pos) |
                 (read ? I2C_CR2_RD_WRN : 0) | (autoend ? I2C_CR2_AUTOEND : 0) |
                 I2C_CR2_START;
}

static int i2c_stop_done(void) {
  if (i2c_wait(I2C_ISR_STOPF) != 0) {
    return -1;
  }
  IMU_I2C->ICR = I2C_ICR_STOPCF;
  return 0;
}

static int bmi_write(uint8_t reg, uint16_t value) {
  uint8_t buf[3] = {reg, (uint8_t)value, (uint8_t)(value >> 8)};

  i2c_start(imu_addr, sizeof(buf), 0, 1);
  for (uint32_t i = 0; i < sizeof(buf); i++) {
    if (i2c_wait(I2C_ISR_TXIS) != 0) {
      return -1;
    }
    IMU_I2C->TXDR = buf[i];
  }
  return i2c_stop_done();
}

// Reads count consecutive 16-bit registers starting at reg
static int bmi_read(uint8_t reg, uint16_t *out, uint8_t count) {
  uint8_t buf[2 + 2 * 7];
  uint8_t len = 2 + 2 * count;

  if (len > sizeof(buf)) {
    return -1;
  }

  i2c_start(imu_addr, 1, 0, 0);
  if (i2c_wait(I2C_ISR_TXIS) != 0) {
    return -1;
  }
  IMU_I2C->TXDR = reg;
  if (i2c_wait(I2C_ISR_TC) != 0) {
    return -1;
  }

  i2c_start(imu_addr, len, 1, 1);  // repeated start
  for (uint8_t i = 0; i < len; i++) {
    if (i2c_wait(I2C_ISR_RXNE) != 0) {
      return -1;
    }
    buf[i] = (uint8_t)IMU_I2C->RXDR;
  }
  if (i2c_stop_done() != 0) {
    return -1;
  }

  for (uint8_t i = 0; i < count; i++) {
    out[i] = (uint16_t)(buf[2 + 2 * i] | (buf[3 + 2 * i] << 8));
  }
  return 0;
}

static int imu_init(void) {
  uint16_t val;

  // SMBus device mode with ALERTEN set holds SMBA (PB5) low; release it
  IMU_I2C->CR1 &= ~I2C_CR1_ALERTEN;

  imu_addr = 0;
  if (HAL_SMBUS_IsDeviceReady(&hsmbus1, BMI_ADDR_SDO_LOW << 1, 2, 2) == HAL_OK) {
    imu_addr = BMI_ADDR_SDO_LOW;
  } else if (HAL_SMBUS_IsDeviceReady(&hsmbus1, BMI_ADDR_SDO_HIGH << 1, 2, 2) ==
             HAL_OK) {
    imu_addr = BMI_ADDR_SDO_HIGH;
  } else {
    printf("IMU: no ACK at 0x%02x or 0x%02x\r\n", BMI_ADDR_SDO_LOW,
           BMI_ADDR_SDO_HIGH);
    return -1;
  }

  if (bmi_write(BMI_REG_CMD, BMI_CMD_SOFT_RESET) != 0) {
    printf("IMU: soft reset failed\r\n");
    return -1;
  }
  HAL_Delay(2);

  if (bmi_read(BMI_REG_CHIP_ID, &val, 1) != 0 || (val & 0xFF) != BMI_CHIP_ID) {
    printf("IMU: unexpected chip id 0x%04x at 0x%02x\r\n", val, imu_addr);
    return -1;
  }

  if (bmi_write(BMI_REG_ACC_CONF, BMI_ACC_CONF) != 0 ||
      bmi_write(BMI_REG_GYR_CONF, BMI_GYR_CONF) != 0) {
    printf("IMU: config write failed\r\n");
    return -1;
  }
  HAL_Delay(50);  // first samples arrive after a few ODR periods

  if (bmi_read(BMI_REG_ERR, &val, 1) != 0 || (val & BMI_ERR_CONF)) {
    printf("IMU: config rejected (ERR_REG 0x%04x)\r\n", val);
    return -1;
  }

  printf("IMU: BMI323 ready at 0x%02x\r\n", imu_addr);
  return 0;
}

// Reads accel, gyro and temperature and prints them in physical units
void test_imu(void) {
  static uint8_t ready = 0;
  uint16_t raw[7];

  if (!ready) {
    if (imu_init() != 0) {
      return;
    }
    ready = 1;
  }

  if (bmi_read(BMI_REG_ACC_X, raw, 7) != 0) {
    printf("IMU: read failed, re-initializing\r\n");
    ready = 0;
    return;
  }
  if ((int16_t)raw[0] == BMI_INVALID || (int16_t)raw[3] == BMI_INVALID) {
    printf("IMU: no valid sample yet\r\n");
    return;
  }

  for (int i = 0; i < 3; i++) {
    imu_acc[i] = (int16_t)raw[i];
    imu_gyr[i] = (int16_t)raw[3 + i];
  }
  imu_temp = (int16_t)raw[6];

  int32_t temp_cc = (int32_t)imu_temp * 100 / 512 + 2300;
  printf("IMU: acc[mg] %ld %ld %ld  gyr[mdps] %ld %ld %ld  temp %ld.%02ld C\r\n",
         (int32_t)imu_acc[0] * 1000 / 4096, (int32_t)imu_acc[1] * 1000 / 4096,
         (int32_t)imu_acc[2] * 1000 / 4096, (int32_t)imu_gyr[0] * 10000 / 164,
         (int32_t)imu_gyr[1] * 10000 / 164, (int32_t)imu_gyr[2] * 10000 / 164,
         temp_cc / 100, labs(temp_cc % 100));
}

// ---------------------------------------------------------------- can

static uint8_t can_counter = 0;

static void put_le16(uint8_t *dst, int16_t v) {
  dst[0] = (uint8_t)v;
  dst[1] = (uint8_t)((uint16_t)v >> 8);
}

static void construct_test_message(message_header_t *msg) {
  msg->TxData_[0] = can_counter++;
  msg->TxData_[1] = (uint8_t)(HAL_GetTick() >> 0);
  msg->TxData_[2] = (uint8_t)(HAL_GetTick() >> 8);
  msg->TxData_[3] = (uint8_t)(HAL_GetTick() >> 16);
  msg->TxData_[4] = (uint8_t)(HAL_GetTick() >> 24);
}

// [0..3] raw code (int32 LE), [4..5] millivolts (int16 LE)
static void construct_adc_message(message_header_t *msg) {
  msg->TxData_[0] = (uint8_t)adc_code;
  msg->TxData_[1] = (uint8_t)(adc_code >> 8);
  msg->TxData_[2] = (uint8_t)(adc_code >> 16);
  msg->TxData_[3] = (uint8_t)(adc_code >> 24);
  put_le16(&msg->TxData_[4], adc_mv);
}

// [0..5] raw accel X/Y/Z (int16 LE, 4096 LSB/g)
static void construct_imu_acc_message(message_header_t *msg) {
  for (int i = 0; i < 3; i++) {
    put_le16(&msg->TxData_[2 * i], imu_acc[i]);
  }
}

// [0..5] raw gyro X/Y/Z (int16 LE, 16.4 LSB/dps), [6..7] raw temperature
static void construct_imu_gyr_message(message_header_t *msg) {
  for (int i = 0; i < 3; i++) {
    put_le16(&msg->TxData_[2 * i], imu_gyr[i]);
  }
  put_le16(&msg->TxData_[6], imu_temp);
}

static void add_test_message(tx_messages_t *tx_msgs, char *name, uint32_t id,
                             uint32_t dlc,
                             void (*construct)(message_header_t *msg)) {
  message_header_t msg = {0};
  msg.name = name;
  msg.header.TxHeader_.Identifier = id;
  msg.header.TxHeader_.IdType = FDCAN_STANDARD_ID;
  msg.header.TxHeader_.TxFrameType = FDCAN_DATA_FRAME;
  msg.header.TxHeader_.DataLength = dlc;
  msg.header.TxHeader_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  msg.header.TxHeader_.BitRateSwitch = FDCAN_BRS_OFF;
  msg.header.TxHeader_.FDFormat = FDCAN_CLASSIC_CAN;
  msg.header.TxHeader_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  msg.header.TxHeader_.MessageMarker = 0;
  msg.handler.construct_message = construct;
  msg.speed = FAST;
  add_cantx_message(tx_msgs, msg);
}

// Sends the heartbeat counter plus the latest ADC and IMU readings, then
// prints every frame waiting in RX FIFO0. Nothing is filtered, so any traffic
// on the bus shows up. Run test_adc() / test_imu() first to refresh the data.
void test_can(void) {
  static uint8_t initialized = 0;
  static tx_messages_t tx_msgs = {0};

  if (!initialized) {
    add_test_message(&tx_msgs, "test", TEST_CAN_TX_ID, FDCAN_DLC_BYTES_8,
                     construct_test_message);
    add_test_message(&tx_msgs, "adc", TEST_CAN_ADC_ID, FDCAN_DLC_BYTES_6,
                     construct_adc_message);
    add_test_message(&tx_msgs, "imu_acc", TEST_CAN_IMU_ACC_ID,
                     FDCAN_DLC_BYTES_6, construct_imu_acc_message);
    add_test_message(&tx_msgs, "imu_gyr", TEST_CAN_IMU_GYR_ID,
                     FDCAN_DLC_BYTES_8, construct_imu_gyr_message);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
      printf("CAN: start failed (err 0x%lx)\r\n", HAL_FDCAN_GetError(&hfdcan1));
      return;
    }
    initialized = 1;
  }

  send_cantx_messages(&hfdcan1, &tx_msgs, FAST);
  printf("CAN: tx 0x%03x #%u, 0x%03x adc, 0x%03x acc, 0x%03x gyr\r\n",
         TEST_CAN_TX_ID, (uint8_t)(can_counter - 1), TEST_CAN_ADC_ID,
         TEST_CAN_IMU_ACC_ID, TEST_CAN_IMU_GYR_ID);

  FDCAN_ProtocolStatusTypeDef psr;
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr);
  if (psr.BusOff || psr.ErrorPassive) {
    printf("CAN: %s (last error %lu) - is another node ACKing?\r\n",
           psr.BusOff ? "bus-off" : "error passive", psr.LastErrorCode);
  }
  if (psr.BusOff) {
    // bus-off sets INIT; clearing it rejoins the bus after 129x11 recessive bits
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  }

  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) !=
        HAL_OK) {
      break;
    }
    printf("CAN: rx 0x%03lx [%lu]", rx_header.Identifier, rx_header.DataLength);
    for (uint32_t i = 0; i < rx_header.DataLength && i < 8; i++) {
      printf(" %02x", rx_data[i]);
    }
    printf("\r\n");
  }
}
