/* Most of this code are derived from Microchip MCP2518FD SDK */
#include "mcp2518fd_can.h"
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <unistd.h>

static CAN_CONFIG config;

// Receive objects
static CAN_RX_FIFO_CONFIG rxConfig;
static REG_CiFLTOBJ fObj;
static REG_CiMASK mObj;
static CAN_RX_MSGOBJ rxObj;
static uint8_t rxd[MAX_DATA_BYTES];

// Transmit objects
static CAN_TX_FIFO_CONFIG txConfig;
static CAN_TX_FIFO_EVENT txFlags;
static CAN_TX_MSGOBJ txObj;
static uint8_t txd[MAX_DATA_BYTES];

#define MAX_TXQUEUE_ATTEMPTS 50

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2

// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1

// Maximum number of data bytes in message
#define MAX_DATA_BYTES 64

// *****************************************************************************
// Section: Variables
byte ext_flg; // identifier xxxID
// either extended (the 29 LSB) or standard (the 11 LSB)
unsigned long can_id; // can id
byte rtr;             // is remote frame
static byte SPICS;

static uint32_t mSysClock;   // PLL disabled, mSysClock = Oscillator Frequency
static uint32_t mDesiredArbitrationBitRate; // desired ArbitrationBitRate
static uint8_t  mDataBitRateFactor; // multiplier between ArbitrationBitRate and DataBitrate
//--- Data bit rate; if mDataBitRateFactor==1, theses properties are not used for configuring the MCP2517FD.
static uint8_t mDataPhaseSegment1 = 0 ; // if mDataBitRateFactor > 1: 2...32, else equal to mArbitrationPhaseSegment1
static uint8_t mDataPhaseSegment2 = 0 ; // if mDataBitRateFactor > 1: 1...16, else equal to mArbitrationPhaseSegment2
static uint8_t mDataSJW = 0 ; // if mDataBitRateFactor > 1: 1...16, else equal to mArbitrationSJW
//--- Bit rate prescaler is common to arbitration and data bit rates
static uint16_t mBitRatePrescaler = 0 ; // 1...256
//--- Arbitration bit rate
static uint16_t mArbitrationPhaseSegment1 = 0 ; // 2...256
static uint8_t mArbitrationPhaseSegment2 = 0 ; // 1...128
static uint8_t mArbitrationSJW = 0 ; // 1...128
static bool mArbitrationBitRateClosedToDesiredRate = false ; // The above configuration is not correct
//--- Transmitter Delay Compensation Offset
static int8_t mTDCO = 0 ; // -64 ... +63


static byte mcp2518fd_readMsgBufID(volatile byte *len, volatile byte *buf);
static byte mcp2518fd_sendMsg(const byte *buf, byte len, unsigned long id, byte ext,
		 byte rtr, bool wait_sent);

static uint8_t mcp2518fd_init(uint32_t speedset, const byte clock); // mcp2518fdinit
static int8_t mcp2518fd_reset(void);                            // reset mcp2518fd
static int8_t mcp2518fd_EccEnable(void);                        // Section: ECC
static int8_t mcp2518fd_RamInit(uint8_t d);
static int8_t mcp2518fd_ConfigureObjectReset(CAN_CONFIG *config);
static int8_t mcp2518fd_Configure(CAN_CONFIG *config);
static int8_t
mcp2518fd_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG *config);
static int8_t mcp2518fd_TransmitChannelConfigure(CAN_FIFO_CHANNEL channel,
				    CAN_TX_FIFO_CONFIG *config);
static int8_t
mcp2518fd_ReceiveChannelConfigureObjectReset(CAN_RX_FIFO_CONFIG *config);
static int8_t mcp2518fd_ReceiveChannelConfigure(CAN_FIFO_CHANNEL channel,
				   CAN_RX_FIFO_CONFIG *config);
static int8_t mcp2518fd_FilterObjectConfigure(CAN_FILTER filter,
				 CAN_FILTEROBJ_ID *id);
static int8_t mcp2518fd_FilterMaskConfigure(CAN_FILTER filter, CAN_MASKOBJ_ID *mask);
static int8_t mcp2518fd_FilterToFifoLink(CAN_FILTER filter, CAN_FIFO_CHANNEL channel,
			    bool enable);
static int8_t mcp2518fd_BitTimeConfigure(uint32_t speedset,
			    CAN_SSP_MODE sspMode, CAN_SYSCLK_SPEED clk);
static int8_t mcp2518fd_GpioModeConfigure(GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1);
static int8_t mcp2518fd_ReceiveChannelEventEnable(CAN_FIFO_CHANNEL channel,
				     CAN_RX_FIFO_EVENT flags);
static int8_t mcp2518fd_ModuleEventEnable(CAN_MODULE_EVENT flags);
static int8_t mcp2518fd_OperationModeSelect(CAN_OPERATION_MODE opMode);
static CAN_OPERATION_MODE mcp2518fd_OperationModeGet();
static void mcp2518fd_TransmitMessageQueue();
static int8_t mcp2518fd_TransmitChannelEventGet(CAN_FIFO_CHANNEL channel,
				   CAN_TX_FIFO_EVENT *flags);
static int8_t mcp2518fd_ErrorCountStateGet(uint8_t *tec, uint8_t *rec,
			      CAN_ERROR_STATE *flags);
static int8_t mcp2518fd_TransmitChannelLoad(CAN_FIFO_CHANNEL channel,
			       CAN_TX_MSGOBJ *txObj, uint8_t *txd,
			       uint32_t txdNumBytes, bool flush);
static int8_t mcp2518fd_ReceiveChannelEventGet(CAN_FIFO_CHANNEL channel,
				  CAN_RX_FIFO_EVENT *flags);
static int8_t mcp2518fd_ReceiveMessageGet(CAN_FIFO_CHANNEL channel,
			     CAN_RX_MSGOBJ *rxObj, uint8_t *rxd,
			     uint8_t nBytes);
static int8_t mcp2518fd_ReceiveChannelUpdate(CAN_FIFO_CHANNEL channel);
static int8_t mcp2518fd_TransmitChannelUpdate(CAN_FIFO_CHANNEL channel, bool flush);
static int8_t mcp2518fd_ReceiveChannelStatusGet(CAN_FIFO_CHANNEL channel,
				   CAN_RX_FIFO_STATUS *status);
static int8_t mcp2518fd_ErrorStateGet(CAN_ERROR_STATE *flags);
static int8_t mcp2518fd_TransmitChannelEventAttemptClear(CAN_FIFO_CHANNEL channel);

static int8_t mcp2518fd_LowPowerModeEnable();
static int8_t mcp2518fd_LowPowerModeDisable();

static int8_t mcp2518fd_ReadByte(uint16_t address, uint8_t *rxd);
static int8_t mcp2518fd_WriteByte(uint16_t address, uint8_t txd);
static int8_t mcp2518fd_ReadWord(uint16_t address, uint32_t *rxd);
static int8_t mcp2518fd_WriteWord(uint16_t address, uint32_t txd);
static int8_t mcp2518fd_ReadHalfWord(uint16_t address, uint16_t *rxd);
static int8_t mcp2518fd_WriteHalfWord(uint16_t address, uint16_t txd);
static int8_t mcp2518fd_ReadByteArray(uint16_t address, uint8_t *rxd,
			 uint16_t nBytes);
static int8_t mcp2518fd_WriteByteArray(uint16_t address, uint8_t *txd,
			  uint16_t nBytes);
static int8_t mcp2518fd_ReadWordArray(uint16_t address, uint32_t *rxd,
			 uint16_t nWords);

static int8_t mcp2518fd_BitTimeConfigureNominal(void);
static int8_t mcp2518fd_BitTimeConfigureData(CAN_SSP_MODE sspMode);

static CAN_OPERATION_MODE mcpMode = CAN_CLASSIC_MODE; // Current controller mode


static uint32_t bittime_compat_to_mcp2518fd(uint32_t speedset);
static int calcBittime(const uint32_t inDesiredArbitrationBitRate,
	  const uint32_t inTolerancePPM/* = 10000 1% */);

// *****************************************************************************
// *****************************************************************************
// Section: Defines

#define CRCBASE 0xFFFF
#define CRCUPPER 1

//! Reverse order of bits in byte
const uint8_t BitReverseTable256[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0,
    0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8,
    0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04, 0x84, 0x44, 0xC4,
    0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC,
    0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2,
    0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 0x0A, 0x8A, 0x4A, 0xCA,
    0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6,
    0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE,
    0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01, 0x81, 0x41, 0xC1,
    0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9,
    0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5,
    0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D, 0x8D, 0x4D, 0xCD,
    0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3,
    0x33, 0xB3, 0x73, 0xF3, 0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
    0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07, 0x87, 0x47, 0xC7,
    0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF,
    0x3F, 0xBF, 0x7F, 0xFF};

//! Look-up table for CRC calculation
const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
    0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
    0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
    0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
    0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
    0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
    0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
    0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
    0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
    0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
    0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
    0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
    0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
    0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
    0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
    0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
    0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
    0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
    0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202};


//! SPI Transmit buffer
static uint8_t spiTransmitBuffer[SPI_DEFAULT_BUFFER_LENGTH + 2];

//! SPI Receive buffer
static uint8_t spiReceiveBuffer[SPI_DEFAULT_BUFFER_LENGTH];

uint16_t DRV_CANFDSPI_CalculateCRC16(uint8_t *data, uint16_t size) {
  uint16_t init = CRCBASE;
  uint8_t index;

  while (size-- != 0) {
    index = ((uint8_t *)&init)[CRCUPPER] ^ *data++;
    init = (init << 8) ^ crc16_table[index];
  }

  return init;
}


/*********************************************************************************************************
** Function name:           begin
** Descriptions:            init can and set speed
*********************************************************************************************************/
byte MCP_begin(uint32_t speedset, const byte clockset) {
  // TODO
  // SPI.begin();

  /* compatible layer translation */
  speedset = bittime_compat_to_mcp2518fd(speedset);

  byte res = mcp2518fd_init(speedset, clockset);
  return res;
}

/*********************************************************************************************************
** Function name:           mcp2518fd_reset
** Descriptions:            reset the device
*********************************************************************************************************/
int8_t mcp2518fd_reset(void) {
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] = (uint8_t)(cINSTRUCTION_RESET << 4);
  spiTransmitBuffer[1] = 0;

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif
  usleep(10000);

  return spiTransferError;
}

int8_t mcp2518fd_ReadByte(uint16_t address, uint8_t *rxd) {
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  spiTransmitBuffer[2] = 0;

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spiReceiveBuffer[2] = spi_readwrite(0x00);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif
  // Update data
  *rxd = spiReceiveBuffer[2];

  return spiTransferError;
}

int8_t mcp2518fd_WriteByte(uint16_t address, uint8_t txd) {
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  spiTransmitBuffer[2] = txd;

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

int8_t mcp2518fd_ReadWord(uint16_t address, uint32_t *rxd) {
  uint8_t i;
  uint32_t x;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 2; i < 6; i++) {
    spiReceiveBuffer[i] = spi_readwrite(0x00);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  // Update data
  *rxd = 0;
  for (i = 2; i < 6; i++) {
    x = (uint32_t)spiReceiveBuffer[i];
    *rxd += x << ((i - 2) * 8);
  }

  return spiTransferError;
}

int8_t mcp2518fd_WriteWord(uint16_t address, uint32_t txd) {
  uint8_t i;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

  // Split word into 4 bytes and add them to buffer
  for (i = 0; i < 4; i++) {
    spiTransmitBuffer[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  spi_readwrite(spiTransmitBuffer[3]);
  spi_readwrite(spiTransmitBuffer[4]);
  spi_readwrite(spiTransmitBuffer[5]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

int8_t mcp2518fd_ReadHalfWord(uint16_t address, uint16_t *rxd) {
  uint8_t i;
  uint32_t x;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 2; i < 4; i++) {
    spiReceiveBuffer[i] = spi_readwrite(0x00);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  // Update data
  *rxd = 0;
  for (i = 2; i < 4; i++) {
    x = (uint32_t)spiReceiveBuffer[i];
    *rxd += x << ((i - 2) * 8);
  }

  return spiTransferError;
}

int8_t mcp2518fd_WriteHalfWord(uint16_t address, uint16_t txd) {
  uint8_t i;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

  // Split word into 2 bytes and add them to buffer
  for (i = 0; i < 2; i++) {
    spiTransmitBuffer[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  spi_readwrite(spiTransmitBuffer[3]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

int8_t mcp2518fd_ReadByteArray(uint16_t address, uint8_t *rxd,
                                          uint16_t nBytes) {
  uint16_t i;
  uint16_t spiTransferSize = nBytes + 2;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

  // Clear data
  for (i = 2; i < spiTransferSize; i++) {
    spiTransmitBuffer[i] = 0;
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 0; i < nBytes; i++) {
    spiReceiveBuffer[i + 2] = spi_readwrite(0x00);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  // Update data
  for (i = 0; i < nBytes; i++) {
    rxd[i] = spiReceiveBuffer[i + 2];
  }

  return spiTransferError;
}

int8_t mcp2518fd_WriteByteArray(uint16_t address, uint8_t *txd,
                                           uint16_t nBytes) {
  uint16_t i;
  uint16_t spiTransferSize = nBytes + 2;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  // Add data
  for (i = 2; i < spiTransferSize; i++) {
    spiTransmitBuffer[i] = txd[i - 2];
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 2; i < spiTransferSize; i++) {
    spi_readwrite(spiTransmitBuffer[i]);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

/*
int8_t mcp2518fd_WriteByteSafe(uint16_t address, uint8_t txd) {
  uint16_t crcResult = 0;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  spiTransmitBuffer[2] = txd;

  // Add CRC
  crcResult = DRV_CANFDSPI_CalculateCRC16(spiTransmitBuffer, 3);
  spiTransmitBuffer[3] = (crcResult >> 8) & 0xFF;
  spiTransmitBuffer[4] = crcResult & 0xFF;

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  spi_readwrite(spiTransmitBuffer[3]);
  spi_readwrite(spiTransmitBuffer[4]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

int8_t mcp2518fd_WriteWordSafe(uint16_t address, uint32_t txd) {
  uint8_t i;
  uint16_t crcResult = 0;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);

  // Split word into 4 bytes and add them to buffer
  for (i = 0; i < 4; i++) {
    spiTransmitBuffer[i + 2] = (uint8_t)((txd >> (i * 8)) & 0xFF);
  }

  // Add CRC
  crcResult = DRV_CANFDSPI_CalculateCRC16(spiTransmitBuffer, 6);
  spiTransmitBuffer[6] = (crcResult >> 8) & 0xFF;
  spiTransmitBuffer[7] = crcResult & 0xFF;

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  spi_readwrite(spiTransmitBuffer[3]);
  spi_readwrite(spiTransmitBuffer[4]);
  spi_readwrite(spiTransmitBuffer[5]);
  spi_readwrite(spiTransmitBuffer[6]);
  spi_readwrite(spiTransmitBuffer[7]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}

int8_t mcp2518fd_ReadByteArrayWithCRC(uint16_t address, uint8_t *rxd,
                                                 uint16_t nBytes, bool fromRam,
                                                 bool *crcIsCorrect) {
  uint8_t i;
  uint16_t crcFromSpiSlave = 0;
  uint16_t crcAtController = 0;
  uint16_t spiTransferSize =
      nBytes + 5; // first two bytes for sending command & address, third for
                  // size, last two bytes for CRC
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_READ_CRC << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  spiTransmitBuffer[2] = fromRam? (nBytes >> 2): nBytes;

  // Clear data
  for (i = 3; i < spiTransferSize; i++) {
    spiTransmitBuffer[i] = 0;
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  for (i = 3; i < spiTransferSize; i++) {
    spiReceiveBuffer[i] = spi_readwrite(0x00);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  // Get CRC from controller
  crcFromSpiSlave = (uint16_t)(spiReceiveBuffer[spiTransferSize - 2] << 8) +
                    (uint16_t)(spiReceiveBuffer[spiTransferSize - 1]);

  // Use the receive buffer to calculate CRC
  // First three bytes need to be command
  spiReceiveBuffer[0] = spiTransmitBuffer[0];
  spiReceiveBuffer[1] = spiTransmitBuffer[1];
  spiReceiveBuffer[2] = spiTransmitBuffer[2];
  crcAtController = DRV_CANFDSPI_CalculateCRC16(spiReceiveBuffer, nBytes + 3);

  // Compare CRC readings
  if (crcFromSpiSlave == crcAtController) {
    *crcIsCorrect = true;
  } else {
    *crcIsCorrect = false;
  }

  // Update data
  for (i = 0; i < nBytes; i++) {
    rxd[i] = spiReceiveBuffer[i + 3];
  }

  return spiTransferError;
}

int8_t mcp2518fd_WriteByteArrayWithCRC(uint16_t address,
                                                  uint8_t *txd, uint16_t nBytes,
                                                  bool fromRam) {
  uint16_t i;
  uint16_t crcResult = 0;
  uint16_t spiTransferSize = nBytes + 5;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] =
      (uint8_t)((cINSTRUCTION_WRITE_CRC << 4) + ((address >> 8) & 0xF));
  spiTransmitBuffer[1] = (uint8_t)(address & 0xFF);
  spiTransmitBuffer[2] = fromRam? (nBytes >> 2): nBytes;

  // Add data
  for (i = 0; i < nBytes; i++) {
    spiTransmitBuffer[i + 3] = txd[i];
  }

  // Add CRC
  crcResult = DRV_CANFDSPI_CalculateCRC16(spiTransmitBuffer, spiTransferSize - 2);
  spiTransmitBuffer[spiTransferSize - 2] = (uint8_t)((crcResult >> 8) & 0xFF);
  spiTransmitBuffer[spiTransferSize - 1] = (uint8_t)(crcResult & 0xFF);

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  spi_readwrite(spiTransmitBuffer[2]);
  for (i = 0; i < nBytes; i++) {
    spi_readwrite(spiTransmitBuffer[i + 3]);
  }
  spi_readwrite(spiTransmitBuffer[spiTransferSize - 2]);
  spi_readwrite(spiTransmitBuffer[spiTransferSize - 1]);
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}
*/

int8_t mcp2518fd_ReadWordArray(uint16_t address, uint32_t *rxd,
                                          uint16_t nWords) {
  uint16_t i, j, n;
  REG_t w;
  uint16_t spiTransferSize = nWords * 4 + 2;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] = (cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF);
  spiTransmitBuffer[1] = address & 0xFF;

  // Clear data
  for (i = 2; i < spiTransferSize; i++) {
    spiTransmitBuffer[i] = 0;
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 2; i < spiTransferSize; i++) {
    // for (i = 2; i < 6; i++) {
    spiReceiveBuffer[i] = spi_readwrite(0x00);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif
  // Convert Byte array to Word array
  n = 2;
  for (i = 0; i < nWords; i++) {
    w.word = 0;
    for (j = 0; j < 4; j++, n++) {
      w.byte[j] = spiReceiveBuffer[n];
    }
    rxd[i] = w.word;
  }
  return spiTransferError;
}

/*
int8_t mcp2518fd_WriteWordArray(uint16_t address, uint32_t *txd,
                                           uint16_t nWords) {
  uint16_t i, j, n;
  REG_t w;
  uint16_t spiTransferSize = nWords * 4 + 2;
  int8_t spiTransferError = 0;

  // Compose command
  spiTransmitBuffer[0] = (cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF);
  spiTransmitBuffer[1] = address & 0xFF;

  // Convert ByteArray to word array
  n = 2;
  for (i = 0; i < nWords; i++) {
    w.word = txd[i];
    for (j = 0; j < 4; j++, n++) {
      spiTransmitBuffer[n] = w.byte[j];
    }
  }

#ifdef SPI_HAS_TRANSACTION
  SPI_BEGIN();
#endif
  MCP2518fd_SELECT();
  spi_readwrite(spiTransmitBuffer[0]);
  spi_readwrite(spiTransmitBuffer[1]);
  for (i = 2; i < spiTransferSize; i++) {
    spi_readwrite(spiTransmitBuffer[i]);
  }
  MCP2518fd_UNSELECT();
#ifdef SPI_HAS_TRANSACTION
  SPI_END();
#endif

  return spiTransferError;
}
*/

int8_t mcp2518fd_EccEnable() {
  int8_t spiTransferError = 0;
  uint8_t d = 0;

  // Read
  spiTransferError = mcp2518fd_ReadByte(cREGADDR_ECCCON, &d);
  if (spiTransferError) {
    return -1;
  }
  // Modify
  d |= 0x01;

  // Write
  spiTransferError = mcp2518fd_WriteByte(cREGADDR_ECCCON, d);
  if (spiTransferError) {
    return -2;
  }

  return 0;
}

int8_t mcp2518fd_RamInit(uint8_t d) {
  uint8_t txd[SPI_DEFAULT_BUFFER_LENGTH];
  uint32_t k;
  int8_t spiTransferError = 0;

  // Prepare data
  for (k = 0; k < SPI_DEFAULT_BUFFER_LENGTH; k++) {
    txd[k] = d;
  }

  uint16_t a = cRAMADDR_START;

  for (k = 0; k < (cRAM_SIZE / SPI_DEFAULT_BUFFER_LENGTH); k++) {
    spiTransferError = mcp2518fd_WriteByteArray(a, txd, SPI_DEFAULT_BUFFER_LENGTH);
    if (spiTransferError) {
      return -1;
    }
    a += SPI_DEFAULT_BUFFER_LENGTH;
  }

  return spiTransferError;
}

int8_t mcp2518fd_ConfigureObjectReset(CAN_CONFIG *config) {
  REG_CiCON ciCon;
  ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

  config->DNetFilterCount = ciCon.bF.DNetFilterCount;
  config->IsoCrcEnable = ciCon.bF.IsoCrcEnable;
  config->ProtocolExpectionEventDisable = ciCon.bF.ProtocolExceptionEventDisable;
  config->WakeUpFilterEnable = ciCon.bF.WakeUpFilterEnable;
  config->WakeUpFilterTime = ciCon.bF.WakeUpFilterTime;
  config->BitRateSwitchDisable = ciCon.bF.BitRateSwitchDisable;
  config->RestrictReTxAttempts = ciCon.bF.RestrictReTxAttempts;
  config->EsiInGatewayMode = ciCon.bF.EsiInGatewayMode;
  config->SystemErrorToListenOnly = ciCon.bF.SystemErrorToListenOnly;
  config->StoreInTEF = ciCon.bF.StoreInTEF;
  config->TXQEnable = ciCon.bF.TXQEnable;
  config->TxBandWidthSharing = ciCon.bF.TxBandWidthSharing;

  return 0;
}

int8_t mcp2518fd_Configure(CAN_CONFIG *config) {
  REG_CiCON ciCon;
  int8_t spiTransferError = 0;

  ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

  ciCon.bF.DNetFilterCount = config->DNetFilterCount;
  ciCon.bF.IsoCrcEnable = config->IsoCrcEnable;
  ciCon.bF.ProtocolExceptionEventDisable = config->ProtocolExpectionEventDisable;
  ciCon.bF.WakeUpFilterEnable = config->WakeUpFilterEnable;
  ciCon.bF.WakeUpFilterTime = config->WakeUpFilterTime;
  ciCon.bF.BitRateSwitchDisable = config->BitRateSwitchDisable;
  ciCon.bF.RestrictReTxAttempts = config->RestrictReTxAttempts;
  ciCon.bF.EsiInGatewayMode = config->EsiInGatewayMode;
  ciCon.bF.SystemErrorToListenOnly = config->SystemErrorToListenOnly;
  ciCon.bF.StoreInTEF = config->StoreInTEF;
  ciCon.bF.TXQEnable = config->TXQEnable;
  ciCon.bF.TxBandWidthSharing = config->TxBandWidthSharing;

  spiTransferError = mcp2518fd_WriteWord(cREGADDR_CiCON, ciCon.word);
  if (spiTransferError) {
    return -1;
  }

  return spiTransferError;
}

int8_t mcp2518fd_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG *config) {
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = canFifoResetValues[0]; // 10010010100101010000

  config->RTREnable = ciFifoCon.txBF.RTREnable;
  config->TxPriority = ciFifoCon.txBF.TxPriority;
  config->TxAttempts = ciFifoCon.txBF.TxAttempts;
  config->FifoSize = ciFifoCon.txBF.FifoSize;
  config->PayLoadSize = ciFifoCon.txBF.PayLoadSize;

  return 0;
}

int8_t
mcp2518fd_TransmitChannelConfigure(CAN_FIFO_CHANNEL channel,
                                              CAN_TX_FIFO_CONFIG *config) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;
  // Setup FIFO
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = canFifoResetValues[0];
  ciFifoCon.txBF.TxEnable = 1;
  ciFifoCon.txBF.FifoSize = config->FifoSize;
  ciFifoCon.txBF.PayLoadSize = config->PayLoadSize;
  ciFifoCon.txBF.TxAttempts = config->TxAttempts;
  ciFifoCon.txBF.TxPriority = config->TxPriority;
  ciFifoCon.txBF.RTREnable = config->RTREnable;

  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_WriteWord(a, ciFifoCon.word);

  return spiTransferError;
}

int8_t mcp2518fd_ReceiveChannelConfigureObjectReset(CAN_RX_FIFO_CONFIG *config) {
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = canFifoResetValues[0];

  config->FifoSize = ciFifoCon.rxBF.FifoSize;
  config->PayLoadSize = ciFifoCon.rxBF.PayLoadSize;
  config->RxTimeStampEnable = ciFifoCon.rxBF.RxTimeStampEnable;

  return 0;
}

int8_t
mcp2518fd_ReceiveChannelConfigure(CAN_FIFO_CHANNEL channel,
                                             CAN_RX_FIFO_CONFIG *config) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  if (channel == CAN_TXQUEUE_CH0) {
    return -100;
  }

  // Setup FIFO
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = canFifoResetValues[0];

  ciFifoCon.rxBF.TxEnable = 0;
  ciFifoCon.rxBF.FifoSize = config->FifoSize;
  ciFifoCon.rxBF.PayLoadSize = config->PayLoadSize;
  ciFifoCon.rxBF.RxTimeStampEnable = config->RxTimeStampEnable;

  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_WriteWord(a, ciFifoCon.word);

  return spiTransferError;
}

int8_t mcp2518fd_FilterObjectConfigure(CAN_FILTER filter,
                                                  CAN_FILTEROBJ_ID *id) {
  uint16_t a;
  REG_CiFLTOBJ fObj;
  int8_t spiTransferError = 0;

  // Setup
  fObj.word = 0;
  fObj.bF = *id;
  a = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);

  spiTransferError = mcp2518fd_WriteWord(a, fObj.word);

  return spiTransferError;
}

int8_t mcp2518fd_FilterMaskConfigure(CAN_FILTER filter,
                                                CAN_MASKOBJ_ID *mask) {
  uint16_t a;
  REG_CiMASK mObj;
  int8_t spiTransferError = 0;

  // Setup
  mObj.word = 0;
  mObj.bF = *mask;
  a = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);

  spiTransferError = mcp2518fd_WriteWord(a, mObj.word);

  return spiTransferError;
}

int8_t mcp2518fd_FilterToFifoLink(CAN_FILTER filter,
                                             CAN_FIFO_CHANNEL channel,
                                             bool enable) {
  uint16_t a;
  REG_CiFLTCON_BYTE fCtrl;
  int8_t spiTransferError = 0;

  // Enable
  fCtrl.bF.Enable = !!enable;

  // Link
  fCtrl.bF.BufferPointer = channel;
  a = cREGADDR_CiFLTCON + filter;

  spiTransferError = mcp2518fd_WriteByte(a, fCtrl.byte);

  return spiTransferError;
}


/*
 * bittime calculation code from
 *   https://github.com/pierremolinaro/acan2517FD
 *
 */
static const uint16_t MAX_BRP = 256 ;
static const uint16_t MAX_ARBITRATION_PHASE_SEGMENT_1 = 256 ;
static const uint8_t  MAX_ARBITRATION_PHASE_SEGMENT_2 = 128 ;
static const uint16_t MAX_DATA_PHASE_SEGMENT_1 = 32 ;
static const uint8_t  MAX_DATA_PHASE_SEGMENT_2 = 16 ;

int calcBittime(const uint32_t inDesiredArbitrationBitRate,
                           const uint32_t inTolerancePPM)
{
  if (mDataBitRateFactor <= 1) { // Single bit rate
    const uint32_t maxTQCount = MAX_ARBITRATION_PHASE_SEGMENT_1 + MAX_ARBITRATION_PHASE_SEGMENT_2 + 1 ; // Setting for slowest bit rate
    uint32_t BRP = MAX_BRP ;
    uint32_t smallestError = UINT32_MAX ;
    uint32_t bestBRP = 1 ; // Setting for highest bit rate
    uint32_t bestTQCount = 4 ; // Setting for highest bit rate
    uint32_t TQCount = mSysClock / inDesiredArbitrationBitRate / BRP ;
  //--- Loop for finding best BRP and best TQCount
    while ((TQCount <= (MAX_ARBITRATION_PHASE_SEGMENT_1 + MAX_ARBITRATION_PHASE_SEGMENT_2 + 1)) && (BRP > 0)) {
    //--- Compute error using TQCount
      if ((TQCount >= 4) && (TQCount <= maxTQCount)) {
        const uint32_t error = mSysClock - inDesiredArbitrationBitRate * TQCount * BRP ; // error is always >= 0
        if (error <= smallestError) {
          smallestError = error ;
          bestBRP = BRP ;
          bestTQCount = TQCount ;
        }
      }
    //--- Compute error using TQCount+1
      if ((TQCount >= 3) && (TQCount < maxTQCount)) {
        const uint32_t error = inDesiredArbitrationBitRate * (TQCount + 1) * BRP - mSysClock ; // error is always >= 0
        if (error <= smallestError) {
          smallestError = error ;
          bestBRP = BRP ;
          bestTQCount = TQCount + 1 ;
        }
      }
    //--- Continue with next value of BRP
      BRP -- ;
      TQCount = (BRP == 0) ? (maxTQCount + 1) : (mSysClock / inDesiredArbitrationBitRate / BRP) ;
    }
  //--- Compute PS2 (1 <= PS2 <= 128)
    uint32_t PS2 = bestTQCount / 5 ; // For sampling point at 80%
    if (PS2 == 0) {
      PS2 = 1 ;
    }else if (PS2 > MAX_ARBITRATION_PHASE_SEGMENT_2) {
      PS2 = MAX_ARBITRATION_PHASE_SEGMENT_2 ;
    }
  //--- Compute PS1 (1 <= PS1 <= 256)
    uint32_t PS1 = bestTQCount - PS2 - 1 /* Sync Seg */ ;
    if (PS1 > MAX_ARBITRATION_PHASE_SEGMENT_1) {
      PS2 += PS1 - MAX_ARBITRATION_PHASE_SEGMENT_1 ;
      PS1 = MAX_ARBITRATION_PHASE_SEGMENT_1 ;
    }
  //---
    mBitRatePrescaler = (uint16_t) bestBRP ;
    mArbitrationPhaseSegment1 = (uint16_t) PS1 ;
    mArbitrationPhaseSegment2 = (uint8_t) PS2 ;
    mArbitrationSJW = mArbitrationPhaseSegment2 ; // Always 1 <= SJW <= 128, and SJW <= mArbitrationPhaseSegment2
  //--- Final check of the nominal configuration
    const uint32_t W = bestTQCount * inDesiredArbitrationBitRate * bestBRP ;
    const uint64_t diff = (mSysClock > W) ? (mSysClock - W) : (W - mSysClock) ;
    const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
    mArbitrationBitRateClosedToDesiredRate = (diff * ppm) <= (((uint64_t) W) * inTolerancePPM) ;
  }else{ // Dual bit rate, first compute data bit rate
    const uint32_t maxDataTQCount = MAX_DATA_PHASE_SEGMENT_1 + MAX_DATA_PHASE_SEGMENT_2 ; // Setting for slowest bit rate
    const uint32_t desiredDataBitRate = inDesiredArbitrationBitRate * (uint8_t)mDataBitRateFactor ;
    uint32_t smallestError = UINT32_MAX ;
    uint32_t bestBRP = MAX_BRP ; // Setting for lowest bit rate
    uint32_t bestDataTQCount = maxDataTQCount ; // Setting for lowest bit rate
    uint32_t dataTQCount = 4 ;
    uint32_t brp = mSysClock / desiredDataBitRate / dataTQCount ;
  //--- Loop for finding best BRP and best TQCount
    while ((dataTQCount <= maxDataTQCount) && (brp > 0)) {
    //--- Compute error using brp
      if (brp <= MAX_BRP) {
        const uint32_t error = mSysClock - desiredDataBitRate * dataTQCount * brp ; // error is always >= 0
        if (error <= smallestError) {
          smallestError = error ;
          bestBRP = brp ;
          bestDataTQCount = dataTQCount ;
        }
      }
    //--- Compute error using brp+1
      if (brp < MAX_BRP) {
        const uint32_t error = desiredDataBitRate * dataTQCount * (brp + 1) - mSysClock ; // error is always >= 0
        if (error <= smallestError) {
          smallestError = error ;
          bestBRP = brp + 1 ;
          bestDataTQCount = dataTQCount ;
        }
      }
    //--- Continue with next value of BRP
      dataTQCount += 1 ;
      brp = mSysClock / desiredDataBitRate / dataTQCount ;
    }
  //--- Compute data PS2 (1 <= PS2 <= 16)
    uint32_t dataPS2 = bestDataTQCount / 5 ; // For sampling point at 80%
    if (dataPS2 == 0) {
      dataPS2 = 1 ;
    }
  //--- Compute data PS1 (1 <= PS1 <= 32)
    uint32_t dataPS1 = bestDataTQCount - dataPS2 - 1 /* Sync Seg */ ;
    if (dataPS1 > MAX_DATA_PHASE_SEGMENT_1) {
      dataPS2 += dataPS1 - MAX_DATA_PHASE_SEGMENT_1 ;
      dataPS1 = MAX_DATA_PHASE_SEGMENT_1 ;
    }
  //---
    const int TDCO = bestBRP * dataPS1 ; // According to DS20005678D, §3.4.8 Page 20
    mTDCO = (TDCO > 63) ? 63 : (int8_t) TDCO ;
    mDataPhaseSegment1 = (uint8_t) dataPS1 ;
    mDataPhaseSegment2 = (uint8_t) dataPS2 ;
    mDataSJW = mDataPhaseSegment2 ;
    const uint32_t arbitrationTQCount = bestDataTQCount * (uint8_t)mDataBitRateFactor ;
  //--- Compute arbiration PS2 (1 <= PS2 <= 128)
    uint32_t arbitrationPS2 = arbitrationTQCount / 5 ; // For sampling point at 80%
    if (arbitrationPS2 == 0) {
      arbitrationPS2 = 1 ;
    }
  //--- Compute PS1 (1 <= PS1 <= 256)
    uint32_t arbitrationPS1 = arbitrationTQCount - arbitrationPS2 - 1 /* Sync Seg */ ;
    if (arbitrationPS1 > MAX_ARBITRATION_PHASE_SEGMENT_1) {
      arbitrationPS2 += arbitrationPS1 - MAX_ARBITRATION_PHASE_SEGMENT_1 ;
      arbitrationPS1 = MAX_ARBITRATION_PHASE_SEGMENT_1 ;
    }
  //---
    mBitRatePrescaler = (uint16_t) bestBRP ;
    mArbitrationPhaseSegment1 = (uint16_t) arbitrationPS1 ;
    mArbitrationPhaseSegment2 = (uint8_t) arbitrationPS2 ;
    mArbitrationSJW = mArbitrationPhaseSegment2 ; // Always 1 <= SJW <= 128, and SJW <= mArbitrationPhaseSegment2
  //--- Final check of the nominal configuration
    const uint32_t W = arbitrationTQCount * inDesiredArbitrationBitRate * bestBRP ;
    const uint64_t diff = (mSysClock > W) ? (mSysClock - W) : (W - mSysClock) ;
    const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
    mArbitrationBitRateClosedToDesiredRate = (diff * ppm) <= (((uint64_t) W) * inTolerancePPM) ;
  }
  return mArbitrationBitRateClosedToDesiredRate;
}

int8_t mcp2518fd_BitTimeConfigureNominal() {
  int8_t spiTransferError = 0;
  REG_CiNBTCFG ciNbtcfg;

  ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

  // Arbitration Bit rate
  ciNbtcfg.bF.BRP = mBitRatePrescaler - 1;
  ciNbtcfg.bF.TSEG1 = mArbitrationPhaseSegment1 - 1;
  ciNbtcfg.bF.TSEG2 = mArbitrationPhaseSegment2 - 1;
  ciNbtcfg.bF.SJW = mArbitrationSJW - 1;

  // Write Bit time registers
  spiTransferError = mcp2518fd_WriteWord(cREGADDR_CiNBTCFG, ciNbtcfg.word);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

int8_t
mcp2518fd_BitTimeConfigureData(CAN_SSP_MODE sspMode) {
  int8_t spiTransferError = 0;
  REG_CiDBTCFG ciDbtcfg;
  REG_CiTDC ciTdc;

  // Write Bit time registers
  ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
  ciDbtcfg.bF.BRP = mBitRatePrescaler - 1;
  ciDbtcfg.bF.TSEG1 = mDataPhaseSegment1 - 1;
  ciDbtcfg.bF.TSEG2 = mDataPhaseSegment2 - 1;
  ciDbtcfg.bF.SJW = mDataSJW - 1;

  spiTransferError = mcp2518fd_WriteWord(cREGADDR_CiDBTCFG, ciDbtcfg.word);
  if (spiTransferError) {
    return -2;
  }

  // Configure Bit time and sample point, SSP
  ciTdc.word = canControlResetValues[cREGADDR_CiTDC / 4];
  ciTdc.bF.TDCMode = sspMode;
  ciTdc.bF.TDCOffset = mTDCO;
  // ciTdc.bF.TDCValue = ?;

  spiTransferError = mcp2518fd_WriteWord(cREGADDR_CiTDC, ciTdc.word);
  if (spiTransferError) {
    return -3;
  }

  return spiTransferError;
}

int8_t mcp2518fd_BitTimeConfigure(uint32_t speedset,
                                             CAN_SSP_MODE sspMode,
                                             CAN_SYSCLK_SPEED clk) {
  int8_t spiTransferError = 0;

  // Decode bitrate
  mDesiredArbitrationBitRate = speedset & 0xFFFFFUL;
  mDataBitRateFactor         = (speedset >> 24) & 0xFF;

  // Decode clk
  switch (clk) {
  case CAN_SYSCLK_10M:
    mSysClock = 10UL * 1000UL * 1000UL; break;
  case CAN_SYSCLK_20M:
    mSysClock = 20UL * 1000UL * 1000UL; break;
  case CAN_SYSCLK_40M:
  default:
    mSysClock = 40UL * 1000UL * 1000UL; break;
  }

  calcBittime(mDesiredArbitrationBitRate, 10000 /* 1% */);
  mcp2518fd_BitTimeConfigureNominal();
  mcp2518fd_BitTimeConfigureData(sspMode);

  return spiTransferError;
}

int8_t mcp2518fd_GpioModeConfigure(GPIO_PIN_MODE gpio0,
                                              GPIO_PIN_MODE gpio1) {
  int8_t spiTransferError = 0;

  // Read
  uint16_t a = cREGADDR_IOCON + 3;
  REG_IOCON iocon;
  iocon.word = 0;

  spiTransferError = mcp2518fd_ReadByte(a, &iocon.byte[3]);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  iocon.bF.PinMode0 = gpio0;
  iocon.bF.PinMode1 = gpio1;

  // Write
  spiTransferError = mcp2518fd_WriteByte(a, iocon.byte[3]);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

/*
int8_t
mcp2518fd_TransmitChannelEventEnable(CAN_FIFO_CHANNEL channel,
                                                CAN_TX_FIFO_EVENT flags) {
  int8_t spiTransferError = 0;

  // Read Interrupt Enables
  uint16_t a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = 0;

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoCon.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  ciFifoCon.byte[0] |= (flags & CAN_TX_FIFO_ALL_EVENTS);

  // Write
  spiTransferError = mcp2518fd_WriteByte(a, ciFifoCon.byte[0]);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}
*/

int8_t mcp2518fd_ReceiveChannelEventEnable(CAN_FIFO_CHANNEL channel,
                                                      CAN_RX_FIFO_EVENT flags) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  if (channel == CAN_TXQUEUE_CH0)
    return -100;

  // Read Interrupt Enables
  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
  REG_CiFIFOCON ciFifoCon;
  ciFifoCon.word = 0;

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoCon.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  ciFifoCon.byte[0] |= (flags & CAN_RX_FIFO_ALL_EVENTS);

  // Write
  spiTransferError = mcp2518fd_WriteByte(a, ciFifoCon.byte[0]);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

int8_t mcp2518fd_ModuleEventEnable(CAN_MODULE_EVENT flags) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  // Read Interrupt Enables
  a = cREGADDR_CiINTENABLE;
  REG_CiINTENABLE intEnables;
  intEnables.word = 0;

  spiTransferError = mcp2518fd_ReadHalfWord(a, &intEnables.word);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  intEnables.word |= (flags & CAN_ALL_EVENTS);

  // Write
  spiTransferError = mcp2518fd_WriteHalfWord(a, intEnables.word);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

int8_t mcp2518fd_OperationModeSelect(CAN_OPERATION_MODE opMode) {
  uint8_t d = 0;
  int8_t spiTransferError = 0;

  // Read
  spiTransferError = mcp2518fd_ReadByte(cREGADDR_CiCON + 3, &d);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  d &= ~0x07;
  d |= opMode;

  // Write
  spiTransferError = mcp2518fd_WriteByte(cREGADDR_CiCON + 3, d);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

CAN_OPERATION_MODE mcp2518fd_OperationModeGet() {
  uint8_t d = 0;
  CAN_OPERATION_MODE mode = CAN_INVALID_MODE;
  int8_t spiTransferError = 0;

  // Read Opmode
  spiTransferError = mcp2518fd_ReadByte(cREGADDR_CiCON + 2, &d);
  if (spiTransferError) {
    return CAN_INVALID_MODE;
  }

  // Get Opmode bits
  d = (d >> 5) & 0x7;

  // Decode Opmode
  switch (d) {
  case CAN_NORMAL_MODE:
    mode = CAN_NORMAL_MODE;
    break;
  case CAN_SLEEP_MODE:
    mode = CAN_SLEEP_MODE;
    break;
  case CAN_INTERNAL_LOOPBACK_MODE:
    mode = CAN_INTERNAL_LOOPBACK_MODE;
    break;
  case CAN_EXTERNAL_LOOPBACK_MODE:
    mode = CAN_EXTERNAL_LOOPBACK_MODE;
    break;
  case CAN_LISTEN_ONLY_MODE:
    mode = CAN_LISTEN_ONLY_MODE;
    break;
  case CAN_CONFIGURATION_MODE:
    mode = CAN_CONFIGURATION_MODE;
    break;
  case CAN_CLASSIC_MODE:
    mode = CAN_CLASSIC_MODE;
    break;
  case CAN_RESTRICTED_MODE:
    mode = CAN_RESTRICTED_MODE;
    break;
  default:
    mode = CAN_INVALID_MODE;
    break;
  }

  return mode;
}

int8_t mcp2518fd_TransmitChannelEventGet(CAN_FIFO_CHANNEL channel,
                                                    CAN_TX_FIFO_EVENT *flags) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  // Read Interrupt flags
  REG_CiFIFOSTA ciFifoSta;
  ciFifoSta.word = 0;
  a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoSta.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Update data
  *flags = (CAN_TX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_TX_FIFO_ALL_EVENTS);

  return spiTransferError;
}

int8_t mcp2518fd_ErrorCountStateGet(uint8_t *tec, uint8_t *rec,
                                               CAN_ERROR_STATE *flags) {
  int8_t spiTransferError = 0;

  // Read Error
  uint16_t a = cREGADDR_CiTREC;
  REG_CiTREC ciTrec;
  ciTrec.word = 0;

  spiTransferError = mcp2518fd_ReadWord(a, &ciTrec.word);
  if (spiTransferError) {
    return -1;
  }

  // Update data
  *tec = ciTrec.byte[1];
  *rec = ciTrec.byte[0];
  *flags = (CAN_ERROR_STATE)(ciTrec.byte[2] & CAN_ERROR_ALL);

  return spiTransferError;
}

// *****************************************************************************
// *****************************************************************************
// Section: Miscellaneous
uint32_t DRV_CANFDSPI_DlcToDataBytes(CAN_DLC dlc) {
  uint32_t dataBytesInObject = 0;

  if (dlc < CAN_DLC_12) {
    dataBytesInObject = dlc;
  } else {
    switch (dlc) {
    case CAN_DLC_12:
      dataBytesInObject = 12;
      break;
    case CAN_DLC_16:
      dataBytesInObject = 16;
      break;
    case CAN_DLC_20:
      dataBytesInObject = 20;
      break;
    case CAN_DLC_24:
      dataBytesInObject = 24;
      break;
    case CAN_DLC_32:
      dataBytesInObject = 32;
      break;
    case CAN_DLC_48:
      dataBytesInObject = 48;
      break;
    case CAN_DLC_64:
      dataBytesInObject = 64;
      break;
    default:
      break;
    }
  }

  return dataBytesInObject;
}

int8_t mcp2518fd_TransmitChannelLoad(CAN_FIFO_CHANNEL channel,
                                                CAN_TX_MSGOBJ *txObj,
                                                uint8_t *txd,
                                                uint32_t txdNumBytes,
                                                bool flush) {
  uint16_t a;
  uint32_t fifoReg[3];
  uint32_t dataBytesInObject;
  REG_CiFIFOCON ciFifoCon;
  REG_CiFIFOUA ciFifoUa;
  int8_t spiTransferError = 0;

  // Get FIFO registers
  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_ReadWordArray(a, fifoReg, 3);
  if (spiTransferError) {
    return -1;
  }

  // Check that it is a transmit buffer
  ciFifoCon.word = fifoReg[0];
  if (!ciFifoCon.txBF.TxEnable) {
    return -2;
  }

  // Check that DLC is big enough for data
  dataBytesInObject = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj->bF.ctrl.DLC);
  if (dataBytesInObject < txdNumBytes) {
    return -3;
  }

  // Get address
  ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
  a = 4 * ciFifoUa.bF.UserAddress;
#else
  a = ciFifoUa.bF.UserAddress;
#endif
  a += cRAMADDR_START;
  uint8_t txBuffer[MAX_MSG_SIZE];

  txBuffer[0] = txObj->byte[0]; // not using 'for' to reduce no of instructions
  txBuffer[1] = txObj->byte[1];
  txBuffer[2] = txObj->byte[2];
  txBuffer[3] = txObj->byte[3];

  txBuffer[4] = txObj->byte[4];
  txBuffer[5] = txObj->byte[5];
  txBuffer[6] = txObj->byte[6];
  txBuffer[7] = txObj->byte[7];

  uint8_t i;
  for (i = 0; i < txdNumBytes; i++) {
    txBuffer[i + 8] = txd[i];
  }

  // Make sure we write a multiple of 4 bytes to RAM
  uint16_t n = 0;
  uint8_t j = 0;

  if (txdNumBytes % 4) {
    // Need to add bytes
    n = 4 - (txdNumBytes % 4);
    i = txdNumBytes + 8;

    for (j = 0; j < n; j++) {
      txBuffer[i + 8 + j] = 0;
    }
  }
  spiTransferError = mcp2518fd_WriteByteArray(a, txBuffer, txdNumBytes + 8 + n);
  if (spiTransferError) {
    return -4;
  }

  // Set UINC and TXREQ
  spiTransferError = mcp2518fd_TransmitChannelUpdate(channel, flush);
  if (spiTransferError) {
    return -5;
  }
  return spiTransferError;
}

int8_t mcp2518fd_ReceiveChannelEventGet(CAN_FIFO_CHANNEL channel,
                                                   CAN_RX_FIFO_EVENT *flags) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  if (channel == CAN_TXQUEUE_CH0)
    return -100;

  // Read Interrupt flags
  REG_CiFIFOSTA ciFifoSta;
  ciFifoSta.word = 0;
  a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoSta.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Update data
  *flags = (CAN_RX_FIFO_EVENT)(ciFifoSta.byte[0] & CAN_RX_FIFO_ALL_EVENTS);
  return spiTransferError;
}

int8_t mcp2518fd_ReceiveMessageGet(CAN_FIFO_CHANNEL channel,
                                              CAN_RX_MSGOBJ *rxObj,
                                              uint8_t *rxd, uint8_t nBytes) {
  uint8_t n = 0;
  uint8_t i = 0;
  uint16_t a;
  uint32_t fifoReg[3];
  REG_CiFIFOCON ciFifoCon;
  REG_CiFIFOUA ciFifoUa;
  int8_t spiTransferError = 0;

  // Get FIFO registers
  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_ReadWordArray(a, fifoReg, 3);
  if (spiTransferError) {
    return -1;
  }

  // Check that it is a receive buffer
  ciFifoCon.word = fifoReg[0];
  ciFifoCon.txBF.TxEnable = 0;
  if (ciFifoCon.txBF.TxEnable) {
    return -2;
  }

  // Get address
  ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
  a = 4 * ciFifoUa.bF.UserAddress;
#else
  a = ciFifoUa.bF.UserAddress;
#endif
  a += cRAMADDR_START;

  // Number of bytes to read
  n = nBytes + 8; // Add 8 header bytes

  if (ciFifoCon.rxBF.RxTimeStampEnable) {
    n += 4; // Add 4 time stamp bytes
  }

  // Make sure we read a multiple of 4 bytes from RAM
  if (n % 4) {
    n = n + 4 - (n % 4);
  }

  // Read rxObj using one access
  uint8_t ba[MAX_MSG_SIZE];

  if (n > MAX_MSG_SIZE) {
    n = MAX_MSG_SIZE;
  }

  spiTransferError = mcp2518fd_ReadByteArray(a, ba, n);
  if (spiTransferError) {
    return -3;
  }

  // Assign message header
  REG_t myReg;

  myReg.byte[0] = ba[0];
  myReg.byte[1] = ba[1];
  myReg.byte[2] = ba[2];
  myReg.byte[3] = ba[3];
  rxObj->word[0] = myReg.word;

  myReg.byte[0] = ba[4];
  myReg.byte[1] = ba[5];
  myReg.byte[2] = ba[6];
  myReg.byte[3] = ba[7];
  rxObj->word[1] = myReg.word;

  if (ciFifoCon.rxBF.RxTimeStampEnable) {
    myReg.byte[0] = ba[8];
    myReg.byte[1] = ba[9];
    myReg.byte[2] = ba[10];
    myReg.byte[3] = ba[11];
    rxObj->word[2] = myReg.word;

    // Assign message data
    for (i = 0; i < nBytes; i++) {
      rxd[i] = ba[i + 12];
    }
  } else {
    rxObj->word[2] = 0;

    // Assign message data
    for (i = 0; i < nBytes; i++) {
      rxd[i] = ba[i + 8];
    }
  }

  // UINC channel
  spiTransferError = mcp2518fd_ReceiveChannelUpdate(channel);
  if (spiTransferError) {
    return -4;
  }

  return spiTransferError;
}

int8_t mcp2518fd_ReceiveChannelUpdate(CAN_FIFO_CHANNEL channel) {
  uint16_t a = 0;
  REG_CiFIFOCON ciFifoCon;
  int8_t spiTransferError = 0;
  ciFifoCon.word = 0;

  // Set UINC
  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) +
      1; // Byte that contains FRESET
  ciFifoCon.rxBF.UINC = 1;

  // Write byte
  spiTransferError = mcp2518fd_WriteByte(a, ciFifoCon.byte[1]);

  return spiTransferError;
}

int8_t mcp2518fd_TransmitChannelUpdate(CAN_FIFO_CHANNEL channel,
                                                  bool flush) {
  uint16_t a;
  REG_CiFIFOCON ciFifoCon;
  int8_t spiTransferError = 0;

  // Set UINC
  a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) +
      1; // Byte that contains FRESET
  ciFifoCon.word = 0;
  ciFifoCon.txBF.UINC = 1;

  // Set TXREQ
  if (flush) {
    ciFifoCon.txBF.TxRequest = 1;
  }

  spiTransferError = mcp2518fd_WriteByte(a, ciFifoCon.byte[1]);
  if (spiTransferError) {
    return -1;
  }

  return spiTransferError;
}

int8_t
mcp2518fd_ReceiveChannelStatusGet(CAN_FIFO_CHANNEL channel,
                                             CAN_RX_FIFO_STATUS *status) {
  uint16_t a;
  REG_CiFIFOSTA ciFifoSta;
  int8_t spiTransferError = 0;

  // Read
  ciFifoSta.word = 0;
  a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoSta.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Update data
  *status = (CAN_RX_FIFO_STATUS)(ciFifoSta.byte[0] & 0x0F);

  return spiTransferError;
}

int8_t mcp2518fd_ErrorStateGet(CAN_ERROR_STATE *flags) {
  int8_t spiTransferError = 0;

  // Read Error state
  uint8_t f = 0;

  spiTransferError = mcp2518fd_ReadByte(cREGADDR_CiTREC + 2, &f);
  if (spiTransferError) {
    return -1;
  }

  // Update data
  *flags = (CAN_ERROR_STATE)(f & CAN_ERROR_ALL);

  return spiTransferError;
}

/*
int8_t mcp2518fd_ModuleEventRxCodeGet(CAN_RXCODE *rxCode) {
  int8_t spiTransferError = 0;
  uint8_t rxCodeByte = 0;

  spiTransferError = mcp2518fd_ReadByte(cREGADDR_CiVEC + 3, &rxCodeByte);
  if (spiTransferError) {
    return -1;
  }

  // Decode data
  // 0x40 = "no interrupt" (CAN_FIFO_CIVEC_NOINTERRUPT)
  if ((rxCodeByte < CAN_RXCODE_TOTAL_CHANNELS) ||
      (rxCodeByte == CAN_RXCODE_NO_INT)) {
    *rxCode = (CAN_RXCODE)rxCodeByte;
  } else {
    *rxCode = CAN_RXCODE_RESERVED; // shouldn't get here
  }

  return spiTransferError;
}

int8_t mcp2518fd_ModuleEventTxCodeGet(CAN_TXCODE *txCode) {
  int8_t spiTransferError = 0;
  uint16_t a = 0;
  uint8_t txCodeByte = 0;

  // Read
  a = cREGADDR_CiVEC + 2;

  spiTransferError = mcp2518fd_ReadByte(a, &txCodeByte);
  if (spiTransferError) {
    return -1;
  }

  // Decode data
  // 0x40 = "no interrupt" (CAN_FIFO_CIVEC_NOINTERRUPT)
  if ((txCodeByte < CAN_TXCODE_TOTAL_CHANNELS) ||
      (txCodeByte == CAN_TXCODE_NO_INT)) {
    *txCode = (CAN_TXCODE)txCodeByte;
  } else {
    *txCode = CAN_TXCODE_RESERVED; // shouldn't get here
  }

  return spiTransferError;
}
*/

int8_t mcp2518fd_TransmitChannelEventAttemptClear(CAN_FIFO_CHANNEL channel)
{
  int8_t spiTransferError = 0;
  uint16_t a = 0;

  // Read Interrupt Enables
  a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
  REG_CiFIFOSTA ciFifoSta;
  ciFifoSta.word = 0;

  spiTransferError = mcp2518fd_ReadByte(a, &ciFifoSta.byte[0]);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  ciFifoSta.byte[0] &= ~CAN_TX_FIFO_ATTEMPTS_EXHAUSTED_EVENT;

  // Write
  spiTransferError = mcp2518fd_WriteByte(a, ciFifoSta.byte[0]);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}


int8_t mcp2518fd_LowPowerModeEnable() {
  int8_t spiTransferError = 0;
  uint8_t d = 0;

#ifdef MCP2517FD
  // LPM not implemented
  spiTransferError = -100;
#else
  // Read
  spiTransferError = mcp2518fd_ReadByte(cREGADDR_OSC, &d);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  d |= 0x08;

  // Write
  spiTransferError = mcp2518fd_WriteByte(cREGADDR_OSC, d);
  if (spiTransferError) {
    return -2;
  }
#endif

  return spiTransferError;
}

int8_t mcp2518fd_LowPowerModeDisable() {
  int8_t spiTransferError = 0;
  uint8_t d = 0;

#ifdef MCP2517FD
  // LPM not implemented
  spiTransferError = -100;
#else
  // Read
  spiTransferError = mcp2518fd_ReadByte(cREGADDR_OSC, &d);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  d &= ~0x08;

  // Write
  spiTransferError = mcp2518fd_WriteByte(cREGADDR_OSC, d);
  if (spiTransferError) {
    return -2;
  }
#endif

  return spiTransferError;
}

void mcp2518fd_TransmitMessageQueue(void) {
  uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;
  CAN_ERROR_STATE errorFlags;
  uint8_t tec, rec;

  // Check if FIFO is not full
  do {
    mcp2518fd_TransmitChannelEventGet(APP_TX_FIFO, &txFlags);
    if (attempts == 0) {
      Nop(); Nop();
      mcp2518fd_ErrorCountStateGet(&tec, &rec, &errorFlags);
      return;
    }
    attempts--;
  } while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

  // Load message and transmit
  uint8_t n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj.bF.ctrl.DLC);
  mcp2518fd_TransmitChannelLoad(APP_TX_FIFO, &txObj, txd, n, true);
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
byte mcp2518fd_sendMsg(const byte *buf, byte len, unsigned long id,
                                  byte ext, byte rtr, bool wait_sent) {
  uint8_t n;
  int i;
  byte spiTransferError = 0;
  // Configure message data
  txObj.word[0] = 0;
  txObj.word[1] = 0;

  txObj.bF.ctrl.RTR = !!rtr;
  if (rtr && len > CAN_DLC_8) {
    len = CAN_DLC_8;
  }
  txObj.bF.ctrl.DLC = len;

  txObj.bF.ctrl.IDE = !!ext;
  if (ext) {
    txObj.bF.id.SID = (id >> 18) & 0x7FF;
    txObj.bF.id.EID = id & 0x3FFFF;
  } else {
    txObj.bF.id.SID = id;
  }

  txObj.bF.ctrl.BRS = true;

  txObj.bF.ctrl.FDF = (len > 8);
  n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj.bF.ctrl.DLC);
  // Prepare data
  for (i = 0; i < n; i++) {
    txd[i] = buf[i];
  }

  mcp2518fd_TransmitMessageQueue();
  return spiTransferError;
}

/*
int8_t mcp2518fd_receiveMsg() {
  CAN_RX_FIFO_EVENT rxFlags;
  mcp2518fd_ReceiveChannelEventGet(APP_RX_FIFO, &rxFlags);

  if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
    mcp2518fd_ReceiveMessageGet(APP_RX_FIFO, &rxObj, rxd, 8);
  }
  return 0;
}
*/

uint32_t bittime_compat_to_mcp2518fd(uint32_t speedset) {
  uint32_t r;

  if (speedset > 0x100) {
    return speedset;
  }
  switch (speedset) {
  case CAN_5KBPS:   r = CANFD_BITRATE(   5000UL, 0); break;
  case CAN_10KBPS:  r = CANFD_BITRATE(  10000UL, 0); break;
  case CAN_20KBPS:  r = CANFD_BITRATE(  20000UL, 0); break;
  case CAN_25KBPS:  r = CANFD_BITRATE(  25000UL, 0); break;
  case CAN_31K25BPS:r = CANFD_BITRATE(  31250UL, 0); break;
  case CAN_33KBPS:  r = CANFD_BITRATE(  33000UL, 0); break;
  case CAN_40KBPS:  r = CANFD_BITRATE(  40000UL, 0); break;
  case CAN_50KBPS:  r = CANFD_BITRATE(  50000UL, 0); break;
  case CAN_80KBPS:  r = CANFD_BITRATE(  80000UL, 0); break;
  case CAN_83K3BPS: r = CANFD_BITRATE(  83300UL, 0); break;
  case CAN_95KBPS:  r = CANFD_BITRATE(  95000UL, 0); break;
  case CAN_100KBPS: r = CANFD_BITRATE( 100000UL, 0); break;
  case CAN_125KBPS: r = CANFD_BITRATE( 125000UL, 0); break;
  case CAN_200KBPS: r = CANFD_BITRATE( 200000UL, 0); break;
  case CAN_250KBPS: r = CANFD_BITRATE( 250000UL, 0); break;
  default:
  case CAN_500KBPS: r = CANFD_BITRATE( 500000UL, 0); break;
  case CAN_666KBPS: r = CANFD_BITRATE( 666000UL, 0); break;
  case CAN_800KBPS: r = CANFD_BITRATE( 800000UL, 0); break;
  case CAN_1000KBPS:r = CANFD_BITRATE(1000000UL, 0); break;
  }
  return r;
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
**                          speedset msb  8 bits = factor (0 or 1 is no bit rate switch)
**                                   lsb 24 bits = arbitration bitrate
*********************************************************************************************************/
uint8_t mcp2518fd_init(uint32_t speedset, const byte clock) {
  // Reset device
  mcp2518fd_reset();

  // Enable ECC and initialize RAM
  mcp2518fd_EccEnable();

  mcp2518fd_RamInit(0xff);

  // Configure device
  mcp2518fd_ConfigureObjectReset(&config);
  config.IsoCrcEnable = 1;
  config.StoreInTEF = 0;
  mcp2518fd_Configure(&config);

  // Setup TX FIFO
  mcp2518fd_TransmitChannelConfigureObjectReset(&txConfig);
  txConfig.FifoSize = 7;
  txConfig.PayLoadSize = CAN_PLSIZE_64;
  txConfig.TxPriority = 1;
  mcp2518fd_TransmitChannelConfigure(APP_TX_FIFO, &txConfig);

  // Setup RX FIFO
  mcp2518fd_ReceiveChannelConfigureObjectReset(&rxConfig);
  rxConfig.FifoSize = 15;
  rxConfig.PayLoadSize = CAN_PLSIZE_64;
  mcp2518fd_ReceiveChannelConfigure(APP_RX_FIFO, &rxConfig);

  // Setup RX Filter
  fObj.word = 0;
  mcp2518fd_FilterObjectConfigure(CAN_FILTER0, &fObj.bF);

  // Setup RX Mask
  mObj.word = 0; // Only allow standard IDs
  mcp2518fd_FilterMaskConfigure(CAN_FILTER0, &mObj.bF);

  // Link FIFO and Filter
  mcp2518fd_FilterToFifoLink(CAN_FILTER0, APP_RX_FIFO, true);

  // Setup Bit Time
  mcp2518fd_BitTimeConfigure(speedset, CAN_SSP_MODE_AUTO, (CAN_SYSCLK_SPEED)clock);

  // Setup Transmit and Receive Interrupts
  mcp2518fd_GpioModeConfigure(GPIO_MODE_INT, GPIO_MODE_INT);
#ifdef APP_USE_TX_INT
  mcp2518fd_TransmitChannelEventEnable(APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
#endif
  mcp2518fd_ReceiveChannelEventEnable(APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
  mcp2518fd_ModuleEventEnable((CAN_MODULE_EVENT)(CAN_TX_EVENT | CAN_RX_EVENT));

  // Select Normal Mode
  // mcp2518fd_OperationModeSelect(CAN_CLASSIC_MODE);
  MCP_setMode(mcpMode);

  return 0;
}

/*********************************************************************************************************
** Function name:           enableTxInterrupt
** Descriptions:            enable interrupt for all tx buffers
*********************************************************************************************************/
void MCP_enableTxInterrupt(bool enable) {
  if (enable == true) {
    mcp2518fd_ModuleEventEnable(CAN_TX_EVENT);
  }
  return;
}

byte MCP_init_Mask(byte num, byte ext, unsigned long ulData) {
  int8_t err;

  mcp2518fd_OperationModeSelect(CAN_CONFIGURATION_MODE);

  // Setup RX Mask
  mObj.word = 0;
  mObj.bF.MSID = ulData;
  mObj.bF.MIDE = ext;
  err = mcp2518fd_FilterMaskConfigure((CAN_FILTER)num, &mObj.bF);
  mcp2518fd_OperationModeSelect(mcpMode);

  return err;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************************************************************************************************/
byte MCP_init_Filt(byte num, byte ext, unsigned long ulData) {
  int8_t err;
  err = mcp2518fd_OperationModeSelect(CAN_CONFIGURATION_MODE);

  // Setup RX Filter
  fObj.word = 0;
  if (!ext) {            // standard identifier
    fObj.bF.SID = ulData;
  } else {               // extended identifier
    fObj.bF.EID = ulData;
  }
  fObj.bF.EXIDE = !!ext;
  mcp2518fd_FilterObjectConfigure((CAN_FILTER)num, &fObj.bF);
  mcp2518fd_OperationModeSelect(mcpMode);
  return err;
}

/*********************************************************************************************************
** Function name:           setSleepWakeup
** Descriptions:            Enable or disable the wake up interrupt (If disabled
*the MCP2515 will not be woken up by CAN bus activity)
*********************************************************************************************************/
void MCP_setSleepWakeup(const byte enable) {
  if (enable) {
    mcp2518fd_LowPowerModeEnable();
  } else {
    mcp2518fd_LowPowerModeDisable();
  }
}

/*********************************************************************************************************
** Function name:           sleep
** Descriptions:            Put mcp2515 in sleep mode to save power
*********************************************************************************************************/
byte MCP_sleep() {
  if (MCP_getMode() != 0x01) {
    return mcp2518fd_OperationModeSelect(CAN_SLEEP_MODE);
  } else {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           wake
** Descriptions:            wake MCP2515 manually from sleep. It will come back
*in the mode it was before sleeping.
*********************************************************************************************************/
byte MCP_wake() {
  byte currMode = MCP_getMode();
  if (currMode != mcpMode) {
    return mcp2518fd_OperationModeSelect(mcpMode);
  } else {
    return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           getMode
** Descriptions:            Returns current control mode
*********************************************************************************************************/
byte MCP_getMode() {
  CAN_OPERATION_MODE mode;
  mode = mcp2518fd_OperationModeGet();
  return (byte)mode;
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
byte MCP_setMode(const byte opMode) {
  if ((CAN_OPERATION_MODE)opMode !=
      CAN_SLEEP_MODE) { // if going to sleep, the value stored in opMode is not
                        // changed so that we can return to it later
    mcpMode = (CAN_OPERATION_MODE)opMode;
  }
  return mcp2518fd_OperationModeSelect(mcpMode);
}

/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            Read message buf and can bus source ID according to
*status.
**                          Status has to be read with readRxTxStatus.
*********************************************************************************************************/
byte MCP_readMsgBufIDFull(volatile unsigned long *id,
                             volatile byte *ext, volatile byte *rtr_ptr,
                             volatile byte *len, volatile byte *buf) {

  byte r = mcp2518fd_readMsgBufID(len, buf);
  if (id)
    *id  = can_id;
  if (ext)
    *ext = ext_flg;
  if (rtr_ptr)
    *rtr_ptr = rtr;
  return r;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
byte MCP_checkReceive(void) {
  CAN_RX_FIFO_STATUS status;
  // RXnIF in Bit 1 and 0 return ((res & MCP_STAT_RXIF_MASK)? CAN_MSGAVAIL: CAN_NOMSG);
  mcp2518fd_ReceiveChannelStatusGet(APP_RX_FIFO, &status);

  byte res = (byte)(status & CAN_RX_FIFO_NOT_EMPTY_EVENT) + 2;
  return res;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
byte MCP_checkError(uint8_t* err_ptr) {
  CAN_ERROR_STATE flags;
  mcp2518fd_ErrorStateGet(&flags);
  if (err_ptr) {
    *err_ptr = (byte)flags;
  }
  return (byte)flags;
}

// /*********************************************************************************************************
// ** Function name:           readMsgBufID
// ** Descriptions:            Read message buf and can bus source ID according
// to status.
// **                          Status has to be read with readRxTxStatus.
// *********************************************************************************************************/
byte mcp2518fd_readMsgBufID(volatile byte *len, volatile byte *buf) {
  mcp2518fd_ReceiveMessageGet(APP_RX_FIFO, &rxObj, rxd, MAX_DATA_BYTES);
  ext_flg = rxObj.bF.ctrl.IDE;
  can_id = ext_flg? (rxObj.bF.id.EID | (rxObj.bF.id.SID << 18))
                  :  rxObj.bF.id.SID;
  rtr = rxObj.bF.ctrl.RTR;
  uint8_t n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
  if (len) {
    *len = n;
  }

  int i;
  for (i = 0; i < n; i++) {
    buf[i] = rxd[i];
  }
  return 0;
}

/*********************************************************************************************************
** Function name:           MCP_sendMsgBufFull
** Descriptions:            Try to send message. There is no delays for waiting
*free buffer.
*********************************************************************************************************/
byte MCP_sendMsgBufFull(unsigned long id, byte ext, byte rtr, byte len, const byte *buf) {
  return mcp2518fd_sendMsg(buf, len, id, ext, rtr, false);
}

/*********************************************************************************************************
** Function name:           clearBufferTransmitIfFlags
** Descriptions:            Clear transmit interrupt flags for specific buffer
*or for all unreserved buffers.
**                          If interrupt will be used, it is important to clear
*all flags, when there is no
**                          more data to be sent. Otherwise IRQ will newer
*change state.
*********************************************************************************************************/
void MCP_clearBufferTransmitIfFlags(byte flags) {
  mcp2518fd_TransmitChannelEventAttemptClear(APP_TX_FIFO);
  return;
}


/*********************************************************************************************************
** Function name:           readRxTxStatus
** Descriptions:            Read RX and TX interrupt bits. Function uses status
*reading, but translates.
**                          result to MCP_CANINTF. With this you can check
*status e.g. on interrupt sr
**                          with one single call to save SPI calls. Then use
*checkClearRxStatus and
**                          checkClearTxStatus for testing.
*********************************************************************************************************/
byte MCP_readRxTxStatus(void) {
  CAN_RX_FIFO_EVENT rxFlags;
  mcp2518fd_ReceiveChannelEventGet(APP_RX_FIFO, &rxFlags);
  return (byte)rxFlags;
}

/*********************************************************************************************************
** Function name:           MCP_PinMode
** Descriptions:            switch supported pins between HiZ, interrupt, output
*or input
*********************************************************************************************************/
bool MCP_PinMode(const byte pin, const byte mode) {
  int8_t spiTransferError = 1;

  // Read
  uint16_t a = cREGADDR_IOCON + 3;
  REG_IOCON iocon;
  iocon.word = 0;

  mcp2518fd_ReadByte(a, &iocon.byte[3]);

  if (pin == GPIO_PIN_0) {
    // Modify
    iocon.bF.PinMode0 = (GPIO_PIN_MODE)mode;
  }
  if (pin == GPIO_PIN_1) {
    // Modify
    iocon.bF.PinMode1 = (GPIO_PIN_MODE)mode;
  }
  // Write
  mcp2518fd_WriteByte(a, iocon.byte[3]);

  return spiTransferError;
}

/*********************************************************************************************************
** Function name:           MCP_DigitalWrite
** Descriptions:            write HIGH or LOW to RX0BF/RX1BF
*********************************************************************************************************/
bool MCP_DigitalWrite(const byte pin, const byte mode) {
  int8_t spiTransferError = 0;

  // Read
  uint16_t a = cREGADDR_IOCON + 1;
  REG_IOCON iocon;
  iocon.word = 0;

  spiTransferError = mcp2518fd_ReadByte(a, &iocon.byte[1]);
  if (spiTransferError) {
    return -1;
  }

  // Modify
  switch (pin) {
  case GPIO_PIN_0:
    iocon.bF.LAT0 = (GPIO_PIN_STATE)mode;
    break;
  case GPIO_PIN_1:
    iocon.bF.LAT1 = (GPIO_PIN_STATE)mode;
    break;
  default:
    return -1;
  }

  // Write
  spiTransferError = mcp2518fd_WriteByte(a, iocon.byte[1]);
  if (spiTransferError) {
    return -2;
  }

  return spiTransferError;
}

/*********************************************************************************************************
** Function name:           MCP_DigitalWrite
** Descriptions:            read HIGH or LOW from supported pins
*********************************************************************************************************/
byte MCP_DigitalRead(const byte pin) {
  GPIO_PIN_STATE state;

  // Read
  REG_IOCON iocon;
  iocon.word = 0;

  mcp2518fd_ReadByte(cREGADDR_IOCON + 2, &iocon.byte[2]);

  // Update data
  switch (pin) {
  case GPIO_PIN_0:
    state = (GPIO_PIN_STATE)iocon.bF.GPIO0;
    break;
  case GPIO_PIN_1:
    state = (GPIO_PIN_STATE)iocon.bF.GPIO1;
    break;
  default:
    return (byte)-1;
  }

  return (byte)state;;
}

/* CANFD Auxiliary helper */
byte CANFD_dlc2len(byte dlc) {
  if (dlc <= CAN_DLC_8)
    return dlc;
  switch (dlc) {
  case CAN_DLC_12: return 12;
  case CAN_DLC_16: return 16;
  case CAN_DLC_20: return 20;
  case CAN_DLC_24: return 24;
  case CAN_DLC_32: return 32;
  case CAN_DLC_48: return 48;
  default:
  case CAN_DLC_64: return 64;
  }
}

byte CANFD_len2dlc(byte len) {
  if (len <= CAN_DLC_8)
    return len;
  else if (len <= 12) return CAN_DLC_12;
  else if (len <= 16) return CAN_DLC_16;
  else if (len <= 20) return CAN_DLC_20;
  else if (len <= 24) return CAN_DLC_24;
  else if (len <= 32) return CAN_DLC_32;
  else if (len <= 48) return CAN_DLC_48;
  return CAN_DLC_64;
}
