

#ifndef __MCP2518FD_CAN_H__
#define __MCP2518FD_CAN_H__

#include <stdint.h>
#include <stdbool.h>

#define CAN_OK              (0)
#define CAN_FAILINIT        (1)
#define CAN_FAILTX          (2)
#define CAN_MSGAVAIL        (3)
#define CAN_NOMSG           (4)
#define CAN_CTRLERROR       (5)
#define CAN_GETTXBFTIMEOUT  (6)
#define CAN_SENDMSGTIMEOUT  (7)
#define CAN_FAIL            (0xff)

// clock
typedef enum {
    MCP_NO_MHz,
    /* apply to MCP2515 */
    MCP_16MHz,
    MCP_8MHz,
    /* apply to MCP2518FD */
    MCP2518FD_40MHz = MCP_16MHz /* To compatible MCP2515 shield */,
    MCP2518FD_20MHz,
    MCP2518FD_10MHz,
} MCP_CLOCK_T;

#include "mcp2518fd_can_dfs.h"

typedef enum {
    CAN_NOBPS,
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_25KBPS,
    CAN_31K25BPS,
    CAN_33KBPS  ,
    CAN_40KBPS  ,
    CAN_50KBPS  ,
    CAN_80KBPS  ,
    CAN_83K3BPS ,
    CAN_95KBPS  ,
    CAN_100KBPS ,
    CAN_125KBPS ,
    CAN_200KBPS ,
    CAN_250KBPS ,
    CAN_500KBPS ,
    CAN_666KBPS ,
    CAN_800KBPS ,
    CAN_1000KBPS
} MCP_BITTIME_SETUP;

typedef uint8_t byte;

// *****************************************************************************
// *****************************************************************************
//! Reset DUT

// Code anchor for break points
#define Nop() asm("nop")

// Index to SPI channel
// Used when multiple MCP25xxFD are connected to the same SPI interface, but
// with different CS
#define SPI_DEFAULT_BUFFER_LENGTH 96

// extern SPIClass* pSPI;
#define spi_readwrite pSPI->transfer
#define spi_read() spi_readwrite(0x00)
#define spi_write(spi_val) spi_readwrite(spi_val)
#define SPI_BEGIN()                                                            \
  pSPI->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0))
#define SPI_END() pSPI->endTransaction();

// *****************************************************************************
// *****************************************************************************
// Section: Defines


class SPIClass;

class mcp2518fd : public MCP_CAN {
public:
  mcp2518fd(byte _CS) : MCP_CAN(_CS), nReservedTx(0){};

public:
  virtual void
  enableTxInterrupt(bool enable = true); // enable transmit interrupt
  virtual void reserveTxBuffers(byte nTxBuf = 0) {
    nReservedTx = (nTxBuf < 3 ? nTxBuf : 3 - 1);
  }
  virtual byte getLastTxBuffer() {
    return 3 - 1; // read index of last tx buffer
  }
  /*
   * speedset could be in MCP_BITTIME_SETUP,
   *          or fill by CANFD::BITRATE()
   */
  virtual byte begin(uint32_t speedset,
                     const byte clockset = MCP2518FD_40MHz); // init can
  virtual byte init_Mask(byte num, byte ext, unsigned long ulData);
  virtual byte init_Filt(byte num, byte ext,
                         unsigned long ulData); // init filters
  virtual void setSleepWakeup(const byte enable);
  virtual byte sleep();
  virtual byte wake();
  virtual byte setMode(const byte opMode);
  virtual byte getMode();
  virtual byte checkError(uint8_t* err_ptr = NULL);

  /* ---- receiving ---- */
  virtual byte checkReceive(void);
  virtual byte readMsgBufID(byte status, volatile unsigned long *id,
                            volatile byte *ext, volatile byte *rtr,
                            volatile byte *len,
                            volatile byte *buf); // read buf with object ID
    /* wrapper */
    byte readMsgBufID(unsigned long *ID, byte *len, byte *buf) {
        return readMsgBufID(readRxTxStatus(), ID, &ext_flg, &rtr, len, buf);
    }
    byte readMsgBuf(byte *len, byte *buf) {
        return readMsgBufID(readRxTxStatus(), &can_id, &ext_flg, &rtr, len, buf);
    }

  /* ---- sending ---- */
  /* dlc = CAN_DLC_0..CAN_DLC_64(0..15)
   *       CAN_DLC_0..CAN_DLC_8(0..8) = data bytes count, compatible to MCP2515 APIs
   *       CAN_DLC_12(9)              = 12 bytes
   *       CAN_DLC_16(10)             = 16 bytes
   *       CAN_DLC_20(11)             = 20 bytes
   *       CAN_DLC_24(12)             = 24 bytes
   *       CAN_DLC_32(13)             = 32 bytes
   *       CAN_DLC_48(14)             = 48 bytes
   *       CAN_DLC_64(15)             = 64 bytes
   */
  virtual byte trySendMsgBuf(unsigned long id, byte ext, byte rtr, byte dlc,
                             const byte *buf, byte iTxBuf = 0xff);
  virtual byte sendMsgBuf(byte status, unsigned long id, byte ext, byte rtr,
                          byte dlc, volatile const byte *buf);
  virtual byte sendMsgBuf(unsigned long id, byte ext, byte rtr, byte dlc,
                          const byte *buf, bool wait_sent = true);
  /* wrapper */
  inline byte sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf) {
    return sendMsgBuf(id, ext, 0, len, buf, true);
  }

  virtual void clearBufferTransmitIfFlags(byte flags = 0);
  virtual byte readRxTxStatus(void);
  virtual byte checkClearRxStatus(byte *status);
  virtual byte checkClearTxStatus(byte *status, byte iTxBuf = 0xff);
  virtual bool mcpPinMode(const byte pin, const byte mode);
  virtual bool mcpDigitalWrite(const byte pin, const byte mode);
  virtual byte mcpDigitalRead(const byte pin);
};

/* CANFD Auxiliary helper */
class CANFD {
public:
  static byte dlc2len(byte dlc);
  static byte len2dlc(byte len);
  static uint32_t BITRATE(uint32_t arbitration, uint8_t factor) {
    return ((uint32_t)factor << 24) | (arbitration & 0xFFFFFUL);
  }
};

#endif