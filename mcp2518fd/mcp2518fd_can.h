

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

    extern unsigned long can_id;
    extern uint8_t rtr, ext_flg;

void MCP_enableTxInterrupt(bool enable); // enable transmit interrupt
  /*
   * speedset could be in MCP_BITTIME_SETUP,
   *          or fill by CANFD::BITRATE()
   */
byte MCP_begin(uint32_t speedset,
               const byte clockset/* = MCP2518FD_20MHz*/); // init can driver
byte MCP_init_Mask(byte num, byte ext, unsigned long ulData);
byte MCP_init_Filt(byte num, byte ext,
                         unsigned long ulData); // init filters
void MCP_setSleepWakeup(const byte enable);
byte MCP_sleep();
byte MCP_wake();
byte MCP_setMode(const byte opMode);
byte MCP_getMode();
byte MCP_checkError(uint8_t* err_ptr);


  /* ---- receiving ---- */
byte MCP_checkReceive(void);
byte MCP_readMsgBufIDFull(volatile unsigned long *id,
                            volatile byte *ext, volatile byte *rtr,
                            volatile byte *len,
                            volatile byte *buf); // read buf with object ID
    /* wrapper */
    static inline byte MCP_readMsgBufID(unsigned long *ID, byte *len, byte *buf) {
        return MCP_readMsgBufIDFull(ID, &ext_flg, &rtr, len, buf);
    }
    static inline byte MCP_readMsgBuf(byte *len, byte *buf) {
        return MCP_readMsgBufIDFull(&can_id, &ext_flg, &rtr, len, buf);
    }

    /* could be used after a successful call to MCP_readMsgBuf() */
    static inline unsigned long MCP_getCanId(void) { return can_id; }
    static inline byte MCP_isRemoteRequest(void)   { return rtr;    }
    static inline byte MCP_isExtendedFrame(void)   { return ext_flg;}

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
byte MCP_sendMsgBufFull(unsigned long id, byte ext, byte rtr, byte dlc,
                        const byte *buf);
  /* wrapper */
  static inline byte MCP_sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf) {
    return MCP_sendMsgBufFull(id, ext, 0, len, buf);
  }

void MCP_clearBufferTransmitIfFlags(byte flags);
byte MCP_readRxTxStatus(void);
bool MCP_PinMode(const byte pin, const byte mode);
bool MCP_DigitalWrite(const byte pin, const byte mode);
byte MCP_DigitalRead(const byte pin);

/* CANFD Auxiliary helper */
byte CANFD_dlc2len(byte dlc);
byte CANFD_len2dlc(byte len);
  static inline uint32_t CANFD_BITRATE(uint32_t arbitration, uint8_t factor) {
    return ((uint32_t)factor << 24) | (arbitration & 0xFFFFFUL);
  }

#endif//__MCP2518FD_CAN_H__
