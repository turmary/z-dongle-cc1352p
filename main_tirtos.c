/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,

 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>
#include <stdio.h>

/* POSIX Header files */
#include <pthread.h>
#include <unistd.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "ti_drivers_config.h"

#include "mcp2518fd/mcp2518fd_can.h"

extern void *RFThread(void *arg0);
void *mainThread(void *arg0);

/* Stack size in bytes */
#define THREADSTACKSIZE    2096

Display_Handle display;

/*
 *  ======== main ========
 */
int main(void)
{
    pthread_t           thread;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions */
    Board_initGeneral();

    /* Set priority and stack size attributes */
    pthread_attr_init(&attrs);
    priParam.sched_priority = 1;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    pthread_attr_setschedparam(&attrs, &priParam);

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    retc = pthread_create(&thread, &attrs, mainThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    BIOS_start();

    return (0);
}

void *CANThread(void *arg0);

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions. */
    Display_init();

    GPIO_init();
    /* Free PIN for LED-RED & LED-GREEN which used by RF-thread */
    GPIOCC26xx_release(CONFIG_GPIO_RLED_CONST);
    GPIOCC26xx_release(CONFIG_GPIO_GLED_CONST);

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    Display_printf(display, 0, 0, "%s(): %s %s\r\n", __func__, __TIME__,  __DATE__);

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create RF thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, RFThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    CANThread(NULL);

    for (;;) {
        usleep(1000);
    }

    return (NULL);
}

#define MAX_DATA_SIZE 8

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;
byte cdata[MAX_DATA_SIZE] = {0};

void *CANThread(void *arg0) {

    // Zola_Dongle_v1.0_CC1352P
    // MCP_begin(CAN_500KBPS, MCP2518FD_20MHz);

    // CANBUS(FD) HAT for Raspberry Pi
    MCP_begin(CAN_500KBPS, MCP2518FD_40MHz);

    for (;;) {
        // check if data coming
        if (CAN_MSGAVAIL != MCP_checkReceive()) {
            sleep(1);
            continue;
        }

        char prbuf[32 + MAX_DATA_SIZE * 3];
        int i, n;

        unsigned long t = (1UL * Clock_getTicks() * Clock_tickPeriod) / 1000UL;
        // read data, len: data length, buf: data buf
        MCP_readMsgBuf(&len, cdata);

        id = MCP_getCanId();
        type = (MCP_isExtendedFrame() << 0) |
               (MCP_isRemoteRequest() << 1);
        /*
         * MCP2515(or this driver) could not handle properly
         * the data carried by remote frame
         */

        n = sprintf(prbuf, "%04lu.%03d ", t / 1000, (int)(t % 1000));
        /* Displayed type:
         *
         * 0x00: standard data frame
         * 0x02: extended data frame
         * 0x30: standard remote frame
         * 0x32: extended remote frame
         */
        static const byte type2[] = {0x00, 0x02, 0x30, 0x32};
        n += sprintf(prbuf + n, "RX: [%08lX](%02X) ", (unsigned long)id, type2[type]);
        // n += sprintf(prbuf, "RX: [%08lX](%02X) ", id, type);

        for (i = 0; i < len; i++) {
            n += sprintf(prbuf + n, "%02X ", cdata[i]);
        }
        Display_printf(display, 0, 0, prbuf);
    }
}
