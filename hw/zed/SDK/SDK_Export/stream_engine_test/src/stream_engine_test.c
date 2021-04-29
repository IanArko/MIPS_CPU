/*
 * Copyright (c) 2009-2012 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * stream_engine_test.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "stdlib.h"
#include "dma.h"

static const size_t NUM_INSTR_WORDS = 64;
static const size_t NUM_DATA_WORDS = 32768;     /* 15-bit address space */
static const u32 TEST_PATTERN = 0xc001cafe;

// MIPS buffer copy program
static const u32 instructions[] = {
    0x3c088000,
    0x3c098002,
    0x08000007,
    0x00000000,
    0x8d0a0000,
    0xad0a0000,
    0x25080004,
    0x1509fffc,
    0x00000000,
    0x3c088002,
    0x25080004,
    0x24020001,
    0xad020000,
    0x3c088002,
    0x25080000,
    0x24020001,
    0xad020000,
    0x08000011,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000
};


int main()
{
    init_platform();
    printf("------------------------------------------------------------------------------\n");
    dma_init();

    /* Assert MIPS reset */
    Xil_Out32(se_command, 2);

    /* Allocate insruction buffer. */
    printf("Allocating instruction buffer of size %d bytes\n", (NUM_INSTR_WORDS * sizeof(u32)));

    u32 *instrBuffer = calloc(NUM_INSTR_WORDS, sizeof(u32));
    if (instrBuffer == NULL) {
        printf("Failed to allocate instruction buffer!\n");
        goto out;
    }

    memcpy(instrBuffer, instructions, sizeof(instructions));

    /* Allocate the TX/RX buffers. */
    printf("Allocating TX/RX buffers of size %d bytes each\n", (NUM_DATA_WORDS * sizeof(u32)));

    u32 *txBuffer = (u32 *) malloc(NUM_DATA_WORDS * sizeof(u32));
    if (txBuffer == NULL) {
        printf("Failed to allocate TX buffer!\n");
        goto out_free_instr;
    }

    u32 *rxBuffer = (u32 *) malloc(NUM_DATA_WORDS * sizeof(u32));
    if (rxBuffer == NULL) {
        printf("Failed to allocate RX buffer!\n");
        goto out_free_tx;
    }

    /* Populate TX buffer with test pattern. */
    u32 pattern = TEST_PATTERN;
    for (int i = 0; i < NUM_DATA_WORDS; i++) {
        txBuffer[i] = (u32) pattern;
        pattern = (pattern << 4) | (pattern >> 28);
    }

    /* DMA the instruction buffer to the MIPS processor. */
    int result;
    if ((result = dma_send_instructions(instrBuffer, NUM_INSTR_WORDS)) != XST_SUCCESS) {
        printf("DMA of instruction buffer failed with result code %d\n", result);
        goto done;
    } else {
        printf("DMA transfer to instruction buffer okay!\n");
    }

    /* DMA the transmit buffer to the stream engine's input buffer. */
    if ((result = dma_send_data(txBuffer, NUM_DATA_WORDS)) != XST_SUCCESS) {
        printf("DMA send failed with result code %d\n", result);
        goto done;
    } else {
        printf("DMA transfer to input buffer okay!\n");
    }

#if 1
    u32 status = Xil_In32(se_status);
    printf("Status register = %lu\n", status);

    /* Start the MIPS processor. */
    printf("Starting MIPS...\n");
    Xil_Out32(se_command, 1);
    printf("Started\n");

    /* Wait for completion signal. */
    printf("Waiting...\n");
    while (status != 1)
        status = Xil_In32(se_status);
    printf("Status register = %lu\n", status);

    /* Stop the MIPS processor. */
    printf("Stopping MIPS...\n");
    Xil_Out32(se_command, 0);
    printf("Stopped\n");
#endif

    /* DMA the stream engine's output buffer to the receive buffer. */
    if ((result = dma_receive_data(rxBuffer, NUM_DATA_WORDS)) != XST_SUCCESS) {
        printf("DMA receive failed with result code %d\n", result);
        goto done;
    } else {
        printf("DMA transfer from output buffer okay!\n");
    }

    /* Compare the RX/TX buffers. */
    int passed = 1;
    for (int i = 0; i < NUM_DATA_WORDS; i++) {
        if (txBuffer[i] != rxBuffer[i]) {
            passed = 0;
            printf("TEST FAILED!\n");
            printf("\toutput buffer does not match input buffer at word %d\n", i);
            printf("\texpected 0x%08lx, was 0x%08lx\n", txBuffer[i], rxBuffer[i]);
            break;
        }
    }

    if (passed) {
        printf("TEST PASSED!\n");
        printf("\tCongratulations on getting this to work!\n");
    }

  done:
    printf("Free RX buffer\n");
    free(rxBuffer);
  out_free_tx:
    printf("Free TX buffer\n");
    free(txBuffer);
  out_free_instr:
    printf("Free instruction buffer\n");
    free(instrBuffer);
  out:
    return 0;
}
