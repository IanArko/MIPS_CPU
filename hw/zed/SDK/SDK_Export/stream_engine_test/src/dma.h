#ifndef DMA_H_
#define DMA_H_

#include <stdio.h>
#include "platform.h"
#include "xaxidma.h"
#include "stdlib.h"

static const u32 se_command = XPAR_STREAM_ENGINE_0_BASEADDR;
static const u32 se_input_buf_select = XPAR_STREAM_ENGINE_0_BASEADDR + sizeof(u32);
static const u32 se_packet_size = XPAR_STREAM_ENGINE_0_BASEADDR + 2*sizeof(u32);
static const u32 se_start_address_read = XPAR_STREAM_ENGINE_0_BASEADDR + 3*sizeof(u32);
static const u32 se_start_address_write = XPAR_STREAM_ENGINE_0_BASEADDR + 4*sizeof(u32);
static const u32 se_test = XPAR_STREAM_ENGINE_0_BASEADDR + 5*sizeof(u32);
static const u32 se_status = XPAR_STREAM_ENGINE_0_BASEADDR + 6*sizeof(u32);

int dma_init();
int dma_send_instructions(const u32 *instr_buf, u32 n_instr);
int dma_send_data(const u32 *data_buf, u32 n_data);
int dma_receive_data(u32 *out_buf, u32 n_data);

#endif /* DMA_H_ */
