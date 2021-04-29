#include <stdio.h>
#include "platform.h"
#include "xaxidma.h"
#include "stdlib.h"

static const u32 se_command = XPAR_STREAM_ENGINE_0_BASEADDR;
static const u32 se_input_buf_select = XPAR_STREAM_ENGINE_0_BASEADDR + sizeof(u32);
static const u32 se_packet_size = XPAR_STREAM_ENGINE_0_BASEADDR + 2*sizeof(u32);
static const u32 se_start_address_read = XPAR_STREAM_ENGINE_0_BASEADDR + 3*sizeof(u32);
static const u32 se_start_address_write = XPAR_STREAM_ENGINE_0_BASEADDR + 4*sizeof(u32);

static XAxiDma dma;

int dma_init()
{
    XAxiDma_Config *dma_cfg_ptr = XAxiDma_LookupConfig(XPAR_AXI_DMA_STREAM_DEVICE_ID);
    int result = XAxiDma_CfgInitialize(&dma, dma_cfg_ptr);
    if (result != XST_SUCCESS) {
        printf("Error initializing DMA module.\n");
        return result;
    }

    printf("Initialized DMA module. Resetting...\n");
    XAxiDma_Reset(&dma);
    while (!XAxiDma_ResetIsDone(&dma)) {};
    printf("DMA module reset.\n");

    return XST_SUCCESS;
}

int dma_send_instructions(const u32 *instr_buf, u32 n_instr)
{
    Xil_Out32(se_input_buf_select, 1); // select instruction buffer
    Xil_Out32(se_start_address_write, 0); // set input data start address to 0

    printf("The selected input buffer is %lu\n", Xil_In32(se_input_buf_select));
    printf("The start address for writing is %lu\n", Xil_In32(se_start_address_write));

    Xil_DCacheFlushRange((unsigned int) instr_buf, n_instr * sizeof(u32));
    int result = XAxiDma_SimpleTransfer(&dma, (u32)instr_buf, n_instr * sizeof(u32), XAXIDMA_DMA_TO_DEVICE);

    if (result != XST_SUCCESS) {
        printf("Simple transfer of instruction buffer failed.\n");
        return result;
    }
    while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE)) {};

    return XST_SUCCESS;
}

int dma_send_data(const u32 *data_buf, u32 n_data)
{
    Xil_Out32(se_input_buf_select, 2); // select input data buffer
    Xil_Out32(se_start_address_write, 0); // set input data start address to 0

    printf("The selected input buffer is %lu\n", Xil_In32(se_input_buf_select));
    printf("The start address for writing is %lu\n", Xil_In32(se_start_address_write));

    Xil_DCacheFlushRange((unsigned int)data_buf, n_data * sizeof(u32));
    int result = XAxiDma_SimpleTransfer(&dma, (u32)data_buf, n_data * sizeof(u32), XAXIDMA_DMA_TO_DEVICE);
    if (result != XST_SUCCESS) {
        printf("Simple transfer of data buffer failed.\n");
        return result;
    }
    while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE)) {};

    return XST_SUCCESS;
}

int dma_receive_data(u32 *out_buf, u32 n_data)
{
    Xil_Out32(se_start_address_read, 0); // set input data start address to 0
    Xil_Out32(se_packet_size, n_data); // set transfer size to the data size (in words)
    Xil_DCacheFlushRange((unsigned int)out_buf, n_data * sizeof(u32));

    printf("The start address for reading is %lu\n", Xil_In32(se_start_address_read));
    printf("The packet size is %lu\n", Xil_In32(se_packet_size));

    int result = XAxiDma_SimpleTransfer(&dma, (u32)out_buf, n_data * sizeof(u32), XAXIDMA_DEVICE_TO_DMA);
    if (result != XST_SUCCESS) {
        printf("Simple transfer of output data buffer failed.\n");
        return result;
    }
    while (XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA)) {};

    Xil_DCacheInvalidateRange((unsigned int)out_buf, n_data * sizeof(u32));
    return XST_SUCCESS;
}

int main()
{
    init_platform();

    static const u32 n_data = 64;
    static const u32 n_instr = 10;
    static const u32 n_instr_buf = 64;

    const u32 instrs[10] = {
        0x3c088000, // lui $t0, 0x8000
        0x25090100, // addiu $t1, $t0, 64
        0x8d0a0000, // lw $t2, 0($t0)
        0x000a5100, // sll $t2, $t2, 4
        0xad0a0000, // sw $t2, 0($t0)
        0x25080004, // addiu $t0, $t0, 4
        0x1509fffb, // bne $t0, $t1, -5   # goes to lw $t2
        0x00000000, // nop
        0x08000008, // jump to self
        0x00000000  // nop
    };

    u32 *instr_buf = (u32 *)calloc(n_instr_buf, sizeof(u32));

    memcpy(instr_buf, instrs, n_instr * sizeof(u32));

    u32 *data_buf = (u32 *)malloc(n_data * sizeof(u32));
    u32 *out_data_buf = (u32 *)malloc(n_data * sizeof(u32));

    for (int i = 0; i < n_data; ++i) {
        data_buf[i] = 2*(i+1);
    }

    //Xil_Out32(se_command, 2); // reset the MIPS processor

    dma_init();

    printf("The DMA module has a data width of: %d\n", dma.TxBdRing.DataWidth);
    dma_send_instructions(instr_buf, n_instr_buf);
    dma_send_data(data_buf, n_data);

    Xil_Out32(se_command, 1); // start the MIPS processor

    printf("The input data is:\n");
    for (int i = 0; i < n_data; ++i) {
        printf("%lu ", data_buf[i]);
    }
    printf("\n");

    Xil_Out32(se_command, 0); // stop the MIPS processor


    printf("The original output data buffer contains: ");
    for (int i = 0; i < n_data; ++i) {
        out_data_buf[i] = i;
        printf("%lu ", out_data_buf[i]);
    }
    printf("\n");


    dma_receive_data(out_data_buf, n_data);

    printf("The output data is:\n");
    for (int i = 0; i < n_data; ++i) {
        printf("%lu ", out_data_buf[i]);
    }
    printf("\n");

    return 0;
}
