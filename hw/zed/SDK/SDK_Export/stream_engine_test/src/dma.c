#include "dma.h"

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
