/*
 * File         : sobel.c
 * Project      : EE180 MIPS lab
 * Creator(s)   : Raghu Prabhaakar and Grant Ayers
 *
 * Standards/Formatting:
 *   C99, 2 soft tab, wide column.
 *
 * Description:
 *  Runs a sobel edge detection convolution on an input
 *  image buffer and writes the results to an output buffer.
 *
 *  The sobel function is generic, but the 4-way control
 *  handshake is specific to the lab implementation.
 */
#include <stdlib.h>
#include <stdint.h>

#define IN_BUF_ADDR     0x80000000      // Input image buffer (read-only)
#define OUT_BUF_ADDR    0x80000000      // Output image buffer (write-only)
#define CMD_REG_ADDR    0x80020008      // Control register containing 'go' command
#define STATUS_REG_ADDR 0x80020000      // Status register for setting 'done'
#define YES             1               // Active high: 1

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define BORDER 1
#define IMG_WIDTH_BORDER (IMG_WIDTH+2*BORDER)
#define IMG_HEIGHT_BORDER (IMG_HEIGHT+2*BORDER)

/* Inefficient sobel, adapted from lab1's 'sobelCalc' function in sobel_calc.cpp */
void sobel(uint8_t *inbuf, uint8_t *outbuf, uint32_t rows, uint32_t cols)
{
  uint32_t i = 0, j = 0;

  // Calculate the x and y convolutions
  for (i=1; i<=rows; i++) {
    for (j=1; j<=cols; j++) {
      uint16_t sobel_x, sobel_y, sobel_xy;
      sobel_x = abs(inbuf[IMG_WIDTH_BORDER*(i-1) + (j-1)]-
		  inbuf[IMG_WIDTH_BORDER*(i+1) + (j-1)] +
		  2*inbuf[IMG_WIDTH_BORDER*(i-1) + (j)] -
		  2*inbuf[IMG_WIDTH_BORDER*(i+1) + (j)] +
		  inbuf[IMG_WIDTH_BORDER*(i-1) + (j+1)] -
		  inbuf[IMG_WIDTH_BORDER*(i+1) + (j+1)]);
      sobel_x = (sobel_x > 255) ? 255 : sobel_x;

      sobel_y = abs(inbuf[IMG_WIDTH_BORDER*(i-1) + (j-1)] -
		   inbuf[IMG_WIDTH_BORDER*(i-1) + (j+1)] +
		   2*inbuf[IMG_WIDTH_BORDER*(i) + (j-1)] -
		   2*inbuf[IMG_WIDTH_BORDER*(i) + (j+1)] +
		   inbuf[IMG_WIDTH_BORDER*(i+1) + (j-1)] -
		   inbuf[IMG_WIDTH_BORDER*(i+1) + (j+1)]);
      sobel_y = (sobel_y > 255) ? 255 : sobel_y;
      sobel_xy = sobel_x + sobel_y;
      sobel_xy = (sobel_xy > 255) ? 255 : sobel_xy;
      outbuf[IMG_WIDTH*(i-1) + (j-1)] = sobel_xy;
    }
  }
}

void start_wait(uint32_t *rows, uint32_t *cols)
{
  volatile uint32_t *cmd_reg_addr = (volatile uint32_t *)CMD_REG_ADDR;
  uint32_t cmd;

  // Wait for 'go'
  while (((*cmd_reg_addr) & 0x1) != YES);

  // Read the rows and columns from the command register
  cmd = *cmd_reg_addr;
  *rows = (cmd >> 2) & 0x3ff;
  *cols = (cmd >> 12) & 0x3ff;
}

void done_wait()
{
  volatile uint32_t *cmd_reg_addr    = (volatile uint32_t *)CMD_REG_ADDR;
  volatile uint32_t *status_reg_addr = (volatile uint32_t *)STATUS_REG_ADDR;

  // Set 'done' and wait for 'go' to drop
  *status_reg_addr = YES;
  while (((*cmd_reg_addr) & 0x1) == YES);

  // Drop 'done'
  *status_reg_addr = ~YES;
}

int main()
{
  volatile uint32_t *cmd_reg_addr = (volatile uint32_t *)CMD_REG_ADDR;
  uint8_t *inbuf;//  = (uint8_t *)IN_BUF_ADDR;
  uint8_t *outbuf;// = (uint8_t *)OUT_BUF_ADDR;
  uint32_t cmd, rows, cols;

  while (1) {
    // Wait for 'go'
    start_wait(&rows, &cols);

    // Read the rows and columns from the command register
    cmd = *cmd_reg_addr;
    rows = (cmd >> 2) & 0x3ff;
    cols = (cmd >> 12) & 0x3ff;
    inbuf = (uint8_t *)(IN_BUF_ADDR | (cmd >> 24));   // prevent address optimization (nop)
    outbuf = (uint8_t *)(OUT_BUF_ADDR | (cmd >> 25));  // prevent address optimization (nop)

    // Run sobel
    sobel(inbuf, outbuf, rows, cols);
    //sobel(inbuf, outbuf, 640, 162);

    // Indicate 'done' and drop it when 'go' is dropped
    done_wait();
  }
  return 0;
}
