/**
 * Copyright (C) 2021 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

/* DMA Proxy Test Application
 *
 * This application is intended to be used with the DMA Proxy device driver. It provides
 * an example application showing how to use the device driver to do user space DMA
 * operations.
 *
 * The driver allocates coherent memory which is non-cached in a s/w coherent system
 * or cached in a h/w coherent system.
 *
 * Transmit and receive buffers in that memory are mapped to user space such that the
 * application can send and receive data using DMA channels (transmit and receive).
 *
 * It has been tested with AXI DMA and AXI MCDMA systems with transmit looped back to
 * receive. Note that the receive channel of the AXI DMA throttles the transmit with
 * a loopback while this is not the case with AXI MCDMA.
 *
 * Build information: The pthread library is required for linking. Compiler optimization
 * makes a very big difference in performance with -O3 being good performance and
 * -O0 being very low performance.
 *
 * The user should tune the number of channels and channel names to match the device
 * tree.
 *
 * More complete documentation is contained in the device driver (dma-proxy.c).
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <sys/param.h>

#include "dma-proxy.h"

/* The user must tune the application number of channels to match the proxy driver device tree
 * and the names of each channel must match the dma-names in the device tree for the proxy
 * driver node. The number of channels can be less than the number of names as the other
 * channels will just not be used in testing.
 */
#define TX_CHANNEL_COUNT 0
#define RX_CHANNEL_COUNT 1


#define LVDS_DELAY  0x41230000
#define DMA0_EN     0x41200000
#define DMA0_GATHER 0x41220000

#define GPIO_SIZE  0x10000    // the size of the gpio controller memory region
#define GPIO_DATA  0x0        // the offset of the data register
#define GPIO_TRI   0x4        // the offset of the tri-state register

#define PHYSICAL_BASE_ADDRESS_DMA1   0x40400030
#define PHYSICAL_BASE_ADDRESS_DMA2   0x40410030
#define MM2S_DMA_STATUS              0x04
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#define POINT_BYTE ((31 * 1024 * 1024) - 2) * 32 / 4

static volatile void *g_dma1_virtual_addr = NULL;
static volatile void *g_dma2_virtual_addr = NULL;
static void *g_dma1_map_base = NULL;
static void *g_dma2_map_base = NULL;
volatile unsigned long *mm2s_dma1_status_reg = NULL;
volatile unsigned long *mm2s_dma2_status_reg = NULL;

int gpio_fd; // the file descriptor for /dev/mem
void *gpio; // the pointers to the mapped gpio controller addresses
unsigned int *gpio_data; // the pointers to the data registers
unsigned int *gpio_tri; // the pointers to the tri-state registers

const char *tx_channel_names[] = { "dma_proxy_tx_0", "dma_proxy_tx_1"/* add unique channel names here */ };
const char *rx_channel_names[] = { "dma_proxy_rx_only_0", "dma_proxy_rx_only_1"/* add unique channel names here */ };

#define IOCTL_WAIT _IO('q', 1)

/* Internal data which should work without tuning */

struct channel {
	struct channel_buffer *buf_ptr;
	int fd;
	pthread_t tid;
	int buffer_id;
};

static int verify;
static int test_size;
static volatile int stop = 0;
int num_transfers;

struct channel tx_channels[TX_CHANNEL_COUNT], rx_channels[RX_CHANNEL_COUNT];

/*******************************************************************************************************************/
/* Handle a control C or kill, maybe the actual signal number coming in has to be more filtered?
 * The stop should cause a graceful shutdown of all the transfers so that the application can
 * be started again afterwards.
 */
void sigint(int a)
{
	stop = 1;
}

int gpio_set_value(int gpio_addr, int gpio_value)
{
	// open /dev/mem with read/write access
	gpio_fd = open("/dev/mem", O_RDWR);
	if (gpio_fd < 0) {
		perror("Open /dev/mem failed\n");
		return -1;
	}

	// map the first gpio controller address to user space
	gpio = mmap(NULL, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, gpio_fd, gpio_addr);
	if (gpio == MAP_FAILED) {
		perror("Map gpio failed\n");
		close(gpio_fd);
		return -1;
	}

	// get the pointers to the data and tri-state registers
	gpio_data = (unsigned int *)(gpio + GPIO_DATA);
	gpio_tri = (unsigned int *)(gpio + GPIO_TRI);

	// set the first gpio line of each controller to output
	*gpio_tri &= ~(1 << 0); // clear bit 0 of tri-state register

	// set the value of the first gpio line of each controller to 1
	if (gpio_value == 1){
		*gpio_data |= (1 << 0); // set bit 0 of data register
	}else{
		*gpio_data &= ~(1 << 0); // set bit 0 of data register
	}

	printf("set gpio addr: 0x%x, data: %d\n", gpio_addr, (int)*gpio_data);

	// unmap and close the gpio controller addresses and /dev/mem file
	munmap(gpio, GPIO_SIZE);
	close(gpio_fd);
	return 0;
}

static void *get_map_virtual_addr(off_t physical_addrss)
{
    int fd;
    void *map_base = NULL;
    void *virt_addr = NULL;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        printf("/dev/mem opened error(%s).\n", strerror(errno));
        fflush(stdout);
        // virt_addr = NULL;
        goto error;
    }

    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, physical_addrss & ~MAP_MASK);
    if (map_base == MAP_FAILED) {
        printf("mmap 0x%x error(%s).\n", physical_addrss, strerror(errno));
        fflush(stdout);
        // virt_addr = NULL;
        map_base = NULL;
        goto error;
    }

    close(fd);

error:
    return map_base;
}

static void munmap_virtual_addr(void *addr)
{
    if (munmap(addr, MAP_SIZE) == -1) {
        printf("munmap %p error(%s).\n", addr, strerror(errno));
        fflush(stdout);
    }
}

static void printf_dma_status_reg()
{
    g_dma1_virtual_addr  = g_dma1_map_base + ((unsigned long)PHYSICAL_BASE_ADDRESS_DMA1 & MAP_MASK);
    g_dma2_virtual_addr  = g_dma2_map_base + ((unsigned long)PHYSICAL_BASE_ADDRESS_DMA2 & MAP_MASK);
    mm2s_dma1_status_reg = (unsigned long *)(g_dma1_virtual_addr + MM2S_DMA_STATUS);
    mm2s_dma2_status_reg = (unsigned long *)(g_dma2_virtual_addr + MM2S_DMA_STATUS);
	printf("mm2s_dma1_status_reg: 0x%08lX , mm2s_dma2_status_reg: 0x%08lX\n", *mm2s_dma1_status_reg, *mm2s_dma2_status_reg);
}

static void open_rx_channel()
{
	int i;
	for (i = 0; i < RX_CHANNEL_COUNT; i++) {
		char channel_name[64] = "/dev/";
		strcat(channel_name, rx_channel_names[i]);
		rx_channels[i].fd = open(channel_name, O_RDWR);
		if (rx_channels[i].fd < 1) {
			printf("Unable to open DMA proxy device file: %s\r", channel_name);
		}

		rx_channels[i].buffer_id = i;

		rx_channels[i].buf_ptr = (struct channel_buffer *)mmap(NULL, POINT_BYTE,
										PROT_READ | PROT_WRITE, MAP_SHARED, rx_channels[i].fd, 0);

		printf("rx_channels.buf_ptr mmap %d\n", POINT_BYTE);
		if (rx_channels[i].buf_ptr == MAP_FAILED) {
			printf("Failed to mmap rx channel\n");
		}
	}
}

static void close_rx_channel()
{
	int i;
	for (i = 0; i < RX_CHANNEL_COUNT; i++) {
		munmap(rx_channels[i].buf_ptr, POINT_BYTE);
		close(rx_channels[i].fd);
	}
}

static int count_two_complement(unsigned int data_value)
{
	unsigned long one_complement_value = data_value -1;
	unsigned long sign_and_magnitude_value = 0;
	unsigned long temp = 0;
	int i;
    if (data_value & (1 << 17)) { // 检查第18位是否为1
		for (i=0; i< 17; i++) {
			temp = one_complement_value >> i;
			temp = ~temp;
			temp &= 0x01;
			temp = temp << i;
			sign_and_magnitude_value += temp;
		}

		sign_and_magnitude_value |= 0x20000;
		printf("sign-and-magnitude: %#x\n", sign_and_magnitude_value);
	}
	else {
		sign_and_magnitude_value = data_value;
	}

	return sign_and_magnitude_value;
}

static void write_bin_file_and_printf (int buffer_id, struct channel *channel_ptr)
{
	unsigned int *buffer = &channel_ptr->buf_ptr[buffer_id].buffer;
    int i;
	unsigned int value;
    FILE *fp;
    fp = fopen("/run/media/mmcblk0p1/output.bin", "wb");
    if (fp == NULL) {
        printf("can't open output.bin\n");
        exit(1);
    }
    for (i = 0; i < test_size / sizeof(unsigned int); i++) {

		if (buffer[i] != 0)
			printf("i: %d; buffer: %#x\n", i, buffer[i]);
		value = count_two_complement(buffer[i]);
        fwrite(&value, sizeof(int), 1, fp);
    }
	printf("last i: %d; buffer: %#x\n", i, buffer[i]);
    fclose(fp);
    printf("save data to output.bin\n");
}

static void rx_request(struct channel *channel_ptr)
{

	int in_progress_count = 0, buffer_id = 0;
	int rx_counter = 0;

	// Start all buffers being received

	buffer_id = channel_ptr->buffer_id;
	channel_ptr->buf_ptr[buffer_id].length = test_size;

	printf("buf_ptr[buffer_id].length %d\n", test_size);

	ioctl(channel_ptr->fd, START_XFER, &buffer_id);
	printf("ioctl START_XFER\n");

	// for (buffer_id = 0; buffer_id < RX_BUFFER_COUNT; buffer_id += BUFFER_INCREMENT) {

	// 	/* Don't worry about initializing the receive buffers as the pattern used in the
	// 	 * transmit buffers is unique across every transfer so it should catch errors.
	// 	 */
	// 	channel_ptr->buf_ptr[buffer_id].length = test_size;

	// 	ioctl(channel_ptr->fd, START_XFER, &buffer_id);

	// 	/* Handle the case of a specified number of transfers that is less than the number
	// 	 * of buffers
	// 	 */
	// 	if (++in_progress_count >= num_transfers)
	// 		break;
	// }

	// buffer_id = 0;

	/* Finish each queued up receive buffer and keep starting the buffer over again
	 * until all the transfers are done
	 */
	while (1) {
		printf("ioctl FINISH_XFER\n");
		ioctl(channel_ptr->fd, FINISH_XFER, &buffer_id);
		printf("ioctl FINISH_XFER end\n");

		printf_dma_status_reg();
		if (channel_ptr->buf_ptr[buffer_id].status != PROXY_NO_ERROR) {
			printf("Proxy rx transfer error, # transfers %d, # completed %d, # in progress %d\n",
						num_transfers, rx_counter, in_progress_count);
			exit(1);
		}

		write_bin_file_and_printf(buffer_id, channel_ptr);
		/* Verify the data received matches what was sent (tx is looped back to tx)
		 * A unique value in the buffers is used across all transfers
		 */
		// if (verify) {

			// unsigned int *buffer = &channel_ptr->buf_ptr[buffer_id].buffer;
			// int i;
			// for (i = 0; i < test_size / sizeof(unsigned int); i++) // test_size / sizeof(unsigned int); i++) this is slow
			// 	printf("i: %d; rx buffer: %#x\n", i, buffer[i]);
				// if (buffer[i] != i + rx_counter) {
				// 	printf("buffer not equal, index = %d, data = %d expected data = %d\n", i,
				// 		buffer[i], i + rx_counter);
				// 	break;
				// }
		// }

		/* Keep track how many transfers are in progress so that only the specified number
		 * of transfers are attempted
		 */
		in_progress_count--;

		/* If all the transfers are done then exit */

		if (++rx_counter >= num_transfers)
			break;

		/* If the ones in progress will complete the number of transfers then don't start more
		 * but finish the ones that are already started
		 */
		if ((rx_counter + in_progress_count) >= num_transfers)
			goto end_rx_loop0;

		/* Start the next buffer again with another transfer keeping track of
		 * the number in progress but not finished
		 */
		ioctl(channel_ptr->fd, START_XFER, &buffer_id);

		in_progress_count++;

	end_rx_loop0:

		/* Flip to next buffer treating them as a circular list, and possibly skipping some
		 * to show the results when prefetching is not happening
		 */
		buffer_id += BUFFER_INCREMENT;
		buffer_id %= RX_BUFFER_COUNT;

	}
}

static int ping_pang_read_dma(int gather_num)
{
	int i = 0;
	open_rx_channel();
	printf("open_rx_channel\n");

    while (i < gather_num || (gather_num==-1)) {
		gpio_set_value(DMA0_GATHER, 1);
		printf("Enable DMA0 gather\n");
		rx_request((void *)&rx_channels[0]);
		printf("send once rx request\n");
		gpio_set_value(DMA0_GATHER, 0);
		printf("Stop DMA0 gather\n");
		i = i+1;
		sleep(3);
	}

	close_rx_channel();
	return 0;
}

/*******************************************************************************************************************/
/*
 * The main program starts the transmit thread and then does the receive processing to do a number of DMA transfers.
 */
int main(int argc, char *argv[])
{
	int i;
	uint64_t start_time, end_time, time_diff;
	int mb_sec;
	int buffer_id = 0;
	int gather_num = -1;
	int max_channel_count = MAX(TX_CHANNEL_COUNT, RX_CHANNEL_COUNT);

	printf("Usage: dma-proxy-test <optional test_size, default ((31 * 1024 * 1024) - 2) * 32> <optional verify, 0 or 1, default 1> <optional dma0 gather number, n or -1(wouldn't stop), default 1>\n");
    g_dma1_map_base = get_map_virtual_addr(PHYSICAL_BASE_ADDRESS_DMA1);
    if (!g_dma1_map_base)
        return -1;
    g_dma2_map_base = get_map_virtual_addr(PHYSICAL_BASE_ADDRESS_DMA2);
    if (!g_dma2_map_base)
        return -1;

	printf_dma_status_reg();

	signal(SIGINT, sigint);

	num_transfers = 1;
	printf("get argc\n");
	// if (argc >= 2){
	// 	test_size = atoi(argv[1]);
	// 	test_size *= 1024;
	// } else {
	// 	test_size = POINT_BYTE;
	// }
	test_size = POINT_BYTE;
	printf("test_size = %d\n", test_size);

	if (argc >= 3){
		verify = atoi(argv[2]);
	} else {
		verify = 1;
	}
	printf("Verify = %d\n", verify);

	if (argc >= 4){
		gather_num = atoi(argv[3]);
	} else {
		gather_num = 1;
	}
	printf("Gather num = %d\n", gather_num);

	gpio_set_value(LVDS_DELAY, 1);
	gpio_set_value(LVDS_DELAY, 0);
	printf("Calibrate the LVDS interface delay\n");

	gpio_set_value(DMA0_EN, 1);
	printf("Enable DMA0\n");

	//listening interrupt and read data
	ping_pang_read_dma(gather_num);

    if (g_dma1_map_base)
        munmap_virtual_addr(g_dma1_map_base);
    if (g_dma2_map_base)
        munmap_virtual_addr(g_dma2_map_base);
	printf("DMA proxy test complete\n");

	return 0;
}