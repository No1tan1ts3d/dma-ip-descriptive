/* MD:
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present, Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "../xdma/cdev_sgdma.h"

// MD: Structure for XDMA performance IOCTL
struct xdma_performance_ioctl perf;

// MD: Command-line options for the program
static struct option const long_opts[] = {
    {"device", required_argument, NULL, 'd'},
    {"count", required_argument, NULL, 'c'},
    {"size", required_argument, NULL, 's'},
    {"incremental", no_argument, NULL, 'i'},
    {"non-incremental", no_argument, NULL, 'n'},
    {"verbose", no_argument, NULL, 'v'},
    {"help", no_argument, NULL, 'h'},
    {0, 0, 0, 0}
};

// MD: Function to display usage information
static void usage(const char* name)
{
    int i = 0;
    printf("%s\n\n", name);
    printf("usage: %s [OPTIONS]\n\n", name);
    printf("Performance test for XDMA SGDMA engine.\n\n");

    // MD: Iterate over the options and print their descriptions
    printf("  -%c (--%s) device\n", long_opts[i].val, long_opts[i].name); i++;
    printf("  -%c (--%s) incremental\n", long_opts[i].val, long_opts[i].name); i++;
    printf("  -%c (--%s) non-incremental\n", long_opts[i].val, long_opts[i].name); i++;
    printf("  -%c (--%s) be more verbose during test\n", long_opts[i].val, long_opts[i].name); i++;
    printf("  -%c (--%s) print usage help and exit\n", long_opts[i].val, long_opts[i].name); i++;
}

// MD: Function to convert a string to an integer
static uint32_t getopt_integer(char *optarg)
{
    int rc;
    uint32_t value;

    // MD: Attempt to parse the string as a hexadecimal value
    rc = sscanf(optarg, "0x%x", &value);
    if (rc <= 0) {
        // MD: If parsing as hex fails, attempt to parse as a decimal value
        rc = sscanf(optarg, "%ul", &value);
    }

    // MD: Debug print statement to show the result of parsing
    printf("sscanf() = %d, value = 0x%08x\n", rc, (unsigned int)value);

    return value;
}

// MD: Function prototypes
int test_dma(char *device_name, int size, int count);
uint32_t getopt_integer(const char *optarg);
void usage(const char *progname);

// MD: Global verbosity level
static int verbosity = 0;

// MD: Main function to parse command-line arguments and initiate DMA test
int main(int argc, char *argv[]) {
    int cmd_opt;
    char *device = "/dev/xdma/card0/h2c0"; // MD: Default device path
    uint32_t size = 32768; // MD: Default transfer size in bytes
    uint32_t count = 1; // MD: Default count of transfers
    char *filename = NULL; // MD: Placeholder for filename (unused)

    // MD: Parse command-line options
    while ((cmd_opt = getopt_long(argc, argv, "vhic:d:s:", long_opts, NULL)) != -1) {
        switch (cmd_opt) {
            case 0:
                // MD: Long option, already handled
                break;
            case 'v':
                // MD: Increase verbosity level
                verbosity++;
                break;
            case 'd':
                // MD: Set device node name
                printf("Device set to '%s'\n", optarg);
                device = strdup(optarg);
                break;
            case 's':
                // MD: Set transfer size in bytes
                size = getopt_integer(optarg);
                break;
            case 'c':
                // MD: Set count of transfers
                count = getopt_integer(optarg);
                printf("Count set to %d\n", count);
                break;
            case 'h':
            default:
                // MD: Print usage help and exit
                usage(argv[0]);
                exit(0);
                break;
        }
    }

    // MD: Print the configuration before starting the test
    printf("Starting DMA test with device = %s, size = 0x%08x, count = %u\n", device, size, count);

    // MD: Call the test_dma function with the parsed parameters
    test_dma(device, size, count);

    return 0;
}

// MD: Function to perform DMA test
int test_dma(char *device_name, int size, int count) {
    int rc = 0;
    int fd = open(device_name, O_RDWR); // MD: Open the device for read/write
    if (fd < 0) {
        printf("FAILURE: Could not open %s. Ensure xdma device driver is loaded and you have access rights (maybe use sudo?).\n", device_name);
        exit(1);
    }

    // MD: Initialize performance data structure
    perf.version = IOCTL_XDMA_PERF_V1;
    perf.transfer_size = size;

    // MD: Start the DMA performance test
    rc = ioctl(fd, IOCTL_XDMA_PERF_START, &perf);
    if (rc == 0) {
        printf("IOCTL_XDMA_PERF_START successful.\n");
    } else {
        printf("ioctl(..., IOCTL_XDMA_PERF_START) = %d\n", rc);
    }

    // MD: Perform the DMA test for the specified count
    while (count--) {
        sleep(2); // MD: Wait for 2 seconds between iterations

        // MD: Get the performance data
        rc = ioctl(fd, IOCTL_XDMA_PERF_GET, &perf);
        if (rc == 0) {
            printf("IOCTL_XDMA_PERF_GET successful.\n");
        } else {
            printf("ioctl(..., IOCTL_XDMA_PERF_GET) = %d\n", rc);
        }

        // MD: Print performance data
        printf("perf.transfer_size = %d\n", perf.transfer_size);
        printf("perf.iterations = %d\n", perf.iterations);
        printf("(data transferred = %lld bytes)\n", (long long)perf.transfer_size * (long long)perf.iterations);
        printf("perf.clock_cycle_count = %lld\n", (long long)perf.clock_cycle_count);
        printf("perf.data_cycle_count = %lld\n", (long long)perf.data_cycle_count);
        if (perf.clock_cycle_count && perf.data_cycle_count) {
            printf("(data duty cycle = %lld%%)\n", (long long)perf.data_cycle_count * 100 / (long long)perf.clock_cycle_count);
        }
    }

    // MD: Stop the DMA performance test
    rc = ioctl(fd, IOCTL_XDMA_PERF_STOP, &perf);
    if (rc == 0) {
        printf("IOCTL_XDMA_PERF_STOP successful.\n");
    } else {
        printf("ioctl(..., IOCTL_XDMA_PERF_STOP) = %d\n", rc);
    }

    // MD: Print final performance data
    printf("perf.transfer_size = %d bytes\n", perf.transfer_size);
    printf("perf.iterations = %d\n", perf.iterations);
    printf("(data transferred = %lld bytes)\n", (long long)perf.transfer_size * (long long)perf.iterations);
    printf("perf.clock_cycle_count = %lld\n", (long long)perf.clock_cycle_count);
    printf("perf.data_cycle_count = %lld\n", (long long)perf.data_cycle_count);
    if (perf.clock_cycle_count && perf.data_cycle_count) {
        printf("(data duty cycle = %lld%%)\n", (long long)perf.data_cycle_count * 100 / (long long)perf.clock_cycle_count);
        printf("Data rate: bytes length = %d, rate = %f\n", perf.transfer_size, (double)(long long)perf.data_cycle_count / (long long)perf.clock_cycle_count);
    }
    printf("perf.pending_count = %lld\n", (long long)perf.pending_count);

    close(fd); // MD: Close the device
    return 0;
}
