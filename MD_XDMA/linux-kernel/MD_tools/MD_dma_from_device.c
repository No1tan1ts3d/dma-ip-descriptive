/* MD:
 * This file is part of the Xilinx DMA IP Core driver tool for Linux
 *
 * Copyright (c) 2016-present, Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "../xdma/cdev_sgdma.h"

#include "dma_utils.c"

// MD: Default device name and transfer parameters
#define DEVICE_NAME_DEFAULT "/dev/xdma0_c2h_0"
#define SIZE_DEFAULT (32)
#define COUNT_DEFAULT (1)

// MD: Command-line options for the program
static struct option const long_opts[] = {
    {"device", required_argument, NULL, 'd'},
    {"address", required_argument, NULL, 'a'},
    {"aperture", required_argument, NULL, 'k'},
    {"size", required_argument, NULL, 's'},
    {"offset", required_argument, NULL, 'o'},
    {"count", required_argument, NULL, 'c'},
    {"file", required_argument, NULL, 'f'},
    {"eop_flush", no_argument, NULL, 'e'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
};

// MD: Function prototypes
static int test_dma(char *devname, uint64_t addr, uint64_t aperture, 
        uint64_t size, uint64_t offset, uint64_t count,
        char *ofname);
static int eop_flush = 0;

// MD: Function to display usage information
static void usage(const char *name)
{
    int i = 0;
    fprintf(stdout, "%s\n\n", name);
    fprintf(stdout, "usage: %s [OPTIONS]\n\n", name);
    fprintf(stdout, "Read via SGDMA, optionally save output to a file\n\n");

    // MD: Iterate over the options and print their descriptions
    fprintf(stdout, "  -%c (--%s) device (defaults to %s)\n",
        long_opts[i].val, long_opts[i].name, DEVICE_NAME_DEFAULT);
    i++;
    fprintf(stdout, "  -%c (--%s) the start address on the AXI bus\n",
           long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) memory address aperture\n",
           long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout,
        "  -%c (--%s) size of a single transfer in bytes, default %d.\n",
        long_opts[i].val, long_opts[i].name, SIZE_DEFAULT);
    i++;
    fprintf(stdout, "  -%c (--%s) page offset of transfer\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) number of transfers, default is %d.\n",
           long_opts[i].val, long_opts[i].name, COUNT_DEFAULT);
    i++;
    fprintf(stdout,
        "  -%c (--%s) file to write the data of the transfers\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout,
         "  -%c (--%s) end dma when ST end-of-packet(eop) is rcved\n",
        long_opts[i].val, long_opts[i].name);
    fprintf(stdout,
         "\t\t* streaming only, ignored for memory-mapped channels\n");
    fprintf(stdout,
         "\t\t* actual # of bytes dma'ed could be smaller than specified\n");
    i++;
    fprintf(stdout, "  -%c (--%s) print usage help and exit\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) verbose output\n",
        long_opts[i].val, long_opts[i].name);
    i++;

    // MD: Return code information
    fprintf(stdout, "\nReturn code:\n");
    fprintf(stdout, "  0: all bytes were dma'ed successfully\n");
    fprintf(stdout, "     * with -e set, the bytes dma'ed could be smaller\n");
    fprintf(stdout, "  < 0: error\n\n");
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <stdint.h>
#include <unistd.h>

// MD: Assuming these are defined elsewhere in your code
#define DEVICE_NAME_DEFAULT "/dev/xdma0_c2h_0"
#define SIZE_DEFAULT (32)
#define COUNT_DEFAULT (1)

// MD: External variables and functions
extern int verbose;
extern int eop_flush;
extern struct option long_opts[];
extern void usage(const char *name);
extern int test_dma(char *devname, uint64_t addr, uint64_t aperture, uint64_t size, uint64_t offset, uint64_t count, char *ofname);
extern uint64_t getopt_integer(const char *optarg);

int main(int argc, char *argv[])
{
    int cmd_opt; // MD: Variable to store command line option
    char *device = DEVICE_NAME_DEFAULT; // MD: Default device name
    uint64_t address = 0; // MD: RAM address on the AXI bus
    uint64_t aperture = 0; // MD: Memory aperture window size
    uint64_t size = SIZE_DEFAULT; // MD: RAM size in bytes
    uint64_t offset = 0; // MD: Offset for the transfer
    uint64_t count = COUNT_DEFAULT; // MD: Number of transfers
    char *ofname = NULL; // MD: Output file name

    // MD: Parse command line options
    while ((cmd_opt = getopt_long(argc, argv, "vhec:f:d:a:k:s:o:", long_opts, NULL)) != -1) {
        switch (cmd_opt) {
            case 0:
                // MD: Long option, no action needed here
                break;
            case 'd':
                // MD: Device node name
                device = strdup(optarg);
                break;
            case 'a':
                // MD: RAM address on the AXI bus in bytes
                address = getopt_integer(optarg);
                break;
            case 'k':
                // MD: Memory aperture window size
                aperture = getopt_integer(optarg);
                break;
            case 's':
                // MD: RAM size in bytes
                size = getopt_integer(optarg);
                break;
            case 'o':
                // MD: Offset for the transfer, masked with 4095
                offset = getopt_integer(optarg) & 4095;
                break;
            case 'c':
                // MD: Number of transfers
                count = getopt_integer(optarg);
                break;
            case 'f':
                // MD: Output file name
                ofname = strdup(optarg);
                break;
            case 'v':
                // MD: Enable verbose output
                verbose = 1;
                break;
            case 'e':
                // MD: Enable end-of-packet flush
                eop_flush = 1;
                break;
            case 'h':
            default:
                // MD: Print usage help and exit
                usage(argv[0]);
                exit(0);
                break;
        }
    }

    // MD: Print verbose output if enabled
    if (verbose) {
        fprintf(stdout,
            "Device: %s, Address: 0x%lx, Aperture: 0x%lx, Size: 0x%lx, Offset: 0x%lx, Count: %lu\n",
            device, address, aperture, size, offset, count);
    }

    // MD: Call the test_dma function with the parsed parameters
    return test_dma(device, address, aperture, size, offset, count, ofname);
}

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>

// MD: Assuming these are defined elsewhere
#define IOCTL_XDMA_APERTURE_R 0x1234 // MD: Placeholder value
#define EOP_FLUSH 1 // MD: Placeholder value for eop_flush
#define VERBOSE 1 // MD: Placeholder value for verbose

struct xdma_aperture_ioctl {
    unsigned long buffer;
    size_t len;
    uint64_t ep_addr;
    uint64_t aperture;
    unsigned long done;
    int error;
};

// MD: Placeholder functions
ssize_t read_to_buffer(char *devname, int fpga_fd, char *buffer, size_t size, uint64_t addr) {
    // MD: Implementation goes here
    return size; // MD: Placeholder return
}

ssize_t write_from_buffer(char *ofname, int out_fd, char *buffer, size_t bytes_done, size_t out_offset) {
    // MD: Implementation goes here
    return bytes_done; // MD: Placeholder return
}

void timespec_sub(struct timespec *end, struct timespec *start) {
    // MD: Subtract start from end
    if ((end->tv_nsec - start->tv_nsec) < 0) {
        end->tv_sec = end->tv_sec - start->tv_sec - 1;
        end->tv_nsec = end->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        end->tv_sec = end->tv_sec - start->tv_sec;
        end->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
}

static int test_dma(char *devname, uint64_t addr, uint64_t aperture,
                    uint64_t size, uint64_t offset, uint64_t count,
                    char *ofname) {
    ssize_t rc = 0;
    size_t out_offset = 0;
    size_t bytes_done = 0;
    uint64_t i;
    char *buffer = NULL;
    char *allocated = NULL;
    struct timespec ts_start, ts_end;
    int out_fd = -1;
    int fpga_fd;
    long total_time = 0;
    float result;
    float avg_time = 0;
    int underflow = 0;

    // MD: Open the device with or without truncation based on eop_flush
    if (EOP_FLUSH)
        fpga_fd = open(devname, O_RDWR | O_TRUNC);
    else
        fpga_fd = open(devname, O_RDWR);

    if (fpga_fd < 0) {
        fprintf(stderr, "unable to open device %s, %d.\n", devname, fpga_fd);
        perror("open device");
        return -EINVAL;
    }

    // MD: Create file to write data to if ofname is provided
    if (ofname) {
        out_fd = open(ofname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
        if (out_fd < 0) {
            fprintf(stderr, "unable to open output file %s, %d.\n", ofname, out_fd);
            perror("open output file");
            rc = -EINVAL;
            goto out;
        }
    }

    // MD: Allocate aligned memory for DMA operations
    posix_memalign((void **)&allocated, 4096, size + 4096);
    if (!allocated) {
        fprintf(stderr, "OOM %lu.\n", size + 4096);
        rc = -ENOMEM;
        goto out;
    }

    buffer = allocated + offset;
    if (VERBOSE)
        fprintf(stdout, "host buffer 0x%lx, %p.\n", size + 4096, buffer);

    // MD: Perform DMA operations for the specified count
    for (i = 0; i < count; i++) {
        rc = clock_gettime(CLOCK_MONOTONIC, &ts_start);
        if (aperture) {
            struct xdma_aperture_ioctl io;
            io.buffer = (unsigned long)buffer;
            io.len = size;
            io.ep_addr = addr;
            io.aperture = aperture;
            io.done = 0UL;

            rc = ioctl(fpga_fd, IOCTL_XDMA_APERTURE_R, &io);
            if (rc < 0 || io.error) {
                fprintf(stderr, "#%d: aperture R failed %d,%d.\n", i, rc, io.error);
                goto out;
            }

            bytes_done = io.done;
        } else {
            rc = read_to_buffer(devname, fpga_fd, buffer, size, addr);
            if (rc < 0)
                goto out;
            bytes_done = rc;
        }
        clock_gettime(CLOCK_MONOTONIC, &ts_end);

        if (bytes_done < size) {
            fprintf(stderr, "#%d: underflow %ld/%ld.\n", i, bytes_done, size);
            underflow = 1;
        }

        // MD: Calculate the elapsed time
        timespec_sub(&ts_end, &ts_start);
        total_time += ts_end.tv_nsec;
        if (VERBOSE)
            fprintf(stdout, "#%lu: CLOCK_MONOTONIC %ld.%09ld sec. read %ld/%ld bytes\n",
                    i, ts_end.tv_sec, ts_end.tv_nsec, bytes_done, size);

        // MD: Write data to output file if specified
        if (out_fd >= 0) {
            rc = write_from_buffer(ofname, out_fd, buffer, bytes_done, out_offset);
            if (rc < 0 || rc < bytes_done)
                goto out;
            out_offset += bytes_done;
        }
    }

    // MD: Calculate average time and bandwidth if no underflow occurred
    if (!underflow) {
        avg_time = (float)total_time / (float)count;
        result = ((float)size) * 1000 / avg_time;
        if (VERBOSE)
            printf("** Avg time device %s, total time %ld nsec, avg_time = %f, size = %lu, BW = %f \n",
                   devname, total_time, avg_time, size, result);
        printf("%s ** Average BW = %lu, %f\n", devname, size, result);
        rc = 0;
    } else if (EOP_FLUSH) {
        // MD: Allow underflow with -e option
        rc = 0;
    } else {
        rc = -EIO;
    }

out:
    close(fpga_fd);
    if (out_fd >= 0)
        close(out_fd);
    free(allocated);

    return rc;
}
