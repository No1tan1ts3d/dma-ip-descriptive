/* MD:
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
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
#include <errno.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "../xdma/cdev_sgdma.h"

#include "dma_utils.c"

//MD: Command-line options for the program
static struct option const long_opts[] = {
    {"device", required_argument, NULL, 'd'},
    {"address", required_argument, NULL, 'a'},
    {"aperture", required_argument, NULL, 'k'},
    {"size", required_argument, NULL, 's'},
    {"offset", required_argument, NULL, 'o'},
    {"count", required_argument, NULL, 'c'},
    {"data infile", required_argument, NULL, 'f'},
    {"data outfile", required_argument, NULL, 'w'},
    {"help", no_argument, NULL, 'h'},
    {"verbose", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
};

//MD: Default device name and transfer parameters
#define DEVICE_NAME_DEFAULT "/dev/xdma0_h2c_0"
#define SIZE_DEFAULT (32)
#define COUNT_DEFAULT (1)

//MD: Function prototype for DMA test
static int test_dma(char *devname, uint64_t addr, uint64_t aperture,
                    uint64_t size, uint64_t offset, uint64_t count,
                    char *filename, char *);

//MD: Function to display usage information
static void usage(const char *name)
{
    int i = 0;

    fprintf(stdout, "%s\n\n", name);
    fprintf(stdout, "usage: %s [OPTIONS]\n\n", name);
    fprintf(stdout, 
        "Write via SGDMA, optionally read input from a file.\n\n");

    //MD: Iterate over the options and print their descriptions
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
        "  -%c (--%s) size of a single transfer in bytes, default %d,\n",
        long_opts[i].val, long_opts[i].name, SIZE_DEFAULT);
    i++;
    fprintf(stdout, "  -%c (--%s) page offset of transfer\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) number of transfers, default %d\n",
        long_opts[i].val, long_opts[i].name, COUNT_DEFAULT);
    i++;
    fprintf(stdout, "  -%c (--%s) filename to read the data from.\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout,
        "  -%c (--%s) filename to write the data of the transfers\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) print usage help and exit\n",
        long_opts[i].val, long_opts[i].name);
    i++;
    fprintf(stdout, "  -%c (--%s) verbose output\n",
        long_opts[i].val, long_opts[i].name);
    i++;

    //MD: Return code information
    fprintf(stdout, "\nReturn code:\n");
    fprintf(stdout, "  0: all bytes were dma'ed successfully\n");
    fprintf(stdout, "  < 0: error\n\n");
}

int main(int argc, char *argv[])
{
    int cmd_opt;
    char *device = DEVICE_NAME_DEFAULT; //MD: Default device name
    uint64_t address = 0; //MD: RAM address on the AXI bus
    uint64_t aperture = 0; //MD: Memory aperture window size
    uint64_t size = SIZE_DEFAULT; //MD: Default size in bytes
    uint64_t offset = 0; //MD: Offset value
    uint64_t count = COUNT_DEFAULT; //MD: Default count
    char *infname = NULL; //MD: Input file name
    char *ofname = NULL; //MD: Output file name

    //MD: Parse command-line options
    while ((cmd_opt = getopt_long(argc, argv, "vhc:f:d:a:k:s:o:w:", long_opts, NULL)) != -1) {
        switch (cmd_opt) {
            case 0:
                //MD: Long option, already handled
                break;
            case 'd':
                //MD: Device node name
                device = strdup(optarg);
                break;
            case 'a':
                //MD: RAM address on the AXI bus in bytes
                address = getopt_integer(optarg);
                break;
            case 'k':
                //MD: Memory aperture window size
                aperture = getopt_integer(optarg);
                break;
            case 's':
                //MD: Size in bytes
                size = getopt_integer(optarg);
                break;
            case 'o':
                //MD: Offset value, aligned to 4096
                offset = getopt_integer(optarg) & 4095;
                break;
            case 'c':
                //MD: Count
                count = getopt_integer(optarg);
                break;
            case 'f':
                //MD: Input file name
                infname = strdup(optarg);
                break;
            case 'w':
                //MD: Output file name
                ofname = strdup(optarg);
                break;
            case 'v':
                //MD: Enable verbose output
                verbose = 1;
                break;
            case 'h':
            default:
                //MD: Print usage help and exit
                usage(argv[0]);
                exit(0);
                break;
        }
    }

    //MD: Print verbose output if enabled
    if (verbose) {
        fprintf(stdout, 
            "dev %s, addr 0x%lx, aperture 0x%lx, size 0x%lx, offset 0x%lx, count %lu\n",
            device, address, aperture, size, offset, count);
    }

    //MD: Call the test_dma function with the parsed parameters
    return test_dma(device, address, aperture, size, offset, count, infname, ofname);
}

/* MD:****************************************************************************/
/* MD:*
 * test_dma() - Perform DMA operations between host and FPGA device
 *
 * @param[in]	devname:	Device name for the FPGA
 * @param[in]	addr:		Address on the AXI bus
 * @param[in]	aperture:	Aperture size for the operation
 * @param[in]	size:		Size of the data to transfer
 * @param[in]	offset:		Offset for the buffer alignment
 * @param[in]	count:		Number of iterations for the DMA operation
 * @param[in]	infname:	Input file name (optional)
 * @param[in]	ofname:	Output file name (optional)
 *
 * This function performs DMA operations by writing data from a buffer to an
 * FPGA device. It supports reading from an input file and writing to an
 * output file. It calculates the average bandwidth and handles errors.
 *
 * @return	0 on success
 * @return	-EIO on underflow error
 * @return	<0 on other failures
 *****************************************************************************/
static int test_dma(char *devname, uint64_t addr, uint64_t aperture,
		    uint64_t size, uint64_t offset, uint64_t count,
		    char *infname, char *ofname)
{
	uint64_t i;
	ssize_t rc;
	size_t bytes_done = 0;
	size_t out_offset = 0;
	char *buffer = NULL;
	char *allocated = NULL;
	struct timespec ts_start, ts_end;
	int infile_fd = -1;
	int outfile_fd = -1;
	int fpga_fd = open(devname, O_RDWR);
	long total_time = 0;
	float result;
	float avg_time = 0;
	int underflow = 0;

	//MD: Open the FPGA device
	if (fpga_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\n", devname, fpga_fd);
		perror("open device");
		return -EINVAL;
	}

	//MD: Open the input file if specified
	if (infname) {
		infile_fd = open(infname, O_RDONLY);
		if (infile_fd < 0) {
			fprintf(stderr, "unable to open input file %s, %d.\n", infname, infile_fd);
			perror("open input file");
			rc = -EINVAL;
			goto out;
		}
	}

	//MD: Open the output file if specified
	if (ofname) {
		outfile_fd = open(ofname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC, 0666);
		if (outfile_fd < 0) {
			fprintf(stderr, "unable to open output file %s, %d.\n", ofname, outfile_fd);
			perror("open output file");
			rc = -EINVAL;
			goto out;
		}
	}

	//MD: Allocate aligned memory for the buffer
	posix_memalign((void **)&allocated, 4096 /* MD:alignment*/, size + 4096);
	if (!allocated) {
		fprintf(stderr, "OOM %lu.\n", size + 4096);
		rc = -ENOMEM;
		goto out;
	}
	buffer = allocated + offset;
	if (verbose)
		fprintf(stdout, "host buffer 0x%lx = %p\n", size + 4096, buffer);

	//MD: Read data from the input file into the buffer
	if (infile_fd >= 0) {
		rc = read_to_buffer(infname, infile_fd, buffer, size, 0);
		if (rc < 0 || rc < size)
			goto out;
	}

	//MD: Perform DMA operations for the specified count
	for (i = 0; i < count; i++) {
		//MD: Record the start time
		rc = clock_gettime(CLOCK_MONOTONIC, &ts_start);

		if (aperture) {
			//MD: Use ioctl for aperture-based DMA
			struct xdma_aperture_ioctl io;
			io.buffer = (unsigned long)buffer;
			io.len = size;
			io.ep_addr = addr;
			io.aperture = aperture;
			io.done = 0UL;

			rc = ioctl(fpga_fd, IOCTL_XDMA_APERTURE_W, &io);
			if (rc < 0 || io.error) {
				fprintf(stdout, "#%d: aperture W ioctl failed %d,%d.\n", i, rc, io.error);
				goto out;
			}

			bytes_done = io.done;
		} else {
			//MD: Use write for non-aperture DMA
			rc = write_from_buffer(devname, fpga_fd, buffer, size, addr);
			if (rc < 0)
				goto out;

			bytes_done = rc;
		}

		//MD: Record the end time
		rc = clock_gettime(CLOCK_MONOTONIC, &ts_end);

		//MD: Check for underflow
		if (bytes_done < size) {
			printf("#%d: underflow %ld/%ld.\n", i, bytes_done, size);
			underflow = 1;
		}

		//MD: Calculate the time taken for the operation
		timespec_sub(&ts_end, &ts_start);
		total_time += ts_end.tv_nsec;

		//MD: Log the time taken for each iteration if verbose
		if (verbose)
			fprintf(stdout, "#%lu: CLOCK_MONOTONIC %ld.%09ld sec. write %ld bytes\n", i, ts_end.tv_sec, ts_end.tv_nsec, size);

		//MD: Write data to the output file if specified
		if (outfile_fd >= 0) {
			rc = write_from_buffer(ofname, outfile_fd, buffer, bytes_done, out_offset);
			if (rc < 0 || rc < bytes_done)
				goto out;
			out_offset += bytes_done;
		}
	}

	//MD: Calculate and print average bandwidth if no underflow occurred
	if (!underflow) {
		avg_time = (float)total_time / (float)count;
		result = ((float)size) * 1000 / avg_time;
		if (verbose)
			printf("** Avg time device %s, total time %ld nsec, avg_time = %f, size = %lu, BW = %f \n", devname, total_time, avg_time, size, result);
		printf("%s ** Average BW = %lu, %f\n", devname, size, result);
	}

out:
	//MD: Clean up resources
	close(fpga_fd);
	if (infile_fd >= 0)
		close(infile_fd);
	if (outfile_fd >= 0)
		close(outfile_fd);
	free(allocated);

	//MD: Return error code if any operation failed
	if (rc < 0)
		return rc;

	//MD: Treat underflow as an error
	return underflow ? -EIO : 0;
}
