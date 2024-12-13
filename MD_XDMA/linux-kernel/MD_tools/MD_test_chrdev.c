/* MD:
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present, Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

/* MD:*
 * main() - Entry point for the program
 *
 * @param[in]	argc:	Number of command-line arguments
 * @param[in]	argv:	Array of command-line arguments
 *
 * This program opens a device file specified by the user, verifies the
 * operation, and then closes the file. It provides feedback on success
 * or failure of these operations.
 *
 * @return	0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	int fd; // MD: File descriptor for the device file
	char *filename; // MD: Pointer to the device file name

	// MD: Check if the correct number of arguments is provided
	if ((argc < 2) || (argc >= 3)) {
		printf("usage: %s <device file>\n", argv[0]);
		return -1; // MD: Return error if incorrect usage
	}

	filename = argv[1]; // MD: Assign the device file name from arguments

	// MD: Attempt to open the device file with read/write permissions
	fd = open(filename, O_RDWR);
	if (fd < 0) {
		perror("Device open failed"); // MD: Print error if open fails
		return fd; // MD: Return the error code from open
	}
	printf("%s Device open successful\n", argv[1]); // MD: Confirm successful open

	// MD: Attempt to close the device file
	if (close(fd)) {
		perror("Device close failed"); // MD: Print error if close fails
		return -1; // MD: Return error if close fails
	}
	printf("%s Device close successful\n", argv[1]); // MD: Confirm successful close

	return 0; // MD: Return success
}
