/* MD:
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present, Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <byteswap.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>

#include <sys/types.h>
#include <sys/mman.h>

/* MD: ltoh: little endian to host */
/* MD: htol: host to little endian */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define ltohl(x)       (x)
#define ltohs(x)       (x)
#define htoll(x)       (x)
#define htols(x)       (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define ltohl(x)     __bswap_32(x)
#define ltohs(x)     __bswap_16(x)
#define htoll(x)     __bswap_32(x)
#define htols(x)     __bswap_16(x)
#endif

int main(int argc, char **argv)
{
    int fd; // MD: File descriptor for the device
    int err = 0; // MD: Error flag
    void *map; // MD: Pointer for memory mapping
    uint32_t read_result, writeval; // MD: Variables for read and write operations
    off_t target; // MD: Target memory address
    off_t pgsz, target_aligned, offset; // MD: Variables for page size and alignment
    char access_width = 'w'; // MD: Default access width is word (32-bits)
    char *device; // MD: Device name

    // MD: Check if enough arguments are provided
    if (argc < 3) {
        fprintf(stderr,
            "\nUsage:\t%s <device> <address> [[type] data]\n"
            "\tdevice  : character device to access\n"
            "\taddress : memory address to access\n"
            "\ttype    : access operation type : [b]yte, [h]alfword, [w]ord\n"
            "\tdata    : data to be written for a write\n\n",
            argv[0]);
        exit(1);
    }

    device = strdup(argv[1]); // MD: Duplicate device name
    target = strtoul(argv[2], 0, 0); // MD: Convert address argument to unsigned long

    // MD: Check for target page alignment
    pgsz = sysconf(_SC_PAGESIZE); // MD: Get system page size
    offset = target & (pgsz - 1); // MD: Calculate offset within the page
    target_aligned = target & (~(pgsz - 1)); // MD: Align target to page boundary

    printf("device: %s, address: 0x%lx (0x%lx+0x%lx), access %s.\n",
        device, target, target_aligned, offset,
        argc >= 4 ? "write" : "read");

    // MD: Determine access width if specified
    if (argc >= 4)
        access_width = tolower(argv[3][0]);
    printf("access width: ");
    if (access_width == 'b')
        printf("byte (8-bits)\n");
    else if (access_width == 'h')
        printf("half word (16-bits)\n");
    else if (access_width == 'w')
        printf("word (32-bits)\n");
    else {
        printf("default to word (32-bits)\n");
        access_width = 'w';
    }

    // MD: Open the device file
    if ((fd = open(argv[1], O_RDWR | O_SYNC)) == -1) {
        printf("character device %s opened failed: %s.\n",
            argv[1], strerror(errno));
        return -errno;
    }
    printf("character device %s opened.\n", argv[1]);

    // MD: Map the memory
    map = mmap(NULL, offset + 4, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
               target_aligned);
    if (map == (void *)-1) {
        printf("Memory 0x%lx mapped failed: %s.\n",
            target, strerror(errno));
        err = 1;
        goto close;
    }
    printf("Memory 0x%lx mapped at address %p.\n", target_aligned, map);

    map += offset; // MD: Adjust map pointer by offset

    // MD: Read operation
    if (argc <= 4) {
        switch (access_width) {
        case 'b':
            read_result = *((uint8_t *) map);
            printf("Read 8-bits value at address 0x%lx (%p): 0x%02x\n",
                   target, map, (unsigned int)read_result);
            break;
        case 'h':
            read_result = *((uint16_t *) map);
            read_result = ltohs(read_result); // MD: Swap endianess if necessary
            printf("Read 16-bit value at address 0x%lx (%p): 0x%04x\n",
                   target, map, (unsigned int)read_result);
            break;
        case 'w':
            read_result = *((uint32_t *) map);
            read_result = ltohl(read_result); // MD: Swap endianess if necessary
            printf("Read 32-bit value at address 0x%lx (%p): 0x%08x\n",
                   target, map, (unsigned int)read_result);
            break;
        default:
            fprintf(stderr, "Illegal data type '%c'.\n", access_width);
            err = 1;
            goto unmap;
        }
    }

    // MD: Write operation
    if (argc >= 5) {
        writeval = strtoul(argv[4], 0, 0); // MD: Convert data argument to unsigned long
        switch (access_width) {
        case 'b':
            printf("Write 8-bits value 0x%02x to 0x%lx (0x%p)\n",
                   (unsigned int)writeval, target, map);
            *((uint8_t *) map) = writeval;
            break;
        case 'h':
            printf("Write 16-bits value 0x%04x to 0x%lx (0x%p)\n",
                   (unsigned int)writeval, target, map);
            writeval = htols(writeval); // MD: Swap endianess if necessary
            *((uint16_t *) map) = writeval;
            break;
        case 'w':
            printf("Write 32-bits value 0x%08x to 0x%lx (0x%p)\n",
                   (unsigned int)writeval, target, map);
            writeval = htoll(writeval); // MD: Swap endianess if necessary
            *((uint32_t *) map) = writeval;
            break;
        default:
            fprintf(stderr, "Illegal data type '%c'.\n", access_width);
            err = 1;
            goto unmap;
        }
    }

unmap:
    map -= offset; // MD: Reset map pointer to original
    if (munmap(map, offset + 4) == -1) {
        printf("Memory 0x%lx mapped failed: %s.\n",
            target, strerror(errno));
    }

close:
    close(fd); // MD: Close the device file

    return err; // MD: Return error status
}
