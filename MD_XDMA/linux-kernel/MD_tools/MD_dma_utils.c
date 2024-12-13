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
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>

/* MD:
 * man 2 write:
 * On Linux, write() (and similar system calls) will transfer at most
 * 0x7ffff000 (2,147,479,552) bytes, returning the number of bytes
 * actually transferred. (This is true on both 32-bit and 64-bit systems.)
 */

#define RW_MAX_SIZE 0x7ffff000 // MD: Maximum size for read/write operations

int verbose = 0; // MD: Global variable to control verbosity

/* MD:*
 * getopt_integer() - Parse a string to extract an integer value
 *
 * @param[in] optarg: String containing the integer value
 *
 * This function attempts to parse a string as a hexadecimal or decimal
 * integer and returns the parsed value.
 *
 * @return Parsed integer value
 */
uint64_t getopt_integer(char *optarg)
{
    int rc;
    uint64_t value;

    // MD: Attempt to parse as hexadecimal
    rc = sscanf(optarg, "0x%lx", &value);
    if (rc <= 0)
        // MD: Fallback to parse as decimal
        rc = sscanf(optarg, "%lu", &value);

    // MD: Debug print statement
    if (verbose)
        printf("sscanf() = %d, value = 0x%lx\n", rc, value);

    return value;
}

/* MD:*
 * read_to_buffer() - Read data from a file into a buffer
 *
 * @param[in] fname: Filename for error reporting
 * @param[in] fd: File descriptor to read from
 * @param[out] buffer: Buffer to store the read data
 * @param[in] size: Number of bytes to read
 * @param[in] base: Offset in the file to start reading from
 *
 * This function reads data from a file into a buffer, handling large
 * reads by breaking them into smaller chunks if necessary.
 *
 * @return Number of bytes read on success, -EIO on error
 */
ssize_t read_to_buffer(char *fname, int fd, char *buffer, uint64_t size, uint64_t base)
{
    ssize_t rc;
    uint64_t count = 0;
    char *buf = buffer;
    off_t offset = base;
    int loop = 0;

    while (count < size) {
        uint64_t bytes = size - count;

        // MD: Limit the number of bytes to read in one go
        if (bytes > RW_MAX_SIZE)
            bytes = RW_MAX_SIZE;

        // MD: Seek to the specified offset if necessary
        if (offset) {
            rc = lseek(fd, offset, SEEK_SET);
            if (rc != offset) {
                fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n", fname, rc, offset);
                perror("seek file");
                return -EIO;
            }
        }

        // MD: Read data from file into memory buffer
        rc = read(fd, buf, bytes);
        if (rc < 0) {
            fprintf(stderr, "%s, read 0x%lx @ 0x%lx failed %ld.\n", fname, bytes, offset, rc);
            perror("read file");
            return -EIO;
        }

        count += rc;
        if (rc != bytes) {
            fprintf(stderr, "%s, read underflow 0x%lx/0x%lx @ 0x%lx.\n", fname, rc, bytes, offset);
            break;
        }

        buf += bytes;
        offset += bytes;
        loop++;
    }

    if (count != size && loop)
        fprintf(stderr, "%s, read underflow 0x%lx/0x%lx.\n", fname, count, size);

    return count;
}

/* MD:*
 * write_from_buffer() - Write data from a buffer to a file
 *
 * @param[in] fname: Filename for error reporting
 * @param[in] fd: File descriptor to write to
 * @param[in] buffer: Buffer containing the data to write
 * @param[in] size: Number of bytes to write
 * @param[in] base: Offset in the file to start writing to
 *
 * This function writes data from a buffer to a file, handling large
 * writes by breaking them into smaller chunks if necessary.
 *
 * @return Number of bytes written on success, -EIO on error
 */
ssize_t write_from_buffer(char *fname, int fd, char *buffer, uint64_t size, uint64_t base)
{
    ssize_t rc;
    uint64_t count = 0;
    char *buf = buffer;
    off_t offset = base;
    int loop = 0;

    while (count < size) {
        uint64_t bytes = size - count;

        // MD: Limit the number of bytes to write in one go
        if (bytes > RW_MAX_SIZE)
            bytes = RW_MAX_SIZE;

        // MD: Seek to the specified offset if necessary
        if (offset) {
            rc = lseek(fd, offset, SEEK_SET);
            if (rc != offset) {
                fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n", fname, rc, offset);
                perror("seek file");
                return -EIO;
            }
        }

        // MD: Write data to file from memory buffer
        rc = write(fd, buf, bytes);
        if (rc < 0) {
            fprintf(stderr, "%s, write 0x%lx @ 0x%lx failed %ld.\n", fname, bytes, offset, rc);
            perror("write file");
            return -EIO;
        }

        count += rc;
        if (rc != bytes) {
            fprintf(stderr, "%s, write underflow 0x%lx/0x%lx @ 0x%lx.\n", fname, rc, bytes, offset);
            break;
        }
        buf += bytes;
        offset += bytes;
        loop++;
    }

    if (count != size && loop)
        fprintf(stderr, "%s, write underflow 0x%lx/0x%lx.\n", fname, count, size);

    return count;
}

/* MD:*
 * timespec_check() - Validate a timespec structure
 *
 * @param[in] t: Pointer to the timespec structure
 *
 * This function checks if the timespec structure is normalized, i.e.,
 * 0 <= nsec < 1000000000.
 *
 * @return 0 if valid, -1 if invalid
 */
static int timespec_check(struct timespec *t)
{
    if ((t->tv_nsec < 0) || (t->tv_nsec >= 1000000000))
        return -1;
    return 0;
}

/* MD:*
 * timespec_sub() - Subtract one timespec from another
 *
 * @param[in,out] t1: Pointer to the minuend timespec structure
 * @param[in] t2: Pointer to the subtrahend timespec structure
 *
 * This function subtracts timespec t2 from t1. Both t1 and t2 must be
 * normalized before calling this function.
 */
void timespec_sub(struct timespec *t1, struct timespec *t2)
{
    if (timespec_check(t1) < 0) {
        fprintf(stderr, "invalid time #1: %lld.%.9ld.\n", (long long)t1->tv_sec, t1->tv_nsec);
        return;
    }
    if (timespec_check(t2) < 0) {
        fprintf(stderr, "invalid time #2: %lld.%.9ld.\n", (long long)t2->tv_sec, t2->tv_nsec);
        return;
    }
    t1->tv_sec -= t2->tv_sec;
    t1->tv_nsec -= t2->tv_nsec;
    if (t1->tv_nsec >= 1000000000) {
        t1->tv_sec++;
        t1->tv_nsec -= 1000000000;
    } else if (t1->tv_nsec < 0) {
        t1->tv_sec--;
        t1->tv_nsec += 1000000000;
    }
}
