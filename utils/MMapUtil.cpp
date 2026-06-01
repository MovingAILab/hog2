//
//  MMapUtil.c
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cinttypes>

#ifndef MAP_ANONYMOUS
#define MAP_ANONYMOUS MAP_ANON
#endif

#define handle_error(msg) \
do { perror(msg); exit(EXIT_FAILURE); } while (0)

uint8_t *GetMMAP(const char *filename, uint64_t mapSize, int &fd, bool zero)
{
	uint8_t *memblock;
	
	if (filename == 0)
	{
		memblock = (uint8_t *)mmap(NULL, mapSize, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_ANONYMOUS, -1, 0);
		if (memblock == MAP_FAILED)
		{
			handle_error("mmap");
		}
		return memblock;
	}
	
	//int fd;
	struct stat sb;
	
	printf("Size of off_t is (%lu), uint64_t is (%lu)\n", sizeof(off_t), sizeof(uint64_t));
	if (zero)
	{
		if ((fd = open(filename, O_RDWR|O_CREAT|O_TRUNC, 0666)) == -1)
		{
			handle_error("open");
		}
		if (lseek(fd, mapSize-1, SEEK_SET) == -1)
		{
			handle_error("lseek");
		}
		if (write(fd, "", 1) != 1)
		{
			handle_error("write");
		}

		fstat(fd, &sb);
		printf("Size: %" PRId64 " \n",(uint64_t)sb.st_size);
		assert(sb.st_size == mapSize);
	}
	else {
		if ((fd = open(filename, O_RDWR, 0666)) == -1)
		{
			handle_error("open");
		}
	}
	
	fstat(fd, &sb);
	printf("Size: %" PRId64 " \n",(uint64_t)sb.st_size);
	assert(sb.st_size >= mapSize);
	
	memblock = (uint8_t *)mmap(NULL, sb.st_size, PROT_WRITE|PROT_READ, /*MAP_PRIVATE|MAP_POPULATE*/MAP_SHARED, fd, 0);
	//madvise(memblock, sb.st_size, MADV_RANDOM|MADV_WILLNEED);
	if (memblock == MAP_FAILED)
	{
		handle_error("mmap");
	}
	return memblock;
}

void CloseMMap(uint8_t *mem, uint64_t mapSizeBytes, int fd)
{
	if (munmap(mem, mapSizeBytes) != 0)
	{
		handle_error("unmap");
	}
	if (close(fd) == -1)
	{
		handle_error("close");
	}
}
