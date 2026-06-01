//
//  FourBitArray.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/8/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include "FourBitArray.h"
#include <string.h>
#include <cinttypes>

//	uint8_t *mem;
//	uint64_t entries;

FourBitArray::FourBitArray(uint64_t numEntries)
{
	mem = 0;
	Resize(numEntries);
}

FourBitArray::~FourBitArray()
{
	delete [] mem;
}

void FourBitArray::FillMax()
{
	memset(mem, 0xFF, (entries+1)/2);
}

void FourBitArray::Clear()
{
	memset(mem, 0x0, (entries+1)/2);
}

void FourBitArray::Resize(uint64_t newMaxEntries)
{
	entries = newMaxEntries; // 4 bit entries
	newMaxEntries = (newMaxEntries+1)/2; // bytes
	delete [] mem;
	mem = new uint8_t[newMaxEntries];
}

uint64_t FourBitArray::Size() const
{
	return entries;
}

uint8_t FourBitArray::Get(uint64_t index) const
{
	uint8_t val = mem[index/2];
	if (index&1)
	{
		return val>>4;
	}
	return val&0xF;
}

void FourBitArray::Set(uint64_t index, uint8_t val)
{
	if (index&1)
	{
		mem[index/2] = (mem[index/2]&0xF)|(val<<4);
	}
	else {
		mem[index/2] = (mem[index/2]&0xF0)|val;
	}
}

void FourBitArray::Write(const char *file)
{
	FILE *f = fopen(file, "w+");
	fprintf(f, "%" PRId64 "\n", entries);
	fwrite(mem, sizeof(uint8_t), (entries+1)/2, f);
	fclose(f);
}

void FourBitArray::Read(const char *file)
{
	FILE *f = fopen(file, "r");
	fscanf(f, "%" PRId64 "\n", &entries);
	Resize(entries);
	fread(mem, sizeof(uint8_t), (entries+1)/2, f);
	fclose(f);
}
