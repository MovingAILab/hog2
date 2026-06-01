//
//  NBitArray.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_NBitArray_h
#define hog2_glut_NBitArray_h

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <cinttypes>

/**
 * This class supports compact n-bit arrays. For (1 <= n <= 64). 
 * It is efficient for powers of two, but less so
 * for non-powers of two. (Currently about 3x slower.)
 */
template <uint64_t numBits>
class NBitArray
{
public:
	NBitArray(uint64_t numEntries = 0);
	NBitArray(const char *);
	NBitArray(const NBitArray &copyMe);
	~NBitArray();
	NBitArray &operator=(const NBitArray &copyMe);
	bool Equal(const NBitArray &compare) const;
	void FillMax();
	void Clear();
	void Resize(uint64_t newMaxEntries);
	uint64_t Size() const;
	uint64_t Get(uint64_t index) const;
	void Set(uint64_t index, uint64_t val);
	uint64_t GetMaxValue() const { uint64_t v = 1; return (v<<numBits)-1;}
	
	bool Write(FILE *);
	bool Read(FILE *);
	bool Write(const char *);
	bool Read(const char *);
private:
	uint64_t *mem;
	uint64_t entries;
	uint64_t memorySize;
};


template <uint64_t numBits>
bool operator==(const NBitArray<numBits>& l, const NBitArray<numBits>& r)
{
	return l.Equal(r);
}

template <uint64_t numBits>
NBitArray<numBits>::NBitArray(uint64_t numEntries)
:entries(numEntries), memorySize(((entries*numBits+63)/64))
{
	static_assert(numBits >= 1 && numBits <= 64, "numBits out of bounds!");

	mem = new uint64_t[memorySize];
}

template <uint64_t numBits>
NBitArray<numBits>::NBitArray(const char *file)
:mem(0)
{
	static_assert(numBits >= 1 && numBits <= 64, "numBits out of bounds!");
	Read(file);
}

template <uint64_t numBits>
NBitArray<numBits>::NBitArray(const NBitArray &copyMe)
{
	entries = copyMe.entries;
	memorySize = copyMe.memorySize;
	mem = new uint64_t[memorySize];
	memcpy(mem, copyMe.mem, memorySize*sizeof(mem[0]));
}


template <uint64_t numBits>
NBitArray<numBits>::~NBitArray()
{
	delete [] mem;
}

template <uint64_t numBits>
NBitArray<numBits> &NBitArray<numBits>::operator=(const NBitArray &copyMe)
{
	if (this == &copyMe)
		return *this;
	delete mem;
	entries = copyMe.entries;
	memorySize = copyMe.memorySize;
	mem = new uint64_t[memorySize];
	memcpy(mem, copyMe.mem, memorySize*sizeof(mem[0]));
	return *this;
}

template <uint64_t numBits>
bool NBitArray<numBits>::Equal(const NBitArray &compare) const
{
	if (this == &compare)
		return true;
	if (entries != compare.entries)
		return false;
	for (size_t x = 0; x < memorySize; x++)
	{
		if (mem[x] != compare.mem[x])
			return false;
	}
#pragma message("Last bits are not being tested properly")
	// TODO: figure out how many remaining entries are left, and
	// only compare relevant bits
	return true;
}


template <uint64_t numBits>
void NBitArray<numBits>::FillMax()
{
	memset(mem, 0xFF, memorySize*8);
}

template <uint64_t numBits>
void NBitArray<numBits>::Clear()
{
	memset(mem, 0, memorySize*8);
}

template <uint64_t numBits>
void NBitArray<numBits>::Resize(uint64_t newMaxEntries)
{
	entries = newMaxEntries;
	memorySize = ((entries*numBits+63)/64);
	delete [] mem;
	mem = new uint64_t[memorySize];
}

template <uint64_t numBits>
uint64_t NBitArray<numBits>::Size() const
{
	return entries;
}

template <uint64_t numBits>
bool NBitArray<numBits>::Write(FILE *f)
{
	if (fwrite(&entries, sizeof(uint64_t), 1, f) != 1)
		return false;
	if (fwrite(&memorySize, sizeof(uint64_t), 1, f) != 1)
		return false;
	if (fwrite(mem, sizeof(uint64_t), memorySize, f) != memorySize)
		return false;
	printf("Wrote %" PRId64 " bytes to disk\n", memorySize*sizeof(uint64_t));
	return true;
}

template <uint64_t numBits>
bool NBitArray<numBits>::Read(FILE *f)
{
	bool success = true;
	uint64_t e1, m1;
	success = success&&(fread(&e1, sizeof(uint64_t), 1, f) == 1);
	success = success&&(fread(&m1, sizeof(uint64_t), 1, f) == 1);
	if (success)
	{
		entries = e1;
		memorySize = m1;
		delete [] mem;
		mem = new uint64_t[memorySize];
		success = success&&(fread(mem, sizeof(uint64_t), memorySize, f) == memorySize);
	}
	return success;
}

template <uint64_t numBits>
bool NBitArray<numBits>::Write(const char *file)
{
	FILE *f = fopen(file, "w+b");
	if (f == 0)
	{
		perror("Could not open file for writing in NBitArray");
		return false;
	}
	bool result = Write(f);
	fclose(f);
	return result;
}

template <uint64_t numBits>
bool NBitArray<numBits>::Read(const char *file)
{
	FILE *f = fopen(file, "rb");
	if (f == 0)
	{
		perror("Could not open file for reading in NBitArray");
		return false;
	}
	bool result = Read(f);
	fclose(f);
	return result;
}

template <uint64_t numBits>
uint64_t NBitArray<numBits>::Get(uint64_t index) const
{
	uint64_t startingBit = index*numBits;
	uint64_t offset1 = startingBit/64;
	uint64_t bitOffset1 = startingBit&0x3F; // same as mod 64
	uint64_t bitCount1 = std::min(64-bitOffset1, numBits);
	uint64_t bitCount2 = numBits - bitCount1;
	uint64_t bitMask1 = (1ull<<bitCount1)-1;
	uint64_t bitMask2 = (1ull<<bitCount2)-1;
	uint64_t result = (mem[offset1]>>bitOffset1)&bitMask1;
	result = ((mem[offset1+1]&bitMask2)<<bitCount1) | result;
	return result;
}

template <uint64_t numBits>
void NBitArray<numBits>::Set(uint64_t index, uint64_t val)
{
	uint64_t startingBit = index*numBits;
	uint64_t offset1 = startingBit/64;
	uint64_t bitOffset1 = startingBit&0x3F; // same as mod 64
	uint64_t bitCount1 = std::min(64-bitOffset1, (uint64_t)numBits);
	uint64_t bitCount2 = numBits - bitCount1;
	uint64_t bitMask1 = (1ull<<bitCount1)-1;
	uint64_t bitMask2 = (1ull<<bitCount2)-1;
	mem[offset1] = (mem[offset1]&(~(bitMask1<<bitOffset1))) | ((val&bitMask1)<<bitOffset1);
	mem[offset1+1] = (mem[offset1+1]&(~(bitMask2))) | ((val>>bitCount1)&bitMask2);
	//	uint64_t result = (mem[offset1]>>bitOffset1)&bitMask1;
	//	result = ((mem[offset1+1]&bitMask2)<<bitCount2) | result;
}

template <>
uint64_t NBitArray<64>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<32>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<16>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<8>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<4>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<2>::Get(uint64_t index) const;
template <>
uint64_t NBitArray<1>::Get(uint64_t index) const;
template <>
void NBitArray<64>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<32>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<16>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<8>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<4>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<2>::Set(uint64_t index, uint64_t val);
template <>
void NBitArray<1>::Set(uint64_t index, uint64_t val);



#endif
