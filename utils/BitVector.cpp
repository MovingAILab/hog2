/*
 *  $Id: BitVector.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstdlib>
#include <cstdio>
#include "BitVector.h"
#include <cinttypes>
#include <cassert>

BitVector::BitVector(uint64_t _size)
{
	true_size = _size;
	size = (_size>>storageBitsPower)+1;
	storage = new storageElement[size];
	for (size_t x = 0; x < size; x++)
		storage[x] = 0;
}

BitVector::BitVector(uint64_t size, const char *fname)
{
	Load(fname);
	assert(size == true_size);
}

BitVector::~BitVector()
{
	delete [] storage;
}

void BitVector::Save(const char *file)
{
	FILE *f = fopen(file, "w+");
	if (f == 0)
	{
		printf("File write error (%s)\n", file);
		exit(0);
	}
	fwrite(&true_size, sizeof(true_size), 1, f);
	fwrite(storage, sizeof(storageElement), size, f);
	fclose(f);
}

void BitVector::Load(const char *file)
{
	delete [] storage;
	FILE *f = fopen(file, "r");
	if (f == 0)
	{
		printf("File read error (%s)\n", file);
		exit(0);
	}
	//fread(&size, sizeof(size), 1, f);
	fread(&true_size, sizeof(true_size), 1, f);
	printf("Loading %" PRId64 " entries\n", true_size);
	// allocate storage
	size = (true_size>>storageBitsPower)+1;
	
	storage = new storageElement[size];
	//	for (int x = 0; x < size; x++)
	//		storage[x] = 0;
	
	// TODO:
	fread(storage, sizeof(storageElement), size, f);
	fclose(f);
}

void BitVector::clear()
{
	for (int x = 0; x < size; x++)
		storage[x] = 0;
}

//BitVector *BitVector::Clone()
//{
//  BitVector *bv = new BitVector(true_size);
//  bv->Merge(this);
//  return bv;
//}

void BitVector::Set(uint64_t index, bool value)
{
	if ((index>>storageBitsPower) > size) {
		printf("SET %" PRId64 " OUT OF RANGE\n", index);
		exit(0);
	}
	if (value)
		storage[index>>storageBitsPower] = storage[index>>storageBitsPower]|(1<<(index&storageMask));
	else
		storage[index>>storageBitsPower] = storage[index>>storageBitsPower]&(~(1<<(index&storageMask)));
}


//void BitVector::Merge(BitVector *bv)
//{
//  if (bv == 0) return;
//  if (bv->size != size) {
//    printf("Error; can't merge vectors of different sizes (%d/%d)\n", bv->true_size, true_size);
//    return;
//  }
//  for (int x = 0; x < size; x++) storage[x] |= bv->storage[x];
//}

bool BitVector::Equals(BitVector *bv)
{
	if (bv->size != size) return false;
	for (size_t x = 0; x < size; x++)
		if (storage[x] != bv->storage[x])
			return false;
	return true;
}

uint64_t BitVector::GetNumSetBits()
{
	uint64_t sum = 0;
	for (size_t x = 0; x < size; x++)
	{
		storageElement iter = storage[x];
		while (iter) {
			sum++;
			iter &= (iter-1);
		}
	}
	return sum;
}
