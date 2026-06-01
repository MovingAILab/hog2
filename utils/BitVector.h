// HOG File
/*
 *  $Id: BitVector.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 09/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

#ifndef _BITVECTOR_
#define _BITVECTOR_

#include <stdint.h>
#include "MMapUtil.h"

/**
 * An efficient bit-wise vector implementation.
 */

//typedef uint32_t storageElement;
//const int storageBits = 32;
//const int storageBitsPower = 5;
//const int storageMask = 0x1F;

typedef uint8_t storageElement;
const int storageBits = 8;
const int storageBitsPower = 3;
const int storageMask = 0x7;


class BitVector {
public:
	BitVector(uint64_t size);
	BitVector(uint64_t size, const char *);
	~BitVector();
	void clear();
	uint64_t GetSize() { return true_size; }
	bool Get(uint64_t index) const;
	void Set(uint64_t index, bool value);
	void SetTrue(uint64_t index);
	void Save(const char *);
	void Load(const char *);
	bool Equals(BitVector *);
	uint64_t GetNumSetBits();
private:
	uint64_t size, true_size;
	storageElement *storage;
	int fd;
};

inline bool BitVector::Get(uint64_t index) const
{
	return (((storage[index>>storageBitsPower])>>(index&storageMask))&0x1);
}

inline void BitVector::SetTrue(uint64_t index)
{
	storage[index>>storageBitsPower] = storage[index>>storageBitsPower]|(1<<(index&storageMask));
}

#endif
