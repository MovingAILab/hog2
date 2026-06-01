//
//  FourBitArray.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 4/8/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__FourBitArray__
#define __hog2_glut__FourBitArray__

#include <iostream>
#include <stdint.h>

// simple four-bit array with no bounds checking
class FourBitArray
{
public:
	FourBitArray(uint64_t numEntries = 0);
	~FourBitArray();
	void FillMax();
	void Clear();
	void Resize(uint64_t newMaxEntries);
	uint64_t Size() const;
	uint8_t Get(uint64_t index) const;
	void Set(uint64_t index, uint8_t val);
	void Write(const char *);
	void Read(const char *);
private:
	uint8_t *mem;
	uint64_t entries;
};

#endif /* defined(__hog2_glut__FourBitArray__) */
