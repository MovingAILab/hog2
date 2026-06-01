//
//  NBitArray.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/5/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include <stdio.h>
#include "NBitArray.h"

template <>
uint64_t NBitArray<64>::GetMaxValue() const
{
	uint64_t v = 0;
	return ~v;
}


template <>
uint64_t NBitArray<64>::Get(uint64_t index) const
{
	return mem[index];
}

template <>
uint64_t NBitArray<32>::Get(uint64_t index) const
{
	return (mem[index>>1]>>(32*(index&0x1)))&0xFFFFFFFF;
}

template <>
uint64_t NBitArray<16>::Get(uint64_t index) const
{
	return (mem[index>>2]>>(16*(index&0x3)))&0xFFFF;
}

template <>
uint64_t NBitArray<8>::Get(uint64_t index) const
{
	return (mem[index>>3]>>(8*(index&0x7)))&0xFF;
}

template <>
uint64_t NBitArray<4>::Get(uint64_t index) const
{
	return (mem[index>>4]>>(4*(index&0xF)))&0xF;
}

template <>
uint64_t NBitArray<2>::Get(uint64_t index) const
{
	return (mem[index>>5]>>(2*(index&0x1F)))&0x3;
}

template <>
uint64_t NBitArray<1>::Get(uint64_t index) const
{
	return (mem[index>>6]>>(1*(index&0x3F)))&0x1;
}



template <>
void NBitArray<64>::Set(uint64_t index, uint64_t val)
{
	mem[index] = val;
}

template <>
void NBitArray<32>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFFFFFFFF)<<(32*(index&0x1));
	uint64_t mask = ~((0xFFFFFFFFull)<<(32*(index&0x1)));
	mem[index>>1] = (mem[index>>1]&mask)|val;
}

template <>
void NBitArray<16>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFFFF)<<(16*(index&0x3));
	uint64_t mask = ~((0xFFFFull)<<(16*(index&0x3)));
	mem[index>>2] = (mem[index>>2]&mask)|val;
}

template <>
void NBitArray<8>::Set(uint64_t index, uint64_t val)
{
	val = (val&0xFF)<<(8*(index&0x7));
	uint64_t mask = ~((0xFFull)<<(8*(index&0x7)));
	mem[index>>3] = (mem[index>>3]&mask)|val;
}

template <>
void NBitArray<4>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0xF)<<(4*(index&0xF));
	uint64_t mask = ~((0xFull)<<(4*(index&0xF)));
	mem[index>>4] = (mem[index>>4]&mask)|val;
}

template <>
void NBitArray<2>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0x3)<<(2*(index&0x1F));
	uint64_t mask = ~((0x3ull)<<(2*(index&0x1F)));
	mem[index>>5] = (mem[index>>5]&mask)|val;
}

template <>
void NBitArray<1>::Set(uint64_t index, uint64_t val)
{
	// 2^4 = 16;
	val = (val&0x1)<<(1*(index&0x3F));
	uint64_t mask = ~((0x1ull)<<(1*(index&0x3F)));
	mem[index>>6] = (mem[index>>6]&mask)|val;
}
