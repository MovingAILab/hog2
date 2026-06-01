//
//  NBitVectorTest.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "NBitVectorTest.h"
#include "NBitArray.h"
#include "Timer.h"
#include "FourBitArray.h"
#include <assert.h>

void TimingTest()
{
	const int arraySize = 1000001;
	Timer t;
	
	{
		NBitArray<4> bitArray(arraySize);
		t.StartTimer();
		for (int x = 0; x < 100; x++)
		{
			bitArray.Clear();
			for (int y = 0; y < arraySize; y++)
				bitArray.Set(y, y);
			for (int y = 0; y < arraySize; y++)
				bitArray.Set(y, bitArray.Get(y)+1);
		}
		t.EndTimer();
		printf("%1.2fs elapsed testing generic 4-bit array\n", t.GetElapsedTime());
	}

	{
		FourBitArray bitArray(arraySize);
		t.StartTimer();
		for (int x = 0; x < 100; x++)
		{
			bitArray.Clear();
			for (int y = 0; y < arraySize; y++)
				bitArray.Set(y, y);
			for (int y = 0; y < arraySize; y++)
				bitArray.Set(y, bitArray.Get(y)+1);
		}
		t.EndTimer();
		printf("%1.2fs elapsed testing specific 4-bit array\n", t.GetElapsedTime());
	}
}

void Correctness4Test()
{
	const int arraySize = 1000001;
	{
		NBitArray<4> bitArray(arraySize);
		FourBitArray bitArray2(arraySize);

		bitArray.Clear();
		bitArray2.Clear();
		for (int y = 0; y < arraySize; y++)
		{
			bitArray.Set(y, y);
			bitArray2.Set(y, y);
		}
		for (int y = 0; y < arraySize; y++)
		{
			assert(bitArray.Get(y) == bitArray2.Get(y));
			assert(bitArray.Get(y) == (y&0xF));
			
			bitArray.Set(y, bitArray.Get(y)+1);
			bitArray2.Set(y, bitArray2.Get(y)+1);
		}
		for (int y = 0; y < arraySize; y++)
		{
			assert(bitArray.Get(y) == bitArray2.Get(y));
		}
	}
	printf("Generic and specific 4-bit arrays match\n");
}

template <int numBits>
void CorrectnessTest()
{
	printf("[%d bit test] Starting...\n", numBits);
	Timer t;
	t.StartTimer();
	const int arraySize = 10000019;
	{
		NBitArray<numBits> bitArray(arraySize);
		
		uint64_t maxVal;// = -1;
		if (numBits == 64)
			maxVal = -1;
		else
			maxVal = (1ull<<numBits)-1;
		
		bitArray.Clear();
		for (int y = 0; y < arraySize; y++)
		{
			if (bitArray.Get(y) != 0)
			{
				printf("Getting element %d, should be %d; got %llu\n",
					   y, 0, bitArray.Get(y));
			}
			assert(bitArray.Get(y) == 0);
		}
		bitArray.FillMax();
		for (int y = 0; y < arraySize; y++)
		{
			if (bitArray.Get(y) != maxVal)
			{
				printf("Getting element %d, should be %llu; got %llu\n",
					   y, maxVal, bitArray.Get(y));
			}
			assert(bitArray.Get(y) == maxVal);
		}
		for (int y = 0; y < arraySize; y++)
		{
			bitArray.Set(y, y);
		}
		for (int y = 0; y < arraySize; y++)
		{
			if (bitArray.Get(y) != (y&maxVal))
			{
				printf("Getting element %d, should be %llu; got %llu\n",
					   y, y&maxVal, bitArray.Get(y));
			}
			assert(bitArray.Get(y) == (y&maxVal));
			
			bitArray.Set(y, bitArray.Get(y)+1);
		}
	}
	printf("[%d bit test] Passed all tests [%1.3f]\n", numBits, t.EndTimer());
}

void TestNBitVector()
{
	TimingTest();
	Correctness4Test();
	CorrectnessTest<1>();
	CorrectnessTest<2>();
	CorrectnessTest<3>();
	CorrectnessTest<4>();
	CorrectnessTest<5>();
	CorrectnessTest<6>();
	CorrectnessTest<7>();
	CorrectnessTest<8>();
	CorrectnessTest<9>();
	CorrectnessTest<10>();
	CorrectnessTest<11>();
	CorrectnessTest<12>();
	CorrectnessTest<13>();
	CorrectnessTest<14>();
	CorrectnessTest<15>();
	CorrectnessTest<16>();
	CorrectnessTest<17>();
	CorrectnessTest<18>();
	CorrectnessTest<19>();
	CorrectnessTest<20>();
	CorrectnessTest<21>();
	CorrectnessTest<22>();
	CorrectnessTest<23>();
	CorrectnessTest<24>();
	CorrectnessTest<25>();
	CorrectnessTest<26>();
	CorrectnessTest<27>();
	CorrectnessTest<28>();
	CorrectnessTest<29>();
	CorrectnessTest<30>();
	CorrectnessTest<31>();
	CorrectnessTest<32>();
	CorrectnessTest<33>();
	CorrectnessTest<34>();
	CorrectnessTest<35>();
	CorrectnessTest<36>();
	CorrectnessTest<37>();
	CorrectnessTest<38>();
	CorrectnessTest<39>();
	CorrectnessTest<40>();
	CorrectnessTest<41>();
	CorrectnessTest<42>();
	CorrectnessTest<43>();
	CorrectnessTest<44>();
	CorrectnessTest<45>();
	CorrectnessTest<46>();
	CorrectnessTest<47>();
	CorrectnessTest<48>();
	CorrectnessTest<49>();
	CorrectnessTest<50>();
	CorrectnessTest<51>();
	CorrectnessTest<52>();
	CorrectnessTest<53>();
	CorrectnessTest<54>();
	CorrectnessTest<55>();
	CorrectnessTest<56>();
	CorrectnessTest<57>();
	CorrectnessTest<58>();
	CorrectnessTest<59>();
	CorrectnessTest<60>();
	CorrectnessTest<61>();
	CorrectnessTest<62>();
	CorrectnessTest<63>();
	CorrectnessTest<64>();
	printf("Passed all tests successfully\n");
}
