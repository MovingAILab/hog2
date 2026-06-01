//
//  DiskBitFile.cpp
//  Rubik
//
//  Created by Nathan Sturtevant on 4/5/13.
//  Copyright (c) 2013 Nathan Sturtevant. All rights reserved.
//

#include "DiskBitFile.h"

DiskBitFile::DiskBitFile(const char *pre)
{
//	subBucketBits = subSize;
	strncpy(prefix, pre, 62);
	outputFile = 0;
	outputBucket = -1;
	outputSubBucket = -1;
	cacheOffset = -1; // beginning of cache [in bytes]
	
	theCacheSize = 0;      // valid bytes in the cache
	cacheFilePosition = 0; // current offset in file (bytes)
	cacheChanged = false;
	
	bytesRead = 0;
	bytesWritten = 0;


	fileOpen = false;
	chunkFile = 0;
	fileOffset = 0;
	currBucket = -1;
	currSubBucket = -1;
}

DiskBitFile::~DiskBitFile()
{
	CloseReadFile();
	CloseReadWriteFile();
}

void DiskBitFile::CloseReadWriteFile()
{
	if (outputFile != 0)
	{
		fclose(outputFile);
		//printf("Closing file\n");fflush(stdout);
		outputFile = 0;
	}
	outputBucket = -1;
	outputSubBucket = -1;
	cacheOffset = -1;
	cacheFilePosition = 0;
}

// incoming offset is in entries, not bytes
void DiskBitFile::WriteFileDepth(int bucket, int64_t offset, uint8_t value)
{
	int64_t subBucket = (offset*BITS/8)>>subBucketBits;
	if (bucket == -1)
	{
		FlushCache();
		CloseReadWriteFile();
		return;
	}
	if ((bucket != outputBucket) || (subBucket != outputSubBucket))
	{
		FlushCache();
		CloseReadWriteFile();
	}
	if (outputFile == 0)
	{
	  //printf("Opening '%s'\n", getBucketFileName(bucket, subBucket)); fflush(stdout);
		outputFile = fopen(getBucketFileName(bucket, subBucket), "r+");
		if (outputFile == 0) { printf("Unable to open '%s'; aborting\n", getBucketFileName(bucket, subBucket)); exit(0); }
		outputBucket = bucket;
		outputSubBucket = subBucket;
		cacheFilePosition = 0;
		cacheOffset = -1;
	}
	
	offset -= subBucket*(1<<subBucketBits)*8/BITS;
#if BITS==8
	assert(cacheOffset != -1);
	assert(offset-cacheOffset < theCacheSize);
	assert(offset-cacheOffset >= 0);
	cache[offset-cacheOffset] = value;
	//printf("Writing %d to local cache offset %lld\n", value, offset-cacheOffset);
	cacheChanged = true;
#elif BITS==4
	assert(cacheOffset != -1);
	assert((offset>>1)-cacheOffset < theCacheSize);
	assert((offset>>1)-cacheOffset >= 0);
	uint8_t curr;
	curr = cache[(offset>>1)-cacheOffset];
	curr &= (~(0xF<<(4*(offset%2)))); // wipe out old value
	curr |= (value<<(4*(offset%2))); // or in new value
	cache[(offset>>1)-cacheOffset] = curr;
	cacheChanged = true;
#elif BITS==2
	assert(cacheOffset != -1);
	assert((offset>>2)-cacheOffset < theCacheSize);
	assert((offset>>2)-cacheOffset >= 0);
	uint8_t curr;
	curr = cache[(offset>>2)-cacheOffset];
	curr &= (~(0x3<<(2*(offset%4)))); // wipe out old value
	curr |= (value<<(2*(offset%4))); // or in new value
	cache[(offset>>2)-cacheOffset] = curr;
	cacheChanged = true;
#else // BITS==1
	assert(false);
#endif
	
}

int DiskBitFile::ReadFileDepth(int bucket, int64_t offset)
{
	int64_t subBucket = (offset*BITS/8)>>subBucketBits;
	if (bucket == -1)
	{
		FlushCache();
		CloseReadWriteFile();
		return 0;
	}
	if ((bucket != outputBucket) || (subBucket != outputSubBucket))
	{
		FlushCache();
		CloseReadWriteFile();
	}
	if (outputFile == 0)
	{
	  //printf("Opening '%s'\n", getBucketFileName(bucket, subBucket)); fflush(stdout);
		outputFile = fopen(getBucketFileName(bucket, subBucket), "r+");
		if (outputFile == 0) { printf("Unable to open '%s'; aborting\n", getBucketFileName(bucket, subBucket)); exit(0); }
		outputBucket = bucket;
		outputSubBucket = subBucket;
		cacheFilePosition = 0;
		cacheOffset = -1;
	}
	
	offset -= subBucket*(1<<subBucketBits)*8/BITS;
#if BITS==8
	if ((cacheOffset == -1) || // just read into memory
		(offset-cacheOffset < 0)  || (offset-cacheOffset >= theCacheSize))
	{
		FlushCache();
		
		cacheOffset = offset&(~(cacheSize-1));
		fseek(outputFile, cacheOffset-cacheFilePosition, SEEK_CUR);
		//fseek(outputFile, cacheOffset, SEEK_SET);
		cacheFilePosition += cacheOffset-cacheFilePosition;
		theCacheSize = fread(cache, sizeof(uint8_t), cacheSize, outputFile);
		chunksRead++;
		cacheFilePosition += theCacheSize;
		//printf("cacheOffset: %lld; cacheFilePosition: %lld\n", cacheOffset, cacheFilePosition);
	}
	return cache[offset-cacheOffset];
	
#elif BITS==4
	
	if ((cacheOffset == -1) || // just read into memory
		(offset>>1)-cacheOffset < 0  || (offset>>1)-cacheOffset >= theCacheSize)
	{
		FlushCache();
		
		cacheOffset = (offset>>1)&(~(cacheSize-1));
		fseek(outputFile, cacheOffset-cacheFilePosition, SEEK_CUR);
		//fseek(outputFile, cacheOffset, SEEK_SET);
		cacheFilePosition += cacheOffset-cacheFilePosition;
		theCacheSize = fread(cache, sizeof(uint8_t), cacheSize, outputFile);
		//chunksRead++;
		bytesRead += cacheSize;
		cacheFilePosition += theCacheSize;
	}
	return (cache[(offset>>1)-cacheOffset]>>(4*(offset%2)))&0xF;
	
#elif BITS==2
	if ((cacheOffset == -1) || // just read into memory
		(offset>>2)-cacheOffset < 0  || (offset>>2)-cacheOffset >= theCacheSize)
	{
		FlushCache();
		
		cacheOffset = (offset>>2)&(~(cacheSize-1));
		fseek(outputFile, cacheOffset-cacheFilePosition, SEEK_CUR);
		//fseek(outputFile, cacheOffset, SEEK_SET);
		cacheFilePosition += cacheOffset-cacheFilePosition;
		theCacheSize = fread(cache, sizeof(uint8_t), cacheSize, outputFile);
		chunksRead++;
		cacheFilePosition += theCacheSize;
	}
	return (cache[(offset>>2)-cacheOffset]>>(2*(offset%4)))&0x3;
	
#else // BITS==1
	assert(false);
#endif
}

void DiskBitFile::CloseReadFile()
{
	if (fileOpen)
	{
		fclose(chunkFile);
		chunkFile = 0;
		//printf("Closing READ file\n");
		fileOpen = false;
	}
}

uint8_t *DiskBitFile::ReadChunk(int bucket, int64_t offset, int numEntries, uint8_t *data)
{
	
	int64_t subBucket = (offset*BITS/8)>>subBucketBits;
	offset -= subBucket*(1<<subBucketBits)*8/BITS;
	
	if (bucket == -1)
	{
		if (fileOpen)
		{
			fclose(chunkFile);
			//printf("Closing READ file\n");
			fileOpen = false;
		}
		return 0;
	}
	if (fileOpen && ((bucket != currBucket) || (subBucket != currSubBucket)))
	{
		fclose(chunkFile);
		chunkFile = 0;
		//printf("Closing READ file\n");
		fileOpen = false;
	}
	if (!fileOpen)
	{
		//printf("Opening %s for READ only\n", getBucketFileName(bucket, subBucket));
		chunkFile = fopen(getBucketFileName(bucket, subBucket), "r");
		if (chunkFile == 0)
		{
			printf("Unable to open file %s\n", getBucketFileName(bucket, subBucket));
			exit(0);
		}
		fileOffset = 0;
		currBucket = bucket;
		currSubBucket = subBucket;
		fileOpen = true;
	}
	
	fseek(chunkFile, offset*BITS/8-fileOffset, SEEK_CUR);
	fileOffset+=offset*BITS/8-fileOffset;
	//	fseek(f, offset*BITS/8, SEEK_SET);
	
	int alignedSize = (numEntries*BITS+7)/8;
	//	uint8_t *data = GetMemoryChunk((openSize*BITS+7)/8); //new uint8_t[alignedSize];
	assert(0 == offset%2);
	fread(data, sizeof(uint8_t), alignedSize, chunkFile);
	fileOffset += alignedSize;
	bytesRead += alignedSize;
	//chunksRead++;
	return data;
}

void DiskBitFile::FlushCache()
{
	if (cacheChanged)
	{
		fseek(outputFile, cacheOffset-cacheFilePosition, SEEK_CUR);
		//fseek(outputFile, cacheOffset, SEEK_SET);
		cacheFilePosition += cacheOffset-cacheFilePosition;
		fwrite(cache, sizeof(uint8_t), theCacheSize, outputFile);
		bytesWritten += theCacheSize;
		//chunksWritten++;
		cacheFilePosition += theCacheSize;
		cacheChanged = false;
	}
}

void DiskBitFile::Init(const std::vector<bucketData> &buckets)
{
	int subBucket = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
	{
		printf("Bucket %d has %" PRId64 " entries\n", x, buckets[x].theSize);
		
		FILE *f = fopen(getBucketFileName(x, subBucket), "w");
		if (f == 0)
		{ printf("Error opening file '%s'\n", getBucketFileName(x, subBucket)); exit(0); }
		
		//fseek(f, offset, SEEK_CUR);
		int64_t totalBytes = (buckets[x].theSize*BITS+7)/8;
		uint8_t data[2048];
		for (int y = 0; y < 2048; y++)
			data[y] = 0xFF;
		for (int64_t y = 0; y < totalBytes; y+=2048)
		{
			int currSubBucket = y>>subBucketBits;
			if (currSubBucket != subBucket)
			{
				fclose(f);
				subBucket = currSubBucket;
				FILE *f = fopen(getBucketFileName(x, subBucket), "w");
				if (f == 0)
				{ printf("Error opening file '%s'\n", getBucketFileName(x, subBucket)); exit(0); }
			}
			int64_t amnt = std::min(totalBytes-y, (int64_t)2048ll);
			fwrite(data, sizeof(uint8_t), amnt, f);
		}
		fclose(f);
		//buckets[x].data.resize(buckets[x].theSize);
		fflush(stdout);
	}
}

const char *DiskBitFile::getBucketFileName(int bucket, int subBucket)
{
	//static char fname[255];
	assert(bucket >= 0);
	sprintf(bucketFileName, "%s-%d-b%d.%d", prefix, BITS, bucket, subBucket);
	return bucketFileName;
}
