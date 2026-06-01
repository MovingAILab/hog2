/*
 *  BitMapPic.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/4/11.
 *  Copyright 2011 NS Software. All rights reserved.
 *
 */

#ifndef BITMAPPIC_H
#define BITMAPPIC_H

#include <stdio.h>
#include <vector>
#ifndef WIN32
#include <stdint.h>
#endif
#ifdef WIN32
#include "pstdint.h"
#endif

struct BitMapHeader
{
	BitMapHeader()
	:zero(0), bfOffBits(sizeof(BitMapHeader)+2), biSize(40), biPlanes(1),
	biBitCount(32), biCompression(0), biSizeImage(0), biXPelsPerMeter(2835), biYPelsPerMeter(2835),
	biClrUsed(0), biClrImportant(0) {}
	
	uint32_t bfSize;
	uint32_t zero;
	uint32_t bfOffBits;
	
	uint32_t biSize;
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes;
	uint16_t biBitCount;
	uint32_t biCompression;
	uint32_t biSizeImage;
	uint32_t biXPelsPerMeter;
	uint32_t biYPelsPerMeter;
	uint32_t biClrUsed;
	uint32_t biClrImportant;
};

class BitMapPic
{
public:
	BitMapPic(int w, int h);
	BitMapPic(const char* file);
	~BitMapPic();
	BitMapPic(const BitMapPic& b);
	BitMapPic& operator=(const BitMapPic& b);
	int GetWidth() { return width; }
	int GetHeight() { return height; }
	void Save(const char *file);
	void SetPixel(int x, int y, uint8_t redByte, uint8_t greenByte, uint8_t blueByte, uint8_t alphaByte=0);
	void GetPixel(int x, int y, uint8_t &redByte, uint8_t &greenByte, uint8_t &blueByte, uint8_t &alphaByte) const;
	uint8_t *GetBytes() { return &image[0]; }
	
private:
	uint32_t width, height;
	uint8_t *image;
};
//
//class BitMapPic {
//public:
//	BitMapPic(const char *file);
//	BitMapPic(int width, int height);
//	BitMapPic(int width, int height, uint8_t *data);
//	void Save(const char *file);
//	void Load(const char *file);
//	int GetWidth() { return width; }
//	int GetHeight() { return height; }
//	void SetPixel(int x, int y, uint8_t redByte, uint8_t greenByte, uint8_t blueByte, uint8_t alphaByte = 0)
//	{
//		if (BytesReversed())
//		{
//			image[y*width*4+x*4+0] = blueByte;
//			image[y*width*4+x*4+1] = greenByte;
//			image[y*width*4+x*4+2] = redByte;
//			image[y*width*4+x*4+3] = alphaByte;
//		}
//		else {
//			image[y*width*4+x*4+0] = redByte;
//			image[y*width*4+x*4+1] = greenByte;
//			image[y*width*4+x*4+2] = blueByte;
//			image[y*width*4+x*4+3] = alphaByte;
//		}
//	}
//	void GetPixel(int x, int y, uint8_t &redByte, uint8_t &greenByte, uint8_t &blueByte, uint8_t &alphaByte)
//	{
//		if (BytesReversed())
//		{
//			blueByte = image[y*width*4+x*4+0];
//			greenByte = image[y*width*4+x*4+1];
//			redByte = image[y*width*4+x*4+2];
//			alphaByte = image[y*width*4+x*4+3];
//		}
//		else {
//			redByte = image[y*width*4+x*4+0];
//			greenByte = image[y*width*4+x*4+1];
//			blueByte = image[y*width*4+x*4+2];
//			alphaByte = image[y*width*4+x*4+3];
//		}
//	}
//	uint8_t *GetBytes() { return &image[0]; }
//	bool BytesReversed() { return bytesReversed; }
//private:
//	uint32_t width, height;
//	bool bytesReversed;
//	std::vector<uint8_t> image;
//};

#endif
