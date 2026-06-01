/*
 *  BitMap.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/4/11.
 *  Copyright 2011 NS Software. All rights reserved.
 *
 */

#include "BitMap.h"
#include <stdio.h>
#include <iostream>

BitMapPic::BitMapPic(int w, int h):width(w), height(h)
{
	image = new uint8_t[width*height*4];
}

BitMapPic::BitMapPic(const char* file)
{
	bool loadFailed = false;
	FILE *f = fopen(file, "rb");
	BitMapHeader header;
	
	if (f == 0)
	{
		std::cout << "Unable to find file\n";
		loadFailed = true;
	}
	else
	{
		uint16_t bfType;
		fread(&bfType, sizeof(bfType), 1, f);
		if (bfType != 19778)
		{
			std::cout << "Unable to load file properly\n";
			fclose(f);
			loadFailed = true;
		}
		else
		{
			fread(&header, sizeof(BitMapHeader), 1, f);
			if (header.biBitCount < 24)
			{
				std::cout << "We only support 24 & 32-bit files\n";
				fclose(f);
				loadFailed = true;
			}
			else
			{
				std::cout << ":" << header.biBitCount << "bit image\n";
				
				if (header.biCompression != 0)
				{
					std::cout << "We don't support compressed files\n";
					fclose(f);
					loadFailed = true;
				}
				else
				{
					height = header.biHeight;
					bool reverseHeight = false;
					if ((int32_t)header.biHeight < 0)
					{
						std::cout << ":height reversed\n";
						height = -height;
						reverseHeight = true;
					}
					
					width = header.biWidth;
					image = new uint8_t[height*width*4];
					
					fseek(f, header.bfOffBits, SEEK_SET);
					
					if (header.biBitCount == 32)
					{
						for (size_t x = 0; x < height; x++)
						{
							if (reverseHeight)
								fread(&image[(height-x-1)*width*4], sizeof(char), width*4, f);
							else
								fread(&image[x*width*4], sizeof(char), width*4, f);
						}
					}
					else if (header.biBitCount == 24)
					{
						bool padding = false;
						for (int x = 0; x < height; x++)
						{
							int bytesRead = 0;
							for (int y = 0; y < width; y++)
							{
								if (reverseHeight)
									bytesRead += fread(&image[(height-x-1)*width*4+y*4], sizeof(char), 3, f);
								else
									bytesRead += fread(&image[x*width*4+y*4], sizeof(char), 3, f);
								image[x*width*4+y*4+3] = 0;
							}
							while (0 != bytesRead%4)
							{
								char zero[4] = {0, 0, 0, 0};
								bytesRead += fread(zero, sizeof(char), 1, f);
								padding = true;
							}
						}
						if (padding)
							std::cout << ":padding necessary\n";
					}
					fclose(f);
				}
			}
		}
	}
	
	if(loadFailed)
	{
		width = 10;
		height = 10;
		image = new uint8_t[width*height*4];
		for(int i=0; i < width*height*4; i++)
		{
			image[i] = 255;
		}
	}
	
	
}

BitMapPic::BitMapPic(const BitMapPic& b):width(b.width), height(b.height)
{
	image = new uint8_t[width*height*4];
	
	for(int i=0; i < width*height*4; i++)
	{
		image[i] = b.image[i];
	}
}

BitMapPic::~BitMapPic()
{
	width = 0;
	height = 0;
	if(image)
	{
		delete[] image;
		image = NULL;
	}
}

BitMapPic& BitMapPic::operator=(const BitMapPic& b)
{
	if(this == &b)
		return *this;
	
	width = b.width;
	height = b.height;
	
	delete[] image;
	image = new uint8_t[width*height*4];
	
	for(int i=0; i < width*height*4; i++)
	{
		image[i] = b.image[i];
	}
	
	return *this;
}

void BitMapPic::Save(const char *file)
{
	FILE *f = fopen(file, "w+");
	
	if (f == 0)
		return;
	
	BitMapHeader header;
	header.biWidth = width;
	header.biHeight = height;
	
	header.bfSize = sizeof(BitMapHeader)+2+(width)*height*4;
	header.biSizeImage = (width)*height*4;
	uint16_t bfType = 19778; // 0x4D42
	fwrite(&bfType, sizeof(bfType), 1, f);
	fwrite(&header, sizeof(header), 1, f);
	for (int x = 0; x < height; x++)
	{
		fwrite(&image[x*width*4], sizeof(char), width*4, f);
	}
	fclose(f);
}


void BitMapPic::SetPixel(int x, int y, uint8_t redByte, uint8_t greenByte, uint8_t blueByte, uint8_t alphaByte )
{
	// BGRA
	image[y*width*4+x*4+0] = blueByte;
	image[y*width*4+x*4+1] = greenByte;
	image[y*width*4+x*4+2] = redByte;
	image[y*width*4+x*4+3] = alphaByte;
}

void BitMapPic::GetPixel(int x, int y, uint8_t &redByte, uint8_t &greenByte, uint8_t &blueByte, uint8_t &alphaByte) const
{
	blueByte = image[y*width*4+x*4+0];
	greenByte = image[y*width*4+x*4+1];
	redByte = image[y*width*4+x*4+2];
	alphaByte = image[y*width*4+x*4+3];
}


//
//BitMapPic::BitMapPic(const char *file)
//{
//	Load(file);
//}
//
//BitMapPic::BitMapPic(int w, int h)
//:width(w), height(h)
//{
//	image.resize(w*h*4);
//	bytesReversed = false;
//}
//
//BitMapPic::BitMapPic(int w, int h, uint8_t *data)
//:width(w), height(h)
//{
//	bytesReversed = false;
//	image.resize(w*h*4);
//	// slow copy, but correct TODO: memcpy
//	for (int x = 0; x < w*h*4; x++)
//		image[x] = data[x];
//}
//
//void BitMapPic::Save(const char *file)
//{
//	FILE *f = fopen(file, "w+");
//
//	if (f == 0) return;
//
////	long rowBytes = width * 4;
//	char zero[4] = {0, 0, 0, 0};
//
//	bmp_header h;
//	h.biWidth = width;
//	if (bytesReversed)
//		h.biHeight = -height;
//	else
//		h.biHeight = height;
//
//	int buffer = (4-width%4)%4;
//	h.bfSize = sizeof(bmp_header)+2+(width+buffer)*height*4;
//	h.biSizeImage = (width+buffer)*height*4;
//	uint16_t bfType = 19778;
//	fwrite(&bfType, sizeof(bfType), 1, f);
//	fwrite(&h, sizeof(bmp_header), 1, f);
//	for (int x = 0; x < height; x++)
//	{
//		fwrite(&image[x*width*4], sizeof(char), width*4, f);
//		if (0 != width%4)
//			fwrite(&zero, sizeof(char), buffer, f);
//	}
//	fclose(f);
//}
//
//void BitMapPic::Load(const char *file)
//{
//	FILE *f = fopen(file, "rb");
//
//	if (f == 0)
//	{
//		printf("Unable to find file\n");
//		width = 0;
//		height = 0;
//		image.resize(0);
//		return;
//	}
//
////	long rowBytes = width * 4;
////	long imageSize = rowBytes * height;
//	char zero[4] = {0, 0, 0, 0};
//	int buffer = (4-width%4)%4;
//
//	bmp_header h;
//	uint16_t bfType;
//	fread(&bfType, sizeof(bfType), 1, f);
//	if (bfType != 19778)
//	{
//		printf("Unable to load file properly\n");
//		width = 0;
//		height = 0;
//		image.resize(0);
//		fclose(f);
//		return;
//	}
//	fread(&h, sizeof(bmp_header), 1, f);
//	h.dump();
//	height = h.biHeight;
//	if ((int32_t)h.biHeight < 0)
//	{
//		height = -h.biHeight;
//		bytesReversed = true;
//	}
//	width = h.biWidth;
//	image.resize(height*width*4);
////	printf("char size: %d\n", sizeof(char));
//	fseek(f, h.bfOffBits, SEEK_SET);
//	for (int x = 0; x < height; x++)
//	{
////		printf("Reading at %d (%d)\n", x*width*4, width*4);
//		//printf("Read: %d\n", );
//		fread(&image[x*width*4], sizeof(char), width*4, f);
//		if (0 != width%4)
//		{
//			fread(&zero, sizeof(char), buffer, f);
//		}
//	}
//	fclose(f);
////	for (unsigned int x = image.size()-4096; x < image.size(); x++)
////		printf("%4d", image[x]);
//}
//
