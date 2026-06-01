/*
 *  $Id: map.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 03/07/07.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

#include <stack>
#include "Map.h"
#include "Constants.h"
#include <cstdlib>
#include <cstring>
#include "BitMap.h"

//GLuint wall = -1;

using namespace std;

static const bool verbose = false; 

//void InitTextures()
//{
//	if (wall == -1)
//	{
//		BitMapPic p("/Users/nathanst/hog2/textures/u.bmp");
//		p.Save("/Users/nathanst/hog2/textures/out.bmp");
//		// Use OpenGL ES to generate a name for the texture.
//		glGenTextures(1, &wall);
//		// Bind the texture name. 
//		glBindTexture(GL_TEXTURE_2D, wall);
//		// Set the texture parameters to use a minifying filter and a linear filer (weighted average)
//		
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 	
//		
////		if (p.BytesReversed())
//		{
//			// Specify a 2D texture image, providing the a pointer to the image data in memory
//			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, p.GetWidth(), p.GetHeight(), 0, GL_BGRA, GL_UNSIGNED_BYTE, p.GetBytes());
//		}
////		else {
////			// Specify a 2D texture image, providing the a pointer to the image data in memory
////			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, p.GetWidth(), p.GetHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE, p.GetBytes());
////		}
//	}
//}

/** 
* Construct a half tile, initializing to flat values.
*
* A halfTile is the lowest level in the map - it contains the height of the
* three relevant corners. The heights are initialized to 0, the type of
* territory is initialized to ground.
*/		
halfTile::halfTile()
{
	corners[0] = corners[1] = corners[2] = 0;
	type = kGround;
	node = -1;
}

/** 
* Construct a tile with no split.
*
* A Tile contains two half tiles and information about how the tile is
* split. If a tile isn't split, the first three parts of the tile are in
* the first halfTile and the fourth is in th second.
*/
Tile::Tile()
:tile1(), tile2(), split(kNoSplit)
{ }

/** 
* Create a new map of a particular size.
*
* A map is an array of tiles according to the height and width of the map.
*/
Map::Map(long _width, long _height)
:width(_width), height(_height)
{
	mapType = kOctile;
	tileSet = kFall;
	map_name[0] = 0;
	sizeMultiplier = 1;
	land = new Tile *[width];
	//	for (int x = 0; x < 8; x++)
	//		g[x] = 0;
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	drawLand = true;
//	dList = 0;
	updated = true;
	revision = 0;
	//	numAbstractions = 1;
	//	pathgraph = 0;
}

/** 
* Create a new map by copying it from another map.
*
* Creates a new map and initializes it with the map passed to it.
*/
Map::Map(Map *m)
{
	mapType = m->mapType;
	tileSet = m->tileSet;
	strncpy(map_name, m->map_name, 128);
	sizeMultiplier = m->sizeMultiplier;
	width = m->width;
	height = m->height;
	
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	
	drawLand = m->drawLand;
//	dList = 0;
	updated = true;
	revision = m->revision;
	
	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
			land[x][y] = m->land[x][y];
}

/** 
* Create a new map by loading it from a file.
*
* Creates a new map and initializes it with the file passed to it.
*/
Map::Map(const char *filename)
{
	sizeMultiplier = 1;
	land = 0;
	Load(filename);
	tileSet = kFall;
}

/** 
* Create a new map by loading it from a file pointer.
*
* Creates a new map and initializes it with the file pointer passed to it.
*/
Map::Map(FILE *f)
{
	sizeMultiplier = 1;
	map_name[0] = 0;
	land = 0;
	Load(f);
	tileSet = kFall;
}

/** 
* Not implemented.
*
* This function is not implemented.
*/
Map::Map(std::istringstream &/*data*/)
{
	sizeMultiplier = 1;
//	dList = 0;
	tileSet = kFall;
}

Map::~Map()
{
	for (int x = 0; x < width; x++)
		delete [] land[x];
	delete [] land;
	land = 0;
}

void Map::Scale(long newWidth, long newHeight)
{
	Tile **newLand;
	newLand = new Tile *[newWidth];
	for (int x = 0; x < newWidth; x++)
	{
		newLand[x] = new Tile [newHeight];
		for (int y = 0; y < newHeight; y++)
		{
			newLand[x][y] = land[(x*width)/newWidth][(y*height)/newHeight];
			
			if (((y*height)/newHeight > 0) && (AdjacentEdges((x*width)/newWidth, (y*height)/newHeight, kTopEdge)))
			{ // make sure top edge is still adjacent
				newLand[x][y].tile1.corners[0] = newLand[x][y-1].tile1.corners[1];
				// this won't work with splits...
				newLand[x][y].tile2.corners[0] = newLand[x][y-1].tile1.corners[2];
			}
			if (((x*width)/newWidth > 0) && (AdjacentEdges((x*width)/newWidth, (y*height)/newHeight, kLeftEdge)))
			{ // make sure left edge is still adjacent
				newLand[x][y].tile1.corners[0] = newLand[x-1][y].tile2.corners[0];
				// this won't work with splits...
				newLand[x][y].tile1.corners[1] = newLand[x-1][y].tile1.corners[2];
			}
		}
	}
	for (int x = 0; x < width; x++) delete [] land[x];
	delete [] land;
	land = newLand;
	width = newWidth;
	height = newHeight;
	revision++;
	updated = true;
	map_name[0] = 0;
}

void Map::Trim()
{
	int MinX = width-1, MinY = height-1, MaxX = 0, MaxY = 0;
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			if (GetTerrainType(x, y) != kOutOfBounds)
			{
				MinX = min(MinX, x);
				MaxX = max(MaxX, x);
				MinY = min(MinY, y);
				MaxY = max(MaxY, y);
			}
		}
	}

	int newWidth = MaxX-MinX+1;
	int newHeight = MaxY-MinY+1;
	Tile **newLand;
	newLand = new Tile *[newWidth];
	for (int x = 0; x < newWidth; x++)
	{
		newLand[x] = new Tile [newHeight];
		for (int y = 0; y < newHeight; y++)
		{
			newLand[x][y] = land[x+MinX][y+MinY];
		}
	}
	for (int x = 0; x < width; x++) delete [] land[x];
	delete [] land;
	land = newLand;
	width = newWidth;
	height = newHeight;
	revision++;
	updated = true;
	map_name[0] = 0;
}


/** 
* Resets the current map by loading the file passed in.
*
* Resets the current map by loading the file passed in.
*/
void Map::Load(const char *filename)
{
	if (land)
	{
		for (int x = 0; x < width; x++)
			delete [] land[x];
		delete [] land;
		land = 0;
	}
	revision++;
	FILE *f = fopen(filename, "r");
	if (f)
	{
		Load(f);
		fclose(f);
		strncpy(map_name, filename, 128);
	}
	else {
		printf("Error! Can't open file %s\n", filename);
		width = height = 64;
		land = new Tile *[width];
		//	for (int x = 0; x < 8; x++)
		//		g[x] = 0;
		for (int x = 0; x < width; x++)
			land[x] = new Tile [height];
		drawLand = true;
//		dList = 0;
		updated = true;
		map_name[0] = 0;
	}
}

/** 
* Resets the current map by loading the file from the pointer passed in.
*/
void Map::Load(FILE *f)
{
	if (land)
	{
		for (int x = 0; x < width; x++)
			delete [] land[x];
		delete [] land;
		land = 0;
	}
	
	char format[32];
	// ADD ERROR HANDLING HERE
	int num = fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", format, &height, &width);
	// printf("got height %d, width %d\n", height, width);
	if (num == 3)
	{
		if (strcmp(format, "octile") == 0)
			loadOctile(f, height, width);
		else if (strcmp(format, "octile-corner") == 0)
			loadOctileCorner(f, height, width);
		else if (strcmp(format, "raw") == 0)
			loadRaw(f, height, width);
		return;
	}
	if (tryLoadRollingStone(f))
	{
		return;
	}
	if (tryDragonAge(f))
	{
		//Trim();
		return;
	}

	{
		printf("Unknown map type; aborting load!\n");
		width = height = 64;
		land = new Tile *[width];
		//	for (int x = 0; x < 8; x++)
		//		g[x] = 0;
		for (int x = 0; x < width; x++) land[x] = new Tile [height];
		drawLand = true;
//		dList = 0;
		updated = true;
		map_name[0] = 0;
	}
}

void Map::loadRaw(FILE *f, int high, int wide)
{
	mapType = kRaw;
	height = high;
	width = wide;
	land = new Tile *[wide];
	for (int x = 0; x < wide; x++)
		land[x] = new Tile [high];
	drawLand = true;
//	dList = 0;
	updated = true;
	for (int x = 0; x < wide; x++)
	{
		fread(&land[x][0], sizeof(Tile), wide, f);
		for (int y = 0; y < high; y++)
		{
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
	}
}

void Map::loadOctile(FILE *f, int high, int wide)
{
	mapType = kOctile;
	height = high*sizeMultiplier;
	width = wide*sizeMultiplier;
	land = new Tile *[wide*sizeMultiplier];
	for (int x = 0; x < wide*sizeMultiplier; x++) land[x] = new Tile [high*sizeMultiplier];
	drawLand = true;
//	dList = 0;
	updated = true;
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			char what;
			fscanf(f, "%c", &what);
			switch (toupper(what))
			{
				case '@':
				case 'O':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kOutOfBounds); break;
				case 'S':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kSwamp); break;
				case 'W':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kWater); break;
				case 'T':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kTrees); break;
				case 'G':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
										   kGrass); break;
				case 'B':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
										   kBlight); break;
				default:
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							SetTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kGround); break;
			}
			for (int r = 0; r < sizeMultiplier; r++)
				for (int s = 0; s < sizeMultiplier; s++)
				{
					land[x*sizeMultiplier+r][y*sizeMultiplier+s].tile1.node = kNoGraphNode;
					land[x*sizeMultiplier+r][y*sizeMultiplier+s].tile2.node = kNoGraphNode;
				}
		}
			fscanf(f, "\n");
	}
}

void Map::loadOctileCorner(FILE *f, int high, int wide)
{
	mapType = kOctileCorner;
	width--;
	height--;
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	drawLand = true;
//	dList = 0;
	updated = true;
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			int h;
			char hc;
			fscanf(f, "%c", &hc);
			h = hc-'0'-2;
			if ((x > 0) && (y < high-1))
				SetCornerHeight(x-1, y, kTopRight, h);
			if ((x < wide-1) && (y > 0))
				SetCornerHeight(x, y-1, kBottomLeft, h);
			if ((x > 0) && (y > 0))
				SetCornerHeight(x-1, y-1, kBottomRight, h);
			if ((x < wide-1) && (y < high-1))
				SetCornerHeight(x, y, kTopLeft, h);
		}
		fscanf(f, "\n");
	}
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			char what;
			fscanf(f, "%c", &what);
			if ((y == high-1) || (x == wide-1))
				continue;
			if (!islower(what))
				SetHeight(x, y, GetCornerHeight(x, y, kTopLeft));
			switch (toupper(what))
			{
				case 'O':
					SetTerrainType(x, y, kOutOfBounds); break;
				case 'S':
					SetTerrainType(x, y, kSwamp); break;
				case 'W':
					SetTerrainType(x, y, kWater); break;
				case 'T':
					SetTerrainType(x, y, kTrees); break;
				default:
					SetTerrainType(x, y, kGround); break;
			}
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
		fscanf(f, "\n");
	}
}

#include <vector>

struct header 
{
     	uint32_t magic;
	uint32_t version;
	uint32_t platform;
	uint32_t filetype;
	uint32_t fileversion;
	uint32_t structcount;
	uint32_t dataoffset;
};

struct structArray
{
	uint32_t id;
	uint32_t fieldCnt;
	uint32_t fieldOffset;
	uint32_t structSize;
};

struct fieldData
{
	uint32_t label;
	uint32_t fieldType;
	uint32_t index;
};

struct fullData
{
	structArray sa;
	std::vector<fieldData> fields;
};

bool Map::tryDragonAge(FILE *f)
{
	rewind(f);
	header h;
	fread(&h, sizeof(header), 1, f);
	printf("header: %ld, struct array: %ld, fieldData %ld\n", sizeof(header), sizeof(structArray), sizeof(fieldData));
	printf("MGC: %c%c%c%c\n", h.magic&0xFF, (h.magic>>8)&0xFF,
		   (h.magic>>16)&0xFF, (h.magic>>24)&0xFF);
	printf("GFF: %c%c%c%c\n", h.version&0xFF, (h.version>>8)&0xFF,
		   (h.version>>16)&0xFF, (h.version>>24)&0xFF);
	printf("TGT: %c%c%c%c\n", h.platform&0xFF, (h.platform>>8)&0xFF,
		   (h.platform>>16)&0xFF, (h.platform>>24)&0xFF);
	printf("TYP: %c%c%c%c\n", h.filetype&0xFF, (h.filetype>>8)&0xFF,
		   (h.filetype>>16)&0xFF, (h.filetype>>24)&0xFF);
	printf("FVR: %c%c%c%c\n", h.fileversion&0xFF, (h.fileversion>>8)&0xFF,
		   (h.fileversion>>16)&0xFF, (h.fileversion>>24)&0xFF);
	printf("STRUCT: %d\n", h.structcount);
	printf("DATA: 0x%X\n", h.dataoffset);
	
	if (('G' != ((char)(h.magic&0xFF))) ||
		('F' != ((char)(h.magic>>8)&0xFF)) ||
		('F' != ((char)(h.magic>>16)&0xFF)) ||
		(' ' != ((char)(h.magic>>24)&0xFF)))
	{
		if ('G' != ((char)(h.magic&0xFF))) printf("bad g\n");
		if ('F' != ((char)(h.magic>>8)&0xFF)) printf("bad f1\n");
		if ('F' != ((char)(h.magic>>16)&0xFF)) printf("bad f2\n");
		if (' ' != ((char)(h.magic>>24)&0xFF)) printf("bad <space>\n");
		printf("DAO: bad magic\n");
		return false;
	}
	if (('V' != ((char)h.version&0xFF)) ||
		('4' != ((char)(h.version>>8)&0xFF)) ||
		('.' != ((char)(h.version>>16)&0xFF)) ||
		('0' != ((char)(h.version>>24)&0xFF)))
	{
		printf("DAO: bad version\n");
		return false;
	}
	if (('P' != ((char)h.platform&0xFF)) ||
		('C' != ((char)(h.platform>>8)&0xFF)) ||
		(' ' != ((char)(h.platform>>16)&0xFF)) ||
		(' ' != ((char)(h.platform>>24)&0xFF)))
	{
		printf("DAO: bad platform\n");
		return false;
	}
	if (('A' != ((char)h.filetype&0xFF)) ||
		('R' != ((char)(h.filetype>>8)&0xFF)) ||
		('L' != ((char)(h.filetype>>16)&0xFF)) ||
		(' ' != ((char)(h.filetype>>24)&0xFF)))
	{
		printf("DAO: bad file type\n");
		return false;
	}
	if (('V' != ((char)h.fileversion&0xFF)) ||
		('4' != ((char)(h.fileversion>>8)&0xFF)) ||
		('.' != ((char)(h.fileversion>>16)&0xFF)) ||
		('0' != ((char)(h.fileversion>>24)&0xFF)))
	{
		printf("DAO: bad file version\n");
		return false;
	}
	
	std::vector<fullData> fd;
	fd.resize(h.structcount);
	//structArray sa
	for (unsigned int x = 0; x < h.structcount; x++)
	{
		fread(&fd[x].sa, sizeof(fd[x].sa), 1, f);
		//		printf("Struct %d of %d\n", x, h.structcount-1);
		//		printf("ID: %c%c%c%c\n", fd[x].sa.id&0xFF, (fd[x].sa.id>>8)&0xFF,
		//			   (fd[x].sa.id>>16)&0xFF, (fd[x].sa.id>>24)&0xFF);
		//		printf("%d fields at 0x%X offset size %d\n",
		//			   fd[x].sa.fieldCnt, fd[x].sa.fieldOffset, fd[x].sa.structSize);
	}
	
	for (unsigned int x = 0; x < h.structcount; x++)
	{
		fd[x].fields.resize(fd[x].sa.fieldCnt);
		fread(&fd[x].fields[0], sizeof(fieldData), fd[x].sa.fieldCnt, f);
	}
	height = width = -1;
	for (unsigned int x = 0; x < h.structcount; x++)
	{
		printf("Struct %d of %d ", x, h.structcount-1);
		printf("ID: %c%c%c%c\n", fd[x].sa.id&0xFF, (fd[x].sa.id>>8)&0xFF,
			   (fd[x].sa.id>>16)&0xFF, (fd[x].sa.id>>24)&0xFF);
		for (unsigned int y = 0; y < fd[x].sa.fieldCnt; y++)
		{
			printf("Field %d of %d\n", y, fd[x].sa.fieldCnt-1);
			printf("LABEL: %d\n TYPE: %d", fd[x].fields[y].label, fd[x].fields[y].fieldType);
			printf("(%s %s %s)\n", fd[x].fields[y].fieldType&0x80?"list":"", fd[x].fields[y].fieldType&0x40?"struct":"", fd[x].fields[y].fieldType&0x20?"reference":"");
			printf("AT: 0x%X (0x%X)\n", fd[x].fields[y].index, fd[x].fields[y].index+h.dataoffset);
			fseek(f, fd[x].fields[y].index+h.dataoffset+0x10, SEEK_SET);
			if (fd[x].fields[y].label == 3127)
			{
				char str[8];
				fread(&str, sizeof(char), 8, f);
				str[7] = 0;
				printf("name: %s\n", str);
			}
			else if ((fd[x].fields[y].label == 3086) ||
					 (fd[x].fields[y].label == 3087) ||
					 //				(fd[x].fields[y].label == 3380) ||
					 //				(fd[x].fields[y].label == 3381) ||
					 (fd[x].fields[y].label == 3092))
			{
				uint32_t loc;
				fread(&loc, sizeof(loc), 1, f);
				//				printf("VALUE: %d\n", loc);
				if (fd[x].fields[y].label == 3086)
				{ width = loc; printf("**Width: %d\n", width); }
				else if (fd[x].fields[y].label == 3087)
				{ height = loc; printf("**Height: %d\n", height); }
				else if (fd[x].fields[y].label == 3381)
				{ width = loc; printf("**Width: %d\n", width); }
				else if (fd[x].fields[y].label == 3380)
				{ height = loc; printf("**Height: %d\n", height); }
				else if (fd[x].fields[y].label == 3092)
				{
					if ((height == -1) || (width == -1))
						printf("Read data before height/width\n");
					if ((height == 0) || (width == 0))
					{
						printf("No height/width data\n");
						height = 500;
						width = 500;
					}
					mapType = kOctile;
					land = new Tile *[width];
					for (int x = 0; x < width; x++) land[x] = new Tile [height];
					drawLand = true;
//					dList = 0;
					updated = true;
					
					fseek(f, h.dataoffset+loc, SEEK_SET);
					fread(&loc, sizeof(loc), 1, f);
					//					printf("DATA SIZE: %d\n", loc);
					for (unsigned int z = 0; z < loc; z++)
					{
						uint8_t next;
						fread(&next, sizeof(next), 1, f);
						//						printf("%2x ", next);
						if (next < 0x9)
							SetTerrainType(z%width, z/width, kGround);
						else if (next < 0xa)
							SetTerrainType(z%width, z/width, kTrees);
						else
							SetTerrainType(z%width, z/width, kOutOfBounds);
						
					}
					//					printf("\n");
				}
			}
			else {
				if (fd[x].fields[y].fieldType == 5)
				{
					uint32_t loc;
					fread(&loc, sizeof(loc), 1, f);
					printf("**VALUE: %d\n", loc);
				}
				if (fd[x].fields[y].fieldType == 8)
				{
					float loc;
					fread(&loc, sizeof(loc), 1, f);
					printf("**VALUE: %f\n", loc);
				}
			}
		}
	}
	return true;
}

// check to see if we can load it as a rolling stone puzzle
bool Map::tryLoadRollingStone(FILE *f)
{
	// preprocess - all chars must be ' ', '\n', '*', '$', '#', '@' or '.'
	// also calculate map dimensions
	rewind(f);
	int nrows = 0;
	int ncols = 0;
	while (1) // get columns
	{
		int currCol = 1;
		char nextc = fgetc(f);
		if (feof(f))
			break;
		if (!isLegalStone(nextc))
		{
			printf("illegal stone on row %d, (column pos %d)\n", nrows, currCol);
			return false;
		}
		nrows++;
		while ((nextc = fgetc(f)) != '\n') // get this row
		{
			if (feof(f))
			{
				break;
			}
			if (!isLegalStone(nextc))
			{
				printf("illegal stone on row %d, column pos %d\n", nrows, currCol);
				return false;
			}
			currCol++;
		}
		ncols = max(currCol, ncols);
	}
	mapType = kSokoban;
	printf("Dimensions (%d, %d)\n", ncols, nrows);
	width = ncols;
	height = nrows;
	
	rewind(f);
	int manx=0, many=0;
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [width];
	drawLand = true;
//	dList = 0;
	updated = true;
	for (int y = 0; y < height; y++)
	{
		bool foundEOL = false;
		for (int x = 0; x < width; x++)
		{
			char what;
			what = fgetc(f);
			//fscanf(f, "%c", &what);
			switch (toupper(what))
			{
				case ' ':
					SetTerrainType(x, y, kOutOfBounds); printf(" "); break;
				case '#':
					SetTerrainType(x, y, kWater); printf("#"); break;
				case '$':
					SetTerrainType(x, y, kSwamp); printf("$"); break;
				case '*':
					SetTerrainType(x, y, kBlight); printf("*"); break;
				case '@':
					manx = x; many = y;
					SetTerrainType(x, y, kGrass); printf("@"); break;
				case '.':
					SetTerrainType(x, y, kBlight); printf("."); break;
				case '\n':
					foundEOL = true;
					for (; x < width; x++)
					{
						SetTerrainType(x, y, kOutOfBounds);
						printf("_"); 
						land[x][y].tile1.node = kNoGraphNode;
						land[x][y].tile2.node = kNoGraphNode;
					}
						x--;
					break;
			}
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
		if (!foundEOL)
			fgetc(f);
		printf("\n");
	}
	// from man location, paint inside of room
	paintRoomInside(manx, many);
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			land[x][y].tile1.node = kNoGraphNode;
			if (GetTerrainType(x, y) == kWater)
				SetTerrainType(x, y, kOutOfBounds);
			SetHeight(x,y,0);
			SetSplit(x,y,kNoSplit);
		}
	}
	return true;
}

void Map::paintRoomInside(int x, int y)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return;
	if (land[x][y].tile1.node != kNoGraphNode)
		return;
	if (GetTerrainType(x, y) == kWater)
		return;
	if (GetTerrainType(x, y) == kOutOfBounds)
		SetTerrainType(x, y, kGround);
	land[x][y].tile1.node = kNoGraphNode+1;
	paintRoomInside(x+1, y);
	paintRoomInside(x-1, y);
	paintRoomInside(x, y+1);
	paintRoomInside(x, y-1);
}

bool Map::isLegalStone(char c)
{
	const int numLegals = 7;
	char legals[numLegals] = {' ', '\n', '$', '#', '@', '.', '*'};
	for (int x = 0; x < numLegals; x++)
		if (c == legals[x])
			return true;
	printf("Found illegal stone: \'%c\'\n", c);
	return false;
}

/** 
* unimplemented.
*/
void Map::Save(std::stringstream &/*data*/) {}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map::Save(const char *filename)
{
	FILE *f = fopen(filename, "w+");
	if (f)
	{
		Save(f);
		fclose(f);
	}
	else {
		printf("Error! Couldn't open file to save\n");
	}
}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map::Save(FILE *f)
{
	switch(mapType)
	{
		case kOctile:
			saveOctile(f);
			break;
		case kSokoban:
		case kOctileCorner:
			printf("Unable to save to identical map type\n");
		case kRaw:
		default:
			saveRaw(f);
			break;
	}
}

void Map::saveOctile(FILE *f)
{
	if (f)
	{
		fprintf(f, "type octile\nheight %d\nwidth %d\nmap\n", height, width);
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				switch (GetTerrainType(x, y))
				{
					case kGround: fprintf(f, "."); break;
					case kSwamp: fprintf(f, "S"); break;
					case kWater: fprintf(f, "W"); break;
					case kTrees: fprintf(f, "T"); break;
					default: fprintf(f, "@"); break; // out of bounds
				}
			}
			fprintf(f, "\n");
		}
	}
}

void Map::saveRaw(FILE *f)
{
	if (f)
	{
		fprintf(f, "type raw\nheight %d\nwidth %d\nmap\n", height, width);
		for (int x = 0; x < width; x++)
		{
			fwrite(&land[x][0], sizeof(Tile), width, f);
		}
	}
}


/** 
* Do an ASCII/ANSI print out of the map.
*
* Moves the cursor to the top of the screen and draws an ASCII version of
* the map. The map is shrunk by a factor of 2 in the vertical scale, but
* left normal in the horizontal scale. For the moment we leave ground blank
* and draw the walls as x/X/^. No other ground type is drawn.
*/
void Map::Print(int _scale)
{
	//  printf("%c[%d;%dmHeight\n", 27, 4, 30);
	//  printf("%c[%d;%dm", 27, 0, 0);
	//  for (int x = 0; x < 5; x++)
	//    printf("%c[%d;%dm%d\n", 27, 1, 31+x, x);
	//  printf("\n");
	// clear the screen
//	printf("%c[H", 27);
	for (int y = 0; y < height; y+=2*_scale)
	{
		for (int x = 0; x < width; x+=_scale)
		{
			//      if (land[x][y].split == kForwardSplit) {
			//				printf("%c[%d;%dm", 27, 0, 0);
			//				printf("/");
			//      } else if (land[x][y].split == kBackwardSplit) {
			//				printf("%c[%d;%dm", 27, 0, 0);
			//				printf("\\");
			//      } else {
			//				printf("%c[%d;%ldm", 27, 1, 31+land[x][y].tile1.corners[0]); }
			switch (land[x][y].tile1.type) {
				case kOutOfBounds:
					if (y+1 >= height)
					{ printf("^"); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("X"); break;
						case kGround: case kSwamp: printf("^"); break;
						default: printf("#"); break;
					}
					break;
					//					case kUndefined: printf("?"); break;
					//					case kWater: printf("%%"); break;
					//					case kSwamp: printf("@"); break;
				case kGround:
					if (y+1 >= height)
					{ printf(" "); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("x"); break;
						case kGround: printf(" "); break;
						default: printf("#"); break;
					}
					break;
				case kSwamp:
					if (y+1 >= height)
					{ printf(","); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("x"); break;
						case kGround: printf("."); break;
						case kSwamp: printf(":"); break;
						default: printf("#"); break;
					}
					break;
				default:
					printf("?");
					break;
			}
		}
		printf("\n");
	}
//	printf("%c[%d;%dm", 27, 0, 0);
	printf("\n");
}

const char *Map::GetMapName()
{
	if (map_name[0] == 0)
		return "";
	return map_name;
}

/** 
* Return the tile at location x, y.
*
* returns a reference to the type at a particular x/y location. (starting from 0)
*/

Tile &Map::GetTile(long x, long y)
{
	return land[x][y];
}

/** 
* Return the split of the tile at x, y.
*
* Returns the type of split; either kNoSplit, kForwardSplit, kBackwardSplit
*/
tSplit Map::GetSplit(long x, long y) const
{
	return land[x][y].split;
}

/** 
* Set the split of the tile at x, y.
*
* Sets how a map is split; either kNoSplit, kForwardSplit, kBackwardSplit
*/
void Map::SetSplit(long x, long y, tSplit split)
{
	revision++;
	land[x][y].split = split;
}


/** 
* Get the terrain type of the (split) tile at x, y.
*
* Gets the terrain type for this tile. By default it looks for the value
* of the whole tile. Possible split values are kWholeTile, kLeftSide, and
* kRightSide. 
*/
long Map::GetTerrainType(long x, long y, tSplitSide split) const
{
	if (land[0] == 0)
		printf("land: %p, land[0] = %p\n", land, land[0]);
	if ((x < 0) || (x >= width) || (y < 0) || (y >= height)) return kUndefined;
	if (split == kRightSide) return land[x][y].tile2.type;
	return land[x][y].tile1.type;
}

/**
 * Map::SetTerrainType()
 *
 * \brief Set all the terrain between two points to be the same
 *
 * \param x1 The first x-coordinate to set
 * \param y1 The first y-coordinate to set
 * \param x2 The second x-coordinate to set
 * \param y2 The second y-coordinate to set
 * \param terrain The terrain for the line between the coordinates
 * \return none
 */
void Map::SetTerrainType(int32_t x1, int32_t y1,
                         int32_t x2, int32_t y2, tTerrain t)
{
    updated = true;
    //printf("---> (%d, %d) to (%d, %d)\n", x1, y1, x2, y2);
    double xdiff = (int)x1-(int)x2;
    double ydiff = (int)y1-(int)y2;
    double dist = sqrt(xdiff*xdiff + ydiff*ydiff);
    SetTerrainType(x1, y1, t);
    for (double c = 0; c < dist; c += 0.5)
    {
        double ratio = c/dist;
        double newx = (double)x1-xdiff*ratio;
        double newy = (double)y1-ydiff*ratio;
        SetTerrainType((uint32_t)newx, (uint32_t)newy, t);
    }
    SetTerrainType(x2, y2, t);
}


/**
* Get the terrain type for one side of the tile at x, y.
 *
 * Gets the terrain type for a particular edge of the type. 
 * (kLeftEdge, kRightEdge, kTopEdge, kBottom Edge)
 * This function avoids making you figure out all the ways a tile could
 * be split to get the correct value out.
 */
long Map::GetTerrainType(long x, long y, tEdge side) const
{
	if ((x < 0) || (x >= width) || (y < 0) || (y >= height)) return kUndefined;
	if (land[x][y].split == kNoSplit) return land[x][y].tile1.type;
	switch (side) {
		case kLeftEdge:
			return land[x][y].tile1.type;
		case kRightEdge:
			return land[x][y].tile2.type;
		case kTopEdge:
			if (land[x][y].split == kForwardSplit)
				return land[x][y].tile1.type;
			else
				return land[x][y].tile2.type;
		case kBottomEdge:
			if (land[x][y].split == kBackwardSplit)
				return land[x][y].tile1.type;
			else
				return land[x][y].tile2.type;
		default:
			return kUndefined;
	}
}

/** Set the terrain type of the side of the tile at x, y.
*
* side is one of kWholeTile, kLeftSide or kRightSide
* If tile is not split and you specify a split side, nothing happens
* If tile is split and you specify kWholeTile, the split remains,
* and the terrain is applied to both sides.
*/
void Map::SetTerrainType(long x, long y, tTerrain type, tSplitSide split)
{
	if ((x >= width)||(x<0)) return;
	if ((y >= height)||(x<0)) return;
	revision++;
	updated = true;
	map_name[0] = 0;
	if ((land[x][y].split == kNoSplit) && (split != kWholeTile)) return;
	if ((y > GetMapHeight()-1) || (x > GetMapWidth()-1))
		return;
	switch (split) {
		
		case kWholeTile:
			land[x][y].tile1.type = type;
			land[x][y].tile2.type = type;
			break;
			
		case kLeftSide:
			land[x][y].tile1.type = type;
			break;
			
		case kRightSide:
			land[x][y].tile2.type = type;
			break;
	}
}

/** 
* Get the (flat) height of the tile at x, y.
*
* returns the height of a particular tile -- actually just one corner of the
* tile. If the tile is sloping you'll get back kUndefined and need to get
* the specific corner heights.
* returns kUndefinedHeight if the tile is split and you specify
* the whole tile
*/
long Map::GetHeight(long x, long y, tSplitSide split)
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile)) return kUndefinedHeight;
	
	switch (split) {
		
		case kWholeTile:
			
		case kLeftSide:
			if ((land[x][y].tile1.corners[0] == land[x][y].tile1.corners[1]) &&
					(land[x][y].tile1.corners[1] == land[x][y].tile1.corners[2]))
				return land[x][y].tile1.corners[0];
			return kUndefinedHeight;
			break;
			
		case kRightSide:
			if ((land[x][y].tile2.corners[0] == land[x][y].tile2.corners[1]) &&
					(land[x][y].tile2.corners[1] == land[x][y].tile2.corners[2]))
				return land[x][y].tile2.corners[0];
			break;
	}
	return kUndefinedHeight;
}

/** 
* Set the (flat) height of the tile at x, y.
* 
* Split is kWholeTile, kLeftSide or kRightSide.
*/
void Map::SetHeight(long x, long y, long tHeight, tSplitSide split)
{
	revision++;
	switch (split) {
		
		case kWholeTile:
			land[x][y].tile1.corners[0] = land[x][y].tile1.corners[1] =
			land[x][y].tile1.corners[2] = tHeight;
			land[x][y].tile2.corners[0] = tHeight;
			land[x][y].tile2.corners[1] = tHeight;
			land[x][y].tile2.corners[2] = tHeight;
			break;
		case kLeftSide:
			land[x][y].tile1.corners[0] = land[x][y].tile1.corners[1] =
			land[x][y].tile1.corners[2] = tHeight;
			break;
		case kRightSide:
			land[x][y].tile2.corners[0] = land[x][y].tile2.corners[1] =
			land[x][y].tile2.corners[2] = tHeight;
			break;
	}
}

/**
* Set the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * edge is kBottomEdge, kLeftEdge, kRightEdge, kTopEdge
 * returns kUndefinedHeight if the split is inconsistant with the tile type
 * The combination of a corner and an edge uniquely define a single height
 */
long Map::GetCornerHeight(long x, long y, tCorner which, tEdge edge) const
{
	if (GetSplit(x, y) == kNoSplit)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile1.corners[2];
			default: break;
		}
	}
	else {
		if (edge == kLeftEdge)
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile1.corners[0];
				case kBottomLeft: return land[x][y].tile1.corners[1];
				default: break;
			}
		}
		else if (edge == kRightEdge)
		{
			switch (which) {
				case kTopRight: return land[x][y].tile2.corners[0];
				case kBottomRight: return land[x][y].tile2.corners[1];
				default: break;
			}
		}
		else if ((edge == kTopEdge) && (GetSplit(x, y) == kForwardSplit))
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile1.corners[0];
				case kTopRight: return land[x][y].tile1.corners[2];
				default: break;
			}
		}
		else if ((edge == kTopEdge) && (GetSplit(x, y) == kBackwardSplit))
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile2.corners[2];
				case kTopRight: return land[x][y].tile2.corners[0];
				default: break;
			}
		}
		else if ((edge == kBottomEdge) && (GetSplit(x, y) == kForwardSplit))
		{
			switch (which) {
				case kBottomLeft: return land[x][y].tile2.corners[2];
				case kBottomRight: return land[x][y].tile2.corners[1];
				default: break;
			}
		}
		else if ((edge == kBottomEdge) && (GetSplit(x, y) == kBackwardSplit))
		{
			switch (which) {
				case kBottomLeft: return land[x][y].tile1.corners[1];
				case kBottomRight: return land[x][y].tile1.corners[2];
				default: break;
			}
		}
	}
	// should never get here...
	return kUndefinedHeight;
}

/**
* Get the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * split is kLeftSide, kRightSide or kWholeTile
 * returns kUndefinedHeight if the split is inconsistant with the tile type
 * The combination of a corner and a split side uniquely define a single height
 */
long Map::GetCornerHeight(long x, long y, tCorner which, tSplitSide split) const
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile))
		return kUndefinedHeight;
	if (split == kWholeTile)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile1.corners[2];
			default: break;
		}
	}
	else if (split == kLeftSide)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight:
				if (land[x][y].split == kForwardSplit)
					return land[x][y].tile1.corners[2];
				return kUndefinedHeight;
			case kBottomRight:
				if (land[x][y].split == kBackwardSplit)
					return land[x][y].tile1.corners[2];
				return kUndefinedHeight;
			default: break;
		}
	}
	else if (split == kRightSide)
	{
		switch (which) {
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile2.corners[1];
			case kTopLeft:
				if (land[x][y].split == kBackwardSplit)
					return land[x][y].tile2.corners[2];
				return kUndefinedHeight;
			case kBottomLeft:
				if (land[x][y].split == kForwardSplit)
					return land[x][y].tile2.corners[2];
				return kUndefinedHeight;
			default: break;
		}
	}
	return kUndefinedHeight;
}

/**
* Set the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * split is kLeftSide, kRightSide or kWholeTile
 * The combination of a corner and a split side uniquely define a single height,
 * which is returned.
 */
void Map::SetCornerHeight(long x, long y, tCorner which,
													long cHeight, tSplitSide split)
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile))
		return;
	revision++;
	if (split == kWholeTile)
	{
		switch (which) {
			case kTopLeft: land[x][y].tile1.corners[0] = cHeight; break;
			case kBottomLeft: land[x][y].tile1.corners[1] = cHeight; break;
			case kTopRight: land[x][y].tile2.corners[0] = cHeight; break;
			case kBottomRight: land[x][y].tile1.corners[2] = cHeight; 
				land[x][y].tile2.corners[1] = cHeight;break;
			default: break;
		}
	}
	else if (split == kLeftSide)
	{
		switch (which) {
			case kTopLeft: land[x][y].tile1.corners[0] = cHeight; break;
			case kBottomLeft: land[x][y].tile1.corners[1] = cHeight; break;
			case kTopRight:
				if (land[x][y].split == kForwardSplit)
					land[x][y].tile1.corners[2] = cHeight;
				break;
			case kBottomRight:
				if (land[x][y].split == kBackwardSplit)
					land[x][y].tile1.corners[2] = cHeight;
				break;
			default: break;
		}
	}
	else if (split == kRightSide)
	{
		switch (which) {
			case kTopRight: land[x][y].tile2.corners[0] = cHeight; break;
			case kBottomRight: land[x][y].tile2.corners[1] = cHeight; break;
			case kTopLeft:
				if (land[x][y].split == kBackwardSplit)
					land[x][y].tile2.corners[2] = cHeight;
				break;
			case kBottomLeft:
				if (land[x][y].split == kForwardSplit)
					land[x][y].tile2.corners[2] = cHeight;
				break;
			default: break;
		}
	}
}


/**
* Places a rectangle into the map, but also modifies the edges to make the
 * transition smooth.
 *
 * sets a rectangle of with corner coordinates (x1, y1) (x2, y2)
 * but also takes the 1-radius tiles surrounding that rectangle and
 * smooths them so you get a nice fit of land together.
 */
void Map::SmoothSetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type)
{
	updated = true;
	map_name[0] = 0;
	if (x1 > x2)
	{
		SmoothSetRectHeight(x2, y1, x1, y2, h, type);
		return;
	}
	else if (y1 > y2)
	{
		SmoothSetRectHeight(x1, y2, x2, y1, h, type);
		return;
	}
	printf("Doing smooth rect between (%ld, %ld) and (%ld, %ld) height %ld\n", x1, y1, x2, y2, h);
	SetRectHeight(x1, y1, x2, y2, h, type);
	
	// top side
	for (int x = x1; x <= x2; x++)
	{
		SetTerrainType(x, y1-1, type);
		switch(GetSplit(x, y1-1)) {
			case kNoSplit:
				if (GetCornerHeight(x, y1-1, kTopLeft) != GetCornerHeight(x, y1-1, kTopRight))
				{ // need to split slanted tile
					SetSplit(x, y1-1, kForwardSplit); // split is arbitarary?
																						//SetCornerHeight(x, y1-1, kBottomLeft, GetCornerHeight(x, y1-1, kBottomLeft, kLeftSide), kRightSide);
					
					SetCornerHeight(x, y1-1, kBottomLeft, h, kRightSide);
					SetCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
					SetCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				}
				else {
					SetCornerHeight(x, y1-1, kBottomLeft, h);
					SetCornerHeight(x, y1-1, kBottomRight, h);
				}
				break;
			case kForwardSplit:
				SetCornerHeight(x, y1-1, kBottomLeft, h, kRightSide);
				SetCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
				SetCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				break;
			case kBackwardSplit:
				SetCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				SetCornerHeight(x, y1-1, kBottomRight, h, kLeftSide);
				SetCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
				break;
		}
	}
	
	// bottom side
	for (int x = x1; x <= x2; x++)
	{
		SetTerrainType(x, y2+1, type);
		switch(GetSplit(x, y2+1)) {
			case kNoSplit:
				if (GetCornerHeight(x, y2+1, kBottomLeft) != GetCornerHeight(x, y2+1, kBottomRight))
				{ // need to split slanted tile
					SetSplit(x, y2+1, kBackwardSplit); // split is arbitarary?
																						 //SetCornerHeight(x, y2+1, kTopLeft, GetCornerHeight(x, y2+1, kTopLeft, kLeftSide), kRightSide);
					
					SetCornerHeight(x, y2+1, kTopLeft, h, kRightSide);
					SetCornerHeight(x, y2+1, kTopRight, h, kRightSide);
					SetCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				}
				else {
					SetCornerHeight(x, y2+1, kTopLeft, h);
					SetCornerHeight(x, y2+1, kTopRight, h);
				}
				break;
			case kBackwardSplit:
				SetCornerHeight(x, y2+1, kTopLeft, h, kRightSide);
				SetCornerHeight(x, y2+1, kTopRight, h, kRightSide);
				SetCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				break;
			case kForwardSplit:
				SetCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				SetCornerHeight(x, y2+1, kTopRight, h, kLeftSide);
				SetCornerHeight(x, y2+1, kTopRight, h, kRightSide);
				break;
		}
	}
	
	// left side
	for (int y = y1; y <= y2; y++)
	{
		SetTerrainType(x1-1, y, type);
		switch(GetSplit(x1-1, y)) {
			case kNoSplit:
				if (GetCornerHeight(x1-1, y, kTopLeft) != GetCornerHeight(x1-1, y, kBottomLeft))
				{ // need to split slanted tile
					SetSplit(x1-1, y, kBackwardSplit); // split is arbitarary?
					SetCornerHeight(x1-1, y, kBottomRight, GetCornerHeight(x1-1, y, kTopRight, kLeftSide), kRightSide);
					SetCornerHeight(x1-1, y, kTopLeft, GetCornerHeight(x1-1, y, kTopLeft, kLeftSide), kRightSide);
					
					SetCornerHeight(x1-1, y, kBottomRight, h, kLeftSide);
					SetCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
					SetCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				}
				else {
					SetCornerHeight(x1-1, y, kTopRight, h);
					SetCornerHeight(x1-1, y, kBottomRight, h);
				}
				break;
			case kBackwardSplit:
				SetCornerHeight(x1-1, y, kBottomRight, h, kLeftSide);
				SetCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
				SetCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				//SetCornerHeight(x1-1, y, kTopLeft, GetCornerHeight(x1-1, y, kTopLeft, kLeftSide), kRightSide);
				break;
			case kForwardSplit:
				SetCornerHeight(x1-1, y, kTopRight, h, kLeftSide);
				SetCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
				SetCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				break;
		}
	}
	
	// right side
	for (int y = y1; y <= y2; y++)
	{
		SetTerrainType(x2+1, y, type);
		switch(GetSplit(x2+1, y)) {
			case kNoSplit:
				if (GetCornerHeight(x2+1, y, kTopLeft) != GetCornerHeight(x2+1, y, kBottomLeft))
				{ // need to split slanted tile
					SetSplit(x2+1, y, kForwardSplit); // split is arbitarary?
					SetCornerHeight(x2+1, y, kTopRight, GetCornerHeight(x2+1, y, kTopRight, kRightSide), kLeftSide);
					
					SetCornerHeight(x2+1, y, kBottomLeft, h, kRightSide);
					SetCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
					SetCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				}
				else {
					SetCornerHeight(x2+1, y, kTopLeft, h);
					SetCornerHeight(x2+1, y, kBottomLeft, h);
				}
				break;
			case kBackwardSplit:
				SetCornerHeight(x2+1, y, kTopLeft, h, kRightSide);
				SetCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
				SetCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				break;
			case kForwardSplit:
				SetCornerHeight(x2+1, y, kBottomLeft, h, kRightSide);
				SetCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
				SetCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				break;
		}
	}
	
	
	SetSplit(x1-1, y1-1, kForwardSplit);
	SetTerrainType(x1-1, y1-1, type, kRightSide);
	SetCornerHeight(x1-1, y1-1, kBottomRight, h, kRightSide);
	SetCornerHeight(x1-1, y1-1, kBottomLeft, GetCornerHeight(x1-1, y1-1, kBottomLeft, kLeftSide), kRightSide);
	SetCornerHeight(x1-1, y1-1, kTopRight, GetCornerHeight(x1-1, y1-1, kTopRight, kRightSide), kLeftSide);
	
	SetSplit(x2+1, y2+1, kForwardSplit);
	SetTerrainType(x2+1, y2+1, type, kLeftSide);
	SetCornerHeight(x2+1, y2+1, kTopLeft, h, kLeftSide);
	SetCornerHeight(x2+1, y2+1, kBottomLeft, GetCornerHeight(x2+1, y2+1, kBottomLeft, kLeftSide), kRightSide);
	SetCornerHeight(x2+1, y2+1, kTopRight, GetCornerHeight(x2+1, y2+1, kTopRight, kRightSide), kLeftSide);
	
	SetSplit(x1-1, y2+1, kBackwardSplit);
	SetTerrainType(x1-1, y2+1, type, kRightSide);
	SetCornerHeight(x1-1, y2+1, kTopRight, h, kRightSide);
	SetCornerHeight(x1-1, y2+1, kTopLeft, GetCornerHeight(x1-1, y2+1, kTopLeft, kLeftSide), kRightSide);
	SetCornerHeight(x1-1, y2+1, kBottomRight, GetCornerHeight(x1-1, y2+1, kBottomRight, kRightSide), kLeftSide);
	
	SetSplit(x2+1, y1-1, kBackwardSplit);
	SetTerrainType(x2+1, y1-1, type, kLeftSide);
	SetCornerHeight(x2+1, y1-1, kBottomLeft, h, kLeftSide);
	SetCornerHeight(x2+1, y1-1, kTopLeft, GetCornerHeight(x2+1, y1-1, kTopLeft, kLeftSide), kRightSide);
	SetCornerHeight(x2+1, y1-1, kBottomRight, GetCornerHeight(x2+1, y1-1, kBottomRight, kRightSide), kLeftSide);
}

/**
* Set the height and terrain of a set of tiles.
 *
 * Sets all the tiles in the region between (x1, y1) (x2, y2) to be the same
 * height and terrain type, with no splits.
 */
void Map::SetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type)
{
	updated = true;
	map_name[0] = 0;
	revision++;
	//printf("Doing rect between (%ld, %ld) and (%ld, %ld) height %ld\n", x1, y1, x2, y2, h);
	for (int x = x1; x <= x2; x++)
	{
		for (int y = y1; y <= y2; y++)
		{
			SetSplit(x, y, kNoSplit);
			SetTerrainType(x, y, type);
			SetHeight(x, y, h);
		}
	}
}

/**
* Is the tile at x, y adjacent across the edge?
 *
 * given an edge (kInternalEdge, kLeftEdge, kRightEdge, kBottomEdge, kTopEdge)
 * returns whether the tiles on both sides of that edge have a smooth boundary
 * that a unit should be able to cross.
 */
bool Map::AdjacentEdges(long x, long y, tEdge edge) const
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return false;
	switch (edge) {
		case kInternalEdge:
		{
			tSplit split;
			if ((split = GetSplit(x, y)) == kNoSplit)
				return ((GetTerrainType(x, y, kLeftSide)>>terrainBits) == (GetTerrainType(x, y, kRightSide)>>terrainBits));
			else if (split == kForwardSplit)
			{
				return ((GetCornerHeight(x, y, kTopRight, kLeftSide) == GetCornerHeight(x, y, kTopRight, kRightSide)) &&
								(GetCornerHeight(x, y, kBottomLeft, kLeftSide) == GetCornerHeight(x, y, kBottomLeft, kRightSide)) &&
								((GetTerrainType(x, y, kLeftSide)>>terrainBits) == (GetTerrainType(x, y, kRightSide)>>terrainBits)));
			}
			else if (split == kBackwardSplit)
			{
				return ((GetCornerHeight(x, y, kTopLeft, kLeftSide) == GetCornerHeight(x, y, kTopLeft, kRightSide)) &&
								(GetCornerHeight(x, y, kBottomRight, kLeftSide) == GetCornerHeight(x, y, kBottomRight, kRightSide)) &&
								((GetTerrainType(x, y, kLeftSide)>>terrainBits) == (GetTerrainType(x, y, kRightSide)>>terrainBits)));
			}
			return false;
		} break;
		case kLeftEdge:
			if (x == 0)
				return false;
			return ((GetCornerHeight(x, y, kTopLeft, kLeftEdge) == GetCornerHeight(x-1, y, kTopRight, kRightEdge)) &&
							(GetCornerHeight(x, y, kBottomLeft, kLeftEdge) == GetCornerHeight(x-1, y, kBottomRight, kRightEdge)) &&
							((GetTerrainType(x, y, kLeftSide)>>terrainBits) == (GetTerrainType(x-1, y, kRightSide)>>terrainBits)));
			break;
		case kRightEdge:
			if (x+1 >= width)
				return false;
			return ((GetCornerHeight(x, y, kTopRight, kRightEdge) == GetCornerHeight(x+1, y, kTopLeft, kLeftEdge)) &&
							(GetCornerHeight(x, y, kBottomRight, kRightEdge) == GetCornerHeight(x+1, y, kBottomLeft, kLeftEdge)) &&
							((GetTerrainType(x, y, kRightSide)>>terrainBits) == (GetTerrainType(x+1, y, kLeftSide)>>terrainBits)));
			break;
		case kTopEdge:
			if (y == 0)
				return false;
			return ((GetCornerHeight(x, y, kTopRight, kTopEdge) == GetCornerHeight(x, y-1, kBottomRight, kBottomEdge)) &&
					(GetCornerHeight(x, y, kTopLeft, kTopEdge) == GetCornerHeight(x, y-1, kBottomLeft, kBottomEdge)) &&
					//(CanPass(GetTerrainType(x, y, kTopEdge), GetTerrainType(x, y-1, kBottomEdge))));
					((GetTerrainType(x, y, kTopEdge)>>terrainBits) == (GetTerrainType(x, y-1, kBottomEdge)>>terrainBits)));
			
			break;
		case kBottomEdge:
			if (y+1 >= height)
				return false;
			return ((GetCornerHeight(x, y, kBottomRight, kBottomEdge) == GetCornerHeight(x, y+1, kTopRight, kTopEdge)) &&
					(GetCornerHeight(x, y, kBottomLeft, kBottomEdge) == GetCornerHeight(x, y+1, kTopLeft, kTopEdge)) &&
					//(CanPass(GetTerrainType(x, y, kBottomEdge), GetTerrainType(x, y+1, kTopEdge))));
					((GetTerrainType(x, y, kBottomEdge)>>terrainBits) == (GetTerrainType(x, y+1, kTopEdge)>>terrainBits)));
			break;
	}
	return false;
}

bool Map::AdjacentCorners(long x, long y, tCorner corner) const
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return false;
	switch (corner)
	{
		case kNone:
			return true;
		case kTopLeft:
			if (((x >= 1) && (y >= 1) && (AdjacentEdges(x, y, kLeftEdge)) && (AdjacentEdges(x, y, kTopEdge)) &&
					 (AdjacentEdges(x, y-1, kLeftEdge)) && (AdjacentEdges(x-1, y, kTopEdge))) &&
					(((AdjacentEdges(x-1, y, kInternalEdge)) || (GetSplit(x-1, y) == kBackwardSplit)) &&
					 ((AdjacentEdges(x, y-1, kInternalEdge)) || (GetSplit(x, y-1) == kBackwardSplit)) &&
					 ((AdjacentEdges(x-1, y-1, kInternalEdge)) || (GetSplit(x-1, y-1) == kForwardSplit)) &&
					 ((AdjacentEdges(x, y, kInternalEdge)) || (GetSplit(x, y) == kForwardSplit))))
				return true;
			return false;
		case kTopRight:
			if (((y >= 1) && (x < GetMapWidth()-1) && (AdjacentEdges(x, y, kRightEdge)) && (AdjacentEdges(x, y, kTopEdge)) &&
					 (AdjacentEdges(x, y-1, kRightEdge)) && (AdjacentEdges(x+1, y, kTopEdge))) &&
					(((AdjacentEdges(x+1, y, kInternalEdge)) || (GetSplit(x+1, y) == kForwardSplit)) &&
					 ((AdjacentEdges(x, y-1, kInternalEdge)) || (GetSplit(x, y-1) == kForwardSplit)) &&
					 ((AdjacentEdges(x+1, y-1, kInternalEdge)) || (GetSplit(x+1, y-1) == kBackwardSplit)) &&
					 ((AdjacentEdges(x, y, kInternalEdge)) || (GetSplit(x, y) == kBackwardSplit))))
				return true;
			return false;
		case kBottomLeft: return AdjacentCorners(x-1, y+1, kTopRight);
		case kBottomRight: return AdjacentCorners(x+1, y+1, kTopLeft);
		default: return false;
	}
	return false;
}

bool Map::CanStep(long x1, long y1, long x2, long y2) const
{
	if ((abs(x1-x2) > 1) || (abs(y1-y2) > 1))
		return false;
	if (mapType == kOctile)
	{
		if (x1 == x2 || y1 == y2)
			return ((GetTerrainType(x1, y1)>>terrainBits) == (GetTerrainType(x2, y2)>>terrainBits));
		else
			return (((GetTerrainType(x1, y1)>>terrainBits) == (GetTerrainType(x2, y2)>>terrainBits)) &&
					((GetTerrainType(x1, y2)>>terrainBits) == (GetTerrainType(x2, y2)>>terrainBits)) &&
					((GetTerrainType(x2, y1)>>terrainBits) == (GetTerrainType(x2, y2)>>terrainBits)));
	}
	else {
		switch (x1-x2) {
			case 0: //return true;
				switch (y1-y2) {
					case 0: return true;
					case 1: return AdjacentEdges(x1, y1, kTopEdge);
					case -1: return AdjacentEdges(x1, y1, kBottomEdge);
				}
				break;
			case 1: //return AdjacentEdges(x1, y1, kLeftEdge);
				switch (y1-y2) {
					case 0: return AdjacentEdges(x1, y1, kLeftEdge);
					case 1: return AdjacentCorners(x1, y1, kTopLeft);
					case -1: return AdjacentCorners(x1, y1, kBottomLeft);
				}
				break;
			case -1: //return AdjacentEdges(x1, y1, kRightEdge);
				switch (y1-y2) {
					case 0: return AdjacentEdges(x1, y1, kRightEdge);
					case 1: return AdjacentCorners(x1, y1, kTopRight);
					case -1: return AdjacentCorners(x1, y1, kBottomRight);
				}
				break;
		}
	}
	return false;
}

/**
* Toggles whether the land is draw when you call OpenGLDraw
 */
void Map::SetDrawLand(bool dLand)
{
	drawLand = dLand;
}

/**
* Choose the tileset used for land colors. Tilesets named xxxTile will draw
 * the map as independant tiles as opposed to a smooth connected map.
 */
void Map::SetTileSet(tTileset ts)
{
	updated = true; // force the display list to re-draw
	tileSet = ts;
//	if (ts == kBitmap)
//		InitTextures();
}

/**
* Get the tileset used for land colors.
 */
tTileset Map::GetTileSet()
{
	return tileSet;
}


//void Map::setDrawAbstractions(int abstractions)
//{
//	numAbstractions = abstractions&0x7;
//}
	
/**
* Does actual OpenGL drawing of the map
 *
 * If drawLand has been set (on by default) the ground will be drawn using
 * the appropriate mode:   kPolygons, kLines, kPoints
 * kPolygon is the default mode. The map is cached in a display list unless
 * it changes.
 */
//void Map::OpenGLDraw(tDisplay how) const
//{
//	glDisable(GL_LIGHTING);
//	if (drawLand)
//	{
//		if (updated)
//		{
//			updated = false;
//			if (dList)
//				glDeleteLists(dList, 1);
//			dList = 0;
//		}
//		
//		if (dList)
//		{
//			glCallList(dList);
//		}
//		else {
//			if (verbose)
//				printf("Drawing land into display list\n");
//			dList = glGenLists(1);
//			glNewList(dList, GL_COMPILE_AND_EXECUTE);
//			
//			if (tileSet == kFast)
//			{
//				drawLandQuickly();
//			}
//			else {
//				for (int y = 0; y < height; y++)
//				{
//					for (int x = 0; x < width; x++)
//					{
//						DrawTile(&land[x][y], x, y, how);
//					}
//				}
//			}
//			glEndList();
//			// printf("Done\n");
//		}
//	}
//}

/**
* Get the openGL coordinates of a given tile.
 *
 * Given a tile in (x, y) coordinates, it returns the OpenGL space coordinates of
 * that tile along with the radius of the tile square. The map is drawn in the
 * x<->z plane, with the y plane up.
 */
bool Map::GetCoord(int _x, int _y, double &x, double &y, double &z, double &radius) const
{
	if (_x >= width) return false;
	if (_y >= height) return false;
	if ((_x == -1) || (_y == -1))
		return false;
	double _scale, xOffset, yOffset;
	if (height > width)
	{
		_scale = 2.0/(double)(height);
		xOffset = (2.0-GetMapWidth()*_scale)*0.5;
		yOffset = 0;
	}
	else {
		_scale = 2.0/(double)(width);
		yOffset = (2.0-GetMapHeight()*_scale)*0.5;
		xOffset = 0;
	}
	double epsilon = _scale/2.0;
	x = -1+_x*_scale+epsilon+xOffset;
	y = -1+_y*_scale+epsilon+yOffset;
//	x = (2*_x-width)*_scale+epsilon;
//	y = (2*_y-height)*_scale+epsilon;
	z = -(double)0.5*(land[_x][_y].tile1.corners[0]+land[_x][_y].tile2.corners[0])*(_scale);//+(double)land[_x][_y].tile1.corners[1]/(2*_scale));
	radius = epsilon;
	return true;
}

/**
 * Get the open coordinates of a given tile.
 *
 * Given a tile in (x, y) coordinates, it returns the coordinates of
 * that tile along with the radius of the tile square. The map is drawn in the
 * x<->z plane, with the y plane up.
 */
bool Map::GetCoord(float _x, float _y, double &x, double &y, double &z, double &radius) const
{
	if (isnan(_x) || isnan(_y))
	{
		x = y = z = 0;
		radius = 0;
		assert(false);
		return false;
	}
	int iX = floor(_x);
	int iY = floor(_y);
	return GetCoord(iX, iY, x, y, z, radius);
//	double _scale;
//	if (height > width)
//		_scale = 1/(double)(height+1);
//	else
//		_scale = 1/(double)(width+1);
//	double epsilon = _scale/2.0;
//	x = (2*_x-width)*_scale+epsilon;
//	y = (2*_y-height)*_scale+epsilon;
//	z = -(double)0.5*(land[iX][iY].tile1.corners[0]+land[iX][iY].tile2.corners[0])*(_scale);//+(double)land[_x][_y].tile1.corners[1]/(2*_scale));
//	radius = _scale;
//	return true;
}

/**
* Returns the scale multiplier between openGL coordinates and map coordinates.
 * If you measure a distance in openGL coordinate space, you can multiply it by
 * this value to convert it to map space, where the distance between adjacent tiles
 * is 1.
 */
double Map::GetCoordinateScale()
{
	//	double scale;
	if (height > width)
		return (double)(height)/2.0;
	return (double)(width)/2.0;
}

void Map::GetPointFromCoordinate(Graphics::point loc, int &px, int &py) const
{
	double _x, _y;
	double _scale, xOffset, yOffset;
	if (height > width)
	{
		_scale = 2.0/(double)(height);
		xOffset = (2.0-GetMapWidth()*_scale)*0.5;
		yOffset = 0;
	}
	else {
		_scale = 2.0/(double)(width);
		yOffset = (2.0-GetMapHeight()*_scale)*0.5;
		xOffset = 0;
	}
	double epsilon = _scale/2.0;

	_x = (loc.x-epsilon+1-xOffset)/_scale;
	_y = (loc.y-epsilon+1-yOffset)/_scale;

//	_x = ((loc.x-epsilon)/_scale+(double)width)/2.0;
//	_y = ((loc.y-epsilon)/_scale+(double)height)/2.0;
	px = (int)(_x+0.5); // round off!
	py = (int)(_y+0.5);
	/*
	 px = (int)(((loc.x/2.0)+0.5)*((double)width));
	 py = (int)(((loc.y/2.0)+0.5)*((double)height));
	 */
	if ((px < 0) || (py < 0) || (px >= width) || (py >= height))
	{
		px = py = -1;
	}
}

/**
* Draw a single tile.
 *
 * draws a single tile which is at (x, y) in the land
 * (technically we don't have to pass the tile, we could get it
		* from the x,y coordinates)
 */
//void Map::DrawTile(Tile *t, int x, int y, tDisplay how) const
//{
//	GLdouble xx, yy, zz, rr;
//	if (GetCoord(x,y,xx,yy,zz,rr) == false)
//		return;
//	
//	if (tileSet == kBitmap)
//	{
//		glNormal3d(0, 0, -1);
//		glColor3f(1, 1, 1);
//
//		float l, r, u, b;
//		switch (t->tile1.type)
//		{
//			case kTrees:
//				l = 0; r = 0.5;
//				u = 0.0; b = 0.5;
//				break;
//			case kGround:
//				l = 0; r = 0.5;
//				u = 0.5; b = 1.0;
//				break;
//			case kWater:
//				glColor3f(0.5, 0.5, 0.5);
//			case kSwamp:
//				l = 0.5; r = 1.0;
//				u = 0; b = 0.5;
//				break;
//
//			default:
//				l = 0.5; r = 1.0;
//				u = 0.5; b = 1.0;
//		}
//
//		glEnable(GL_TEXTURE_2D);
//		glBindTexture(GL_TEXTURE_2D, wall);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//
//		glBegin(GL_QUADS);
//
//		glTexCoord2f(l,b);
//		glVertex3f(xx-rr, yy-rr, /*-2*rr*/-rr*t->tile1.corners[0]);
//		glTexCoord2f(l,u);
//		glVertex3f(xx-rr, yy+rr, /*-2*rr*/-rr*t->tile1.corners[1]);
//		glTexCoord2f(r,u);
//		glVertex3f(xx+rr, yy+rr, /*-2*rr*/-rr*t->tile1.corners[2]);
//		glTexCoord2f(r,b);
//		glVertex3f(xx+rr, yy-rr, /*-2*rr*/-rr*t->tile2.corners[0]);
//
//		glEnd();
//		glDisable(GL_TEXTURE_2D);
//		return;
//	}
//	
//	switch (how) {
//		case kPolygons:
//			if (t->split == kNoSplit)
//				glBegin(GL_QUADS);
//			else
//				glBegin(GL_TRIANGLES);
//			break;
//		case kLines:
//			glBegin(GL_LINE_LOOP);
//			break;
//		case kPoints:
//		default:
//			glBegin(GL_POINTS);
//			break;
//	}
//	switch (t->split) {
//		case kNoSplit:
//			if (t->tile1.type != kOutOfBounds)
//			{
//				if ((tileSet == kWinterTile) || 
//						(tileSet == kFallTile))
//					rr *= 0.9;   // Leave empty grid lines between
//				
//				DoVertexColor(t->tile1.type, t->tile1.corners[0], !AdjacentCorners(x, y, kTopLeft));
//				DoNormal(t->split, &t->tile1, x, y);
//				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
//				
//				DoVertexColor(t->tile1.type, t->tile1.corners[1], !AdjacentCorners(x, y, kBottomLeft));
//				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
//				
//				DoVertexColor(t->tile1.type, t->tile1.corners[2], !AdjacentCorners(x, y, kBottomRight));
//				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[2]);
//				
//				DoVertexColor(t->tile1.type, t->tile2.corners[0], !AdjacentCorners(x, y, kTopRight));
//				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
//			}
//			else {
////				if (wall != -1)
////				{
//				glColor3f(0, 0, 0);
////				}
////				else
////					glColor3f(0.3, 0.3, 0.3);
//				// top
//
//				glNormal3d(0, 0, -1);
//				glTexCoord2f(0.51,1);
//				glVertex3f(xx-rr, yy-rr, /*-2*rr*/-rr*t->tile1.corners[0]);
//				glTexCoord2f(0.51,0.51);
//				glVertex3f(xx-rr, yy+rr, /*-2*rr*/-rr*t->tile1.corners[1]);
//				glTexCoord2f(1,0.51);
//				glVertex3f(xx+rr, yy+rr, /*-2*rr*/-rr*t->tile1.corners[2]);
//				glTexCoord2f(1,1);
//				glVertex3f(xx+rr, yy-rr, /*-2*rr*/-rr*t->tile2.corners[0]);
//
//
////				glColor3f(1.0, 0.1, 0.1);
////				// side 1
////				glVertex3f(xx-rr, yy-rr, -2*rr-rr*t->tile1.corners[0]);
////				glVertex3f(xx-rr, yy+rr, -2*rr-rr*t->tile1.corners[1]);
////				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[2]);
////				glVertex3f(xx-rr, yy-rr, -rr*t->tile2.corners[0]);
////
////				glColor3f(1.0, 1.0, 0.1);
////				// side 2
////				glVertex3f(xx+rr, yy-rr, -rr*t->tile1.corners[0]);
////				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[1]);
////				glVertex3f(xx+rr, yy+rr, -2*rr-rr*t->tile1.corners[2]);
////				glVertex3f(xx+rr, yy-rr, -2*rr-rr*t->tile2.corners[0]);
////
////				glColor3f(0.0, 1.0, 0.1);
////				// side 3
////				glVertex3f(xx-rr, yy-rr, -2*rr-rr*t->tile1.corners[0]);
////				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[1]);
////				glVertex3f(xx+rr, yy-rr, -rr*t->tile1.corners[2]);
////				glVertex3f(xx+rr, yy-rr, -2*rr-rr*t->tile2.corners[0]);
////
////				glColor3f(0.0, 0.0, 1.0);
////				// side 4
////				glVertex3f(xx-rr, yy+rr, -2*rr-rr*t->tile1.corners[0]);
////				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
////				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[2]);
////				glVertex3f(xx+rr, yy+rr, -2*rr-rr*t->tile2.corners[0]);
//
//			}
//			break;
//		case kForwardSplit:
//			if (t->tile1.type != kOutOfBounds)
//			{
//				DoNormal(t->split, &t->tile1, x, y);
//				DoVertexColor(t->tile1.type, t->tile1.corners[0]);
//				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
//				DoVertexColor(t->tile1.type, t->tile1.corners[1]);
//				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
//				DoVertexColor(t->tile1.type, t->tile1.corners[2]);
//				glVertex3f(xx+rr, yy-rr, -rr*t->tile1.corners[2]);
//			}
//			if (how == kLines)
//			{ glEnd(); glBegin(GL_LINE_LOOP); }
//			if (t->tile2.type != kOutOfBounds)
//			{
//				DoNormal(t->split, &t->tile2, x, y);
//				DoVertexColor(t->tile2.type, t->tile2.corners[0]);
//				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
//				DoVertexColor(t->tile2.type, t->tile2.corners[1]);
//				glVertex3f(xx+rr, yy+rr, -rr*t->tile2.corners[1]);
//				DoVertexColor(t->tile2.type, t->tile2.corners[2]);
//				glVertex3f(xx-rr, yy+rr, -rr*t->tile2.corners[2]);
//			}
//			break;
//		case kBackwardSplit:
//			if (t->tile1.type != kOutOfBounds)
//			{
//				DoNormal(t->split, &t->tile1, x, y);
//				DoVertexColor(t->tile1.type, t->tile1.corners[0]);
//				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
//				//glVertex3f((double)x/width-.5, (double)2.0*t->tile1.corners[0]/(height+width), (double)y/height-.5);
//				DoVertexColor(t->tile1.type, t->tile1.corners[1]);
//				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
//				//glVertex3f((double)x/width-.5, (double)2.0*t->tile1.corners[1]/(height+width), (double)y/height+1.0/height-.5);
//				DoVertexColor(t->tile1.type, t->tile1.corners[2]);
//				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[2]);
//				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile1.corners[2]/(height+width), (double)y/height+1.0/height-.5);
//			}
//			if (how == kLines)
//			{ glEnd(); glBegin(GL_LINE_LOOP); }
//			
//			if (t->tile2.type != kOutOfBounds)
//			{
//				DoNormal(t->split, &t->tile2, x, y);
//				DoVertexColor(t->tile2.type, t->tile2.corners[0]);
//				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
//				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile2.corners[0]/(height+width), (double)y/height-.5);
//				DoVertexColor(t->tile2.type, t->tile2.corners[1]);
//				glVertex3f(xx+rr, yy+rr, -rr*t->tile2.corners[1]);
//				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile2.corners[1]/(height+width), (double)y/height+1.0/height-.5);
//				DoVertexColor(t->tile2.type, t->tile2.corners[2]);
//				glVertex3f(xx-rr, yy-rr, -rr*t->tile2.corners[2]);
//				//glVertex3f((double)x/width-.5, (double)2.0*t->tile2.corners[2]/(height+width), (double)y/height-.5);
//			}
//			break;
//	}
//	glEnd();
//	glDisable(GL_TEXTURE_2D);
//}

/**
* Using OpenGL set the correct color for a particular vertex.
 *
 * calls the appropriate openGL functions to set the draw color according
 * to the tile height/type
 */
void Map::DoVertexColor(tTerrain type, int vHeight, bool darken) const
{
	double scaleH = (10.0-vHeight)/10.0;
	double red=0, green=0, blue=0, alpha = 1.0;
	switch (type)
	{
		case kOutOfBounds:
			red = 0; green = 0.2; blue = 0; alpha = .5; // kOutOfBounds
			break;
		case kOutOfBounds2:
			red = .9; green = .9; blue = .9; alpha = .5; // kOutOfBounds2
			break;
		case kWater:
			red = 0; green = (random()%10)/100.0; blue = scaleH*1-(random()%10)/100.0;
			break;
		case kGround:
			if ((tileSet == kFall) || (tileSet == kFallTile))
			{
				double r1=0, r2=0, r3=0;
				if (tileSet == kFall)
				{
					r1 = (random()%10-5)/100.0;
					r2 = (random()%10-5)/100.0;
					r3 = (random()%5)/100.0;
				}
				red = scaleH*.5+r1;
				green = scaleH*.25+r2;
				blue = r3; // kGround
			}
			else {
				double r1=0, r2=0, r3=0;
				if (tileSet == kWinter)
				{
					r1 = -(random()%8)/100.0;
					r2 = -(random()%8)/100.0;
					r3 = -(random()%8)/100.0;
				}
				red = scaleH*0.9+r1;
				green = scaleH*0.9+r2;
				blue = scaleH*0.9+r3;
			}
			break;
		case kTrees:
			red = (random()%10)/100.0;
			green = .45*scaleH-(random()%10)/100.0;
			blue = (random()%10)/100.0;
			break;
		case kSwamp:
			red = (scaleH*.5+(random()%10-5)/100.0)*0.9;
			green = (scaleH*.25+(random()%10-5)/100.0)*0.9;
			blue = (80+random()%10-5)/100.0;
			break;
		case kGrass:
			red = (random()%10)/100.0;
			green = scaleH-(random()%10)/100.0;
			blue = (random()%10)/100.0;
			break;
		case kBlight:
			red = (50+random()%10-5)/100.0;
			green = (50+random()%10-5)/100.0;
			blue = (50+random()%10-5)/100.0;
			break;
		default:
			break;
	}
//	if ((darken) && (type != kGround))
//		glColor4f(.5*red, .5*green, .5*blue, alpha);
//	else if ((darken) && (type == kGround))
//		glColor4f(.8*red, .8*green, .8*blue, alpha);
//	else
//		glColor4f(red, green, blue, alpha);
}

/**
* does a rough approximation of the normal for a particular halfTile.
 * (if I recall, this isn't perfect...)
 */
void Map::DoNormal(tSplit split, halfTile *t, int /*x*/, int /*y*/) const
{
	Graphics::point n,pa,pb;
	
	pa.x = 0;
	pa.y = (t->corners[1]-t->corners[0])/(height+width);
	pa.z = 1/height;
	
	pb.x = 1/width;
	switch (split) {
		case kNoSplit:
		case kBackwardSplit:
			pb.z = 1/height;
			break;
		case kForwardSplit:
			pb.z = 0;
			break;
	}
	pb.y = (t->corners[2]-t->corners[0])/(height+width);
	pa.normalise();
	pb.normalise();
	
	n.x = pb.y * pa.z - pb.z * pa.y;
	n.y = pb.z * pa.x - pb.x * pa.z;
	n.z = pb.x * pa.y - pb.y * pa.x;
	n.normalise();
	
//	glNormal3f(n.x,n.y,n.z);
}

//void Map::drawLandQuickly() const
//{
//	GLdouble xx, yy, zz, rr;
//	glBegin(GL_QUADS);
//	glColor3f(0.5, 0.5, 0.5);
//	for (int y = 0; y < height; y++)
//	{
//		if (GetTerrainType(0, y) == kGround)
//		{
//			GetCoord(0, y, xx, yy, zz, rr);
//			glVertex3f(xx-rr, yy-rr, zz);
//			glVertex3f(xx-rr, yy+rr, zz);
//		}
//		for (int x = 1; x < width; x++)
//		{
//			if (GetTerrainType(x, y) != GetTerrainType(x-1, y))
//			{
//				if (GetTerrainType(x-1, y) == kGround)
//				{
//					GetCoord(x, y, xx, yy, zz, rr);
//					glVertex3f(xx-rr, yy+rr, zz);
//					glVertex3f(xx-rr, yy-rr, zz);
//				}
//				if (GetTerrainType(x, y) == kGround)
//				{
//					GetCoord(x, y, xx, yy, zz, rr);
//					glVertex3f(xx-rr, yy-rr, zz);
//					glVertex3f(xx-rr, yy+rr, zz);
//				}
//			}
//		}
//		if (GetTerrainType(width-1, y) == kGround)
//		{
//			GetCoord(width-1, y, xx, yy, zz, rr);
//			glVertex3f(xx+rr, yy+rr, zz);
//			glVertex3f(xx+rr, yy-rr, zz);
//		}
//	}
//
////  this will draw the lines for the map, but can be too busy
////	for (int y = 0; y < height; y++)
////	{
////		GetCoord(0, y, xx, yy, zz, rr);
////		glVertex3f(xx-rr, yy-rr, zz);
////		GetCoord(width-1, y, xx, yy, zz, rr);
////		glVertex3f(xx+rr, yy-rr, zz);
////	}
////	for (int y = 0; y < height; y++)
////	{
////		GetCoord(x, 0, xx, yy, zz, rr);
////		glVertex3f(xx-rr, yy-rr, zz);
////		GetCoord(x, height-1, xx, yy, zz, rr);
////		glVertex3f(xx-rr, yy+rr, zz);
////	}
//	glEnd();
//}

/**
* Sets the abstract Graph node number for this tile.
 *
 * Because we have a Graph representation of the map as well, we need some
 * way to get back and forth between the representations. This function will
 * set the unique data (nodeNum) for a tile/half tile so that we can go
 * from a tile in the map to a node in the Graph.
 */
void Map::SetNodeNum(int num, int x, int y, tCorner corner)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
	{
		printf("ERROR -- trying to set invalid node number!\n");
		return;
	}
	if ((corner == kBottomRight) || (corner == kTopRight))
		land[x][y].tile2.node = num;
	land[x][y].tile1.node = num;
}

/**
* Gets the abstract Graph node number for this tile.
 *
 * Because we have a Graph representation of the map as well, we need some
 * way to get back and forth between the representations. This function will
 * get the unique data (nodeNum) for a tile/half tile so that we can go
 * from a tile in the map to a node in the Graph.
 */
int Map::GetNodeNum(int x, int y, tCorner corner)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
	{
		//printf("ERROR -- trying to get invalid node number!\n");
		return -1;
	}
	if ((corner == kBottomRight) || (corner == kTopRight))
		return land[x][y].tile2.node;
	return land[x][y].tile1.node;
}

/*
 *
 * Lookup table for the edge widths.
 *
 * The edges are as follows:
 *
 *  ---------
 *  |   |   |
 *  ---------
 *  |  -|-  |
 *  ---------
 *
 * To cross the bottom middle edge.
 *
 * Tile types:
 *   Empty: 0
 *   Forward slash: 1
 *   Backslash: 2
 *
 *
 * To translate into the lookup table index, multiply
 * tile 1's type by 27, multiply tile 2's type by 9,
 * multiply tile 3's type by 3, and add all the values together,
 * along with tile 4.
 *
 * So you end up with: 27*T1 + 9*T2 + 3*T3 + T4
 *
 * For example,
 *
 *  ---------
 *  | \ | / |
 *  ---------
 *  |   | \ |
 *  ---------
 *
 * 27*2 + 9*1 + 3*0 + 2 = 65
 *
 * The edge width will be found in index 65.
 *
 */

extern "C" {
	
  double edgewidth[81] = {
    TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
		
    ROOT_TWO,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE,
		
    ONE
  };
}

/**
* Returns the edge width between (x, y) and (x+1, y)
 */
float Map::GetEdgeWidthX(int x, int y)
{
	// Boundary values
	if (x < 0 || y-1 < 0 || x+1 >= width || y+1 >= height)
		return 0.0f;
	
	int index1, index2;	
	
	// Check if the edge at (x+1, y) is adjacent (i.e. same height and terrain type)
	if (!AdjacentEdges(x, y, kRightEdge))
	{
		// different height or terrain, so undefined
		return 0.0f;
	}
	
	// Check to see that the top edges are both adjacent.  If either is not, get the edge width using the bottom two tiles
	if (!AdjacentEdges(x, y, kTopEdge) || !AdjacentEdges(x+1, y, kTopEdge))
	{
		//(x+1, y+1), (x, y+1), (x+1, y), (x, y)  
		index1 = (27*GetSplit(x+1, y+1)) + (9*GetSplit(x, y+1)) + (3*GetSplit(x+1, y)) + (GetSplit(x, y));
		return edgewidth[index1];
	}
	
	// Check to see that the bottom edges are both adjacent.  If either is not, get the edge width using the top two tiles
	if (!AdjacentEdges(x, y, kBottomEdge) || !AdjacentEdges(x+1, y, kBottomEdge))
	{
		//(x, y-1), (x+1, y-1), (x, y) (x+1, y)
		index1 = (27*GetSplit(x, y-1)) + (9*GetSplit(x+1, y-1)) + (3*GetSplit(x, y)) + (GetSplit(x+1, y));
		return edgewidth[index1];
	}
	
	// Otherwise, return the minimum of the edge widths from the top edges and bottom edges
	index1 = (27*GetSplit(x+1, y+1)) + (9*GetSplit(x, y+1)) + (3*GetSplit(x+1, y)) + (GetSplit(x, y));
	index2 = (27*GetSplit(x, y-1)) + (9*GetSplit(x+1, y-1)) + (3*GetSplit(x, y)) + (GetSplit(x+1, y));
	
	return (edgewidth[index1] < edgewidth[index2]) ? (edgewidth[index1]) : (edgewidth[index2]);
}



/**
* Returns the edge width between (x, y) and (x, y+1)
 */
float Map::GetEdgeWidthY(int x, int y)
{
	// Boundary values
	if (x-1 < 0 || y < 0 || x+1 >= width || y+1 >= height)
		return 0.0f;
	
	int index1, index2;	
	
	// Check if the edge at (x, y+1) is adjacent (i.e. same height and terrain type)
	if (!AdjacentEdges(x, y, kBottomEdge))
	{
		// different height or terrain, so undefined
		return 0.0f;
	}
	
	// Check to see that the right edges are both adjacent.  If either is not, get the edge width using the left two tiles
	if (!AdjacentEdges(x, y, kRightEdge) || !AdjacentEdges(x, y+1, kRightEdge))
	{
		//(x-1, y-1), (x-1, y), (x, y+1), (x, y)  
		index1 = (27*GetSplit(x-1, y-1)) + (9*GetSplit(x-1, y)) + (3*GetSplit(x, y+1)) + (GetSplit(x, y));
		return edgewidth[index1];
	}
	
	// Check to see that the left edges are both adjacent.  If either is not, get the edge width using the right two tiles
	if (!AdjacentEdges(x, y, kLeftEdge) || !AdjacentEdges(x, y+1, kLeftEdge))
	{
		//(x+1, y), (x+1, y+1), (x, y) (x, y+1)
		index1 = (27*GetSplit(x+1, y)) + (9*GetSplit(x+1, y+1)) + (3*GetSplit(x, y)) + (GetSplit(x, y+1));
		return edgewidth[index1];
	}
	
	// Otherwise, return the minimum of the edge widths from the top edges and bottom edges
	index1 = (27*GetSplit(x-1, y-1)) + (9*GetSplit(x-1, y)) + (3*GetSplit(x, y+1)) + (GetSplit(x, y));
	index2 = (27*GetSplit(x+1, y)) + (9*GetSplit(x+1, y+1)) + (3*GetSplit(x, y)) + (GetSplit(x, y+1));
	
	return (edgewidth[index1] < edgewidth[index2]) ? (edgewidth[index1]) : (edgewidth[index2]);
}

