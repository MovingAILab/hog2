/*
 *  $Id: map.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 03/07/07.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

// HOG File

#ifndef MAP_H
#define MAP_H

#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include "Graphics.h"

#include "Constants.h"
//#include "Graph.h"

enum tTileset {
	kFall,
	kFallTile,
	kWinter,
	kWinterTile,
	kFast,
	kBitmap
};

enum tDisplay {
	kPolygons,
	kLines,
	kPoints
};

// types of ground
//  bit definition:
//  low 4 bits are subcategories
//  high bits are large categories

const int terrainBits = 4;

enum tTerrain {
	kOutOfBounds =0x0, // not part of map
	kOutOfBounds2=0x1, // not part of map - different color
	kWater=0x10,     // water
	kGround=0x20,     // ground
	kSwamp=0x21,
	kGrass=0x22,
	kBlight=0x23,
	kTrees=0x30,
	kUndefined=0x40 // mixed type due to split tile
};

inline bool CanPass(long a, long b)
{ return ((a&0xF0) == (b&0xF0)); }

// ground split types
enum tSplit {
	kNoSplit=0,
	kForwardSplit=1, // ie like forward slash
	kBackwardSplit=2     // ie like back slash
};

// specifying which side of the split you want
enum tSplitSide {
	kWholeTile = 1,
	kLeftSide = 2,
	kRightSide = 3
};

enum tEdge {
	kInternalEdge,
	kLeftEdge,
	kRightEdge,
	kTopEdge,
	kBottomEdge
};

// other constants
enum {
	kUndefinedHeight = -999
};

const int kNoGraphNode = -1;

// corner types
enum tCorner {
	kNone = 0,
	kTopLeft = 1,
	kTopRight = 2,
	kBottomLeft = 3,
	kBottomRight = 4
};

class halfTile {
public:
	halfTile();
	
	long corners[3];
	tTerrain type;
	long node;
};

class Tile {
public:
	Tile();
	halfTile tile1, tile2;
	tSplit split;
};

enum tMapType {
	kOctile,
	kOctileCorner,
	kSokoban,
	kRaw
};

/*
 *
 * All the set/get functions should be fairly obvious, with the following
 * behavior:
 *
 * Setting the height/type without a split for a tile will reset it to be a
 * non-split tile.
 *
 * If you specify a split for a tile that is not split, 
 */

/**
 * A tile-based representation of the world.
 */

class Map {
public:
	Map(long width, long height);
	Map(const char *filename);
	Map(Map *);
	Map(FILE *);
	Map(std::istringstream &data);
	~Map();
	void Load(const char *filename);
	void Load(FILE *f);
	void setSizeMultipler(int _sizeMultiplier)
	{ sizeMultiplier = _sizeMultiplier; }
	void Scale(long newWidth, long newHeight);
	void Trim();
	void Save(std::stringstream &data);
	void Save(const char *filename);
	void Save(FILE *f);
	Map *Clone() { return new Map(this); }
	const char *GetMapName();
	void Print(int scale = 1);
	/** return the width of the map */
	inline long GetMapWidth() const { return width; }
	/** return the height of the map */
	inline long GetMapHeight() const { return height; }
	
	void SetTileSet(tTileset ts);
	tTileset GetTileSet();
	
	Tile &GetTile(long x, long y);
	
	tSplit GetSplit(long x, long y) const;
	void SetSplit(long x, long y, tSplit split);
	
	
	// returns kUndefined if tile is split and you request kWholeTile
	long GetTerrainType(long x, long y, tSplitSide split = kWholeTile) const;
	long GetTerrainType(long x, long y, tEdge side) const;
	void SetTerrainType(int32_t x1, int32_t y1,
						int32_t x2, int32_t y2, tTerrain t);
	
	// if tile is not split and you specify a split side, nothing happens
	// if tile is split and you specify kWholeTile, the split remains,
	// and the terrain is applied to both sides
	void SetTerrainType(long x, long y, tTerrain type, tSplitSide split = kWholeTile);
	
	// returns kUndefinedHeight if the tile is split and you specify
	// the whole tile
	long GetHeight(long x, long y, tSplitSide split = kWholeTile);
	
	// if you specify a split, and the tile isn't split that direction,
	// the tile will be split that direction, and then the height applied
	void SetHeight(long x, long y, long height, tSplitSide split = kWholeTile);
	
	// returns kUndefinedHeight if the split is inconsistant with the tile type
	long GetCornerHeight(long x, long y, tCorner which, tEdge edge) const;
	long GetCornerHeight(long x, long y, tCorner which, tSplitSide split = kWholeTile) const;
	void SetCornerHeight(long x, long y, tCorner which, long height, tSplitSide split = kWholeTile);
	
	
	// ===============================================
	void SmoothSetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type = kGround);
	void SetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type = kGround);
	bool AdjacentEdges(long x, long y, tEdge edge) const;
	bool AdjacentCorners(long x, long y, tCorner corner) const;
	// returns whether we can step between two locations or not
	bool CanStep(long x1, long y1, long x2, long y2) const;
	
//	void OpenGLDraw(tDisplay how = kPolygons) const;
	bool GetCoord(int _x, int _y, double &x, double &y, double &z, double &radius) const;
	bool GetCoord(float _x, float _y, double &x, double &y, double &z, double &radius) const;
	void GetPointFromCoordinate(Graphics::point loc, int &px, int &py) const;
	double GetCoordinateScale();
	
	void SetDrawLand(bool land);
	bool GetDrawLand() { return drawLand; }
	//void DrawTile(Tile *t, int x, int y, tDisplay how) const;
	void DoVertexColor(tTerrain type, int height, bool darken = false) const;
	void DoNormal(tSplit split, halfTile *t, int x, int y) const;
	
	float GetEdgeWidthX(int x, int y);
	float GetEdgeWidthY(int x, int y);
	
	int GetNodeNum(int x, int y, tCorner c = kNone);
	void SetNodeNum(int num, int x, int y, tCorner c = kNone);
	int GetRevision() { return revision; }
private:
	void loadRaw(FILE *f, int height, int width);
	void loadOctile(FILE *f, int height, int width);
	void loadOctileCorner(FILE *f, int height, int width);
	void saveOctile(FILE *f);
	void saveRaw(FILE *f);
	bool tryLoadRollingStone(FILE *f);
	bool tryDragonAge(FILE *f);
	bool isLegalStone(char c);
	void paintRoomInside(int x, int y);
	//void drawLandQuickly() const;
	int width, height;
	Tile **land;
	bool drawLand;
//	mutable GLuint dList;
	mutable bool updated;
	int sizeMultiplier;
	int revision;
	char map_name[128];
	tMapType mapType;
	tTileset tileSet;
};

#endif
