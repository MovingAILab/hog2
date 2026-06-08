/*
 *  Map3DGrid.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/6/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */


#ifndef MAP3DGRID_H
#define MAP3DGRID_H

#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <cassert>
#include "SearchEnvironment.h"
#include "BitVector.h"
#include "GraphEnvironment.h"

extern int gSectorSize;

inline int indexFromXY(int x, int y)
{ return y*gSectorSize+x; }

inline void xyFromIndex(int index, int &x, int &y)
{ x = index%gSectorSize; y = index/gSectorSize; }

enum moveDir {
	kWest = 0x1,
	kEast = 0x2,
	kNorth = 0x4,
	kSouth = 0x8,
	kNorthEast = 0x10,
	kNorthWest = 0x20,
	kSouthEast = 0x40,
	kSouthWest = 0x80
};


// 8 bits - movable directions
// 5 bits - base height offset [0xF0]
// 3 bits - blockage
class GridData {
public:
	GridData()
	{
		points = new uint32_t[gSectorSize*gSectorSize];
		for (int x = 0; x < gSectorSize*gSectorSize; x++)
			points[x] = 0xFF;
	}
	GridData(const GridData& copy_from_me)
	{
		points = new uint32_t[gSectorSize*gSectorSize];
		for (int x = 0; x < gSectorSize*gSectorSize; x++)
			points[x] = copy_from_me.points[x];		
	}
	~GridData() {delete [] points; }
	int AddMove(unsigned index, moveDir dir, bool local);
	void AddMove(unsigned int x, unsigned int y, moveDir dir, bool local);
	int ReMove(unsigned index, moveDir dir);
	void ReMove(unsigned int x, unsigned int y, moveDir dir);
	uint8_t GetMoves(int x, int y) const;
	uint8_t GetMoves(int offset) const;
	uint8_t GetMoveLocality(int x, int y) const;
	uint8_t GetMoveLocality(int offset) const;
	void ClearMoves(int x, int y);
	
	int GetHeightOffset(unsigned int x, unsigned int y) const;
	int GetHeightOffset(int offset) const;
	void SetHeightOffset(int x, int y, int height);
	void SetHeightOffset(int offset, int height);
	bool AddPoint(int x, int y, int height);
	void RemovePoint(int x, int y);
	bool IsPointPassable(unsigned int x, unsigned int y) const;
	bool CanPass(unsigned int x, unsigned int y, int offsetx, int offsety) const;
	int GetBlocked(int x, int y);
	void AddBlocked(int x, int y);
	void SubtractBlocked(int x, int y);
	// neighbors: 8+2xladder+1x connection
	int LabelAreas(std::vector<int> &labels, std::vector<int> &counts, std::vector<int> &heights);
private:
	int BFS(std::vector<int> &labels, int offset, int label);
	void AddIntraRegionEdges(int xRegLoc, int yRegLoc);
	void RemoveIntraRegionEdges(int xRegLoc, int yRegLoc);
	uint32_t *points;//[gSectorSize*gSectorSize];
};

// at the top of regions we have:
// [8 bits] last edge
// [8 bits] center location
// [16 bits] grid location
// [8 bits] base height
// [8 bits] # points in region
// [8 bits] # number partially blocked points

class EdgeData
{
public:
//	void GetDestSector(int &sectorX, int &sectorY)
//	{ 
//		switch ((theEdge>>5)&0x3)
//		{
//			case 0: sectorX += 1; break;
//			case 1: sectorX -= 1; break;
//			case 2: sectorY += 1; break;
//			case 3: sectorY -= 1; break;
//			case 4: sectorX += 1; sectorY += 1; break;
//			case 5: sectorX += 1; sectorY -= 1; break;
//			case 6: sectorX -= 1; sectorY += 1; break;
//			case 7: sectorX -= 1; sectorY -= 1; break;
//		}
//	}
//	int GetDestRegion() { return theEdge&0x1F; }
//private:
	EdgeData()
	:sector(-1), region(-1), support(-1) {}
	EdgeData(int sec, int reg, int sup)
	:sector(sec), region(reg), support(sup) {}
	uint16_t sector;
	uint8_t region;
	uint8_t support; // underlying edges supporting this edge
};


class RegionData {
public:
	RegionData()
	:baseHeight(0), pointCount(0) {}
//	int GetRegionGridIndex();
//	int GetRegionEdgeCount();
//	//int GetRegionEdge(int edge);
//	//int GetRegionBaseHeight();
//	void GetRegionCenter(int &x, int &y);
//	int GetRegionPoints();
//	int GetRegionBlocked();

	// points passed in local region coordinates
	bool CanAddPoint(int x, int y, int z);
	bool AddPoint(int x, int y, int z);
	void AddEdge(EdgeData &e);
	void RemoveEdge(EdgeData &e);
	bool LowerBase();
	void RedoCenterLocation();
//private:
	uint8_t centerLocation;
	uint8_t baseHeight;
	uint8_t pointCount;
	uint8_t blockedPointCount;
	std::vector<EdgeData> edges;
	GridData grid;
};

class SectorData {
public:
	bool AddPoint(int x, int y, int z);
	bool RemovePoint(int x, int y, int z);
	// num regions implicit
	std::vector<RegionData> regions;
};

// either sector/region
// or x/y/z location
class state3d
{
public:
	state3d() { sector = -1; region = -1; offset = -1; }
	state3d(int s, int r, int o) :sector(s), region(r), offset(o) { }
	void Init(int sector, int region, int offset)
	{ this->sector = sector; this->region = region; this->offset = offset; }
	int GetSector() const { return sector; }
	int GetRegion() const { return region; }
	int GetOffset() const { return offset; }
	void SetOffset(int off) { offset = off; }
//private:
	uint16_t sector;
	uint8_t region, offset;
};

static bool operator==(const state3d &l1, const state3d &l2)
{
	return ((l1.GetSector() == l2.GetSector()) &&
			(l1.GetRegion() == l2.GetRegion()) &&
			(l1.GetOffset() == l2.GetOffset()));
}

// either direction (x/y offset?)
// or edge [direction + region]
class action3d
{
public:
	uint8_t direction;
	uint8_t destRegion;
	uint8_t destSector;
};

static bool operator==(const action3d &l1, const action3d &l2)
{
	return ((l1.direction == l2.direction) &&
			(l1.destRegion == l2.destRegion) &&
			(l1.destSector == l2.destSector));
}

class Map3DGrid : public SearchEnvironment<state3d, action3d> {
public:
	Map3DGrid(int width, int height, int theSectorSize);
	Map3DGrid(Map *map, int theSectorSize);
	int GetWidth() { return mWidth; }
	int GetHeight() { return mHeight; }
	void PrintStats();
	int GetAbstractionBytesUsed();
	int GetGridBytesUsed();

	void AddMap(Map *map, int elevation);
	void GetSuccessors(const state3d &nodeID, std::vector<state3d> &neighbors) const;
	void GetActions(const state3d &nodeID, std::vector<action3d> &actions) const;
	void ApplyAction(state3d &s, action3d a) const;
	bool InvertAction(action3d &a) const { return false; }
	action3d GetAction(const state3d &s1, const state3d &s2) const { return action3d(); }
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const state3d &node1, const state3d &node2) const;
	double GCost(const state3d &node1, const state3d &node2) const;
	double GCost(const state3d &node, const action3d &act) const;
	bool GoalTest(const state3d &node, const state3d &goal) const;
	
	uint64_t GetStateHash(const state3d &node) const;
	uint64_t GetActionHash(action3d act) const;
	
	int FindNearState(int x, int y, int z, state3d &s) const;
	
//	void OpenGLDraw() const;
//	void OpenGLDraw(const state3d&) const;
//	void OpenGLDraw(const state3d&, const action3d&) const { }
//	void GLDrawLine(const state3d &x, const state3d &y) const;

	
	bool AddPoint(int x, int y, int z);
	bool RemovePoint(int x, int y, int z, bool split = true);
	void SetDrawGrid(bool draw) { drawGrid = draw; }
	bool GetDrawGrid() { return drawGrid; }
	void GetPointFromCoordinate(point3d loc, int &px, int &py, int &pz) const;
private:
	void AddEdge(state3d &from, state3d &to);
	int AddGridEdge(state3d &from, state3d &to, bool local);
	int AddSectorEdge(state3d &from, state3d &to);
	void AddEdge(int sec1, int reg1, int sec2, int reg2, int weight);
	void AddMapPoints(Map *map, std::vector<bool> &visited, int x, int y, int elevation);

	int InternalRemovePoint(int x, int y, int z);
	void RemoveEdge(int sec1, int reg1, int sec2, int reg2, int weight);
	void RemoveEdge(state3d &from, state3d &to);
	int RemoveGridEdge(state3d &from, state3d &to);
	int RemoveSectorEdge(state3d &from, state3d &to);
	
	
	//AddIntraRegionEdges(int xRegLoc, int yRegLoc);
	int GetSector(int x, int y) const { return (y/gSectorSize)*mXSectors +(x/gSectorSize); }
	void GetXYFromState(const state3d &s, int &x, int &y) const;
	void GetXYZFromState(const state3d &s, int &x, int &y, int &z) const;
	void GetStateFromXYZ(state3d &s, int x, int y, int z) const;
//	void GetCoord(int xLoc, int yLoc, int zLoc,
//						GLdouble &x, GLdouble &y, GLdouble &z,
//						GLdouble &r, GLdouble &h) const;
	int mWidth, mHeight;
	int mXSectors, mYSectors;
	std::vector<SectorData> sectors;
	bool drawGrid;
};
	
#endif

