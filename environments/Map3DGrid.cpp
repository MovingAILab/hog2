/*
 *  Map3DGrid.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/6/11.
 *  Copyright 2011 University of Denver. All rights reserved.
 *
 */

#include "Map3DGrid.h"
#include <deque>

int gSectorSize = 16;
//double gInvSectorSize = 1/16;

//	uint8_t points[gSectorSize*gSectorSize];
// 8 bits - movable directions
// 5 bits - base height offset
////// 2 bits - sector connections
// 3 bits - blockage

const uint8_t kUnpassableHeight = 0x1F;

int GridData::AddMove(unsigned index, moveDir dir, bool local)
{
	assert(index < gSectorSize*gSectorSize);
	int cnt = 1;
	if (points[index]&(((int)dir)<<8))
		cnt = 0;
	points[index] |= (((int)dir)<<8);
	uint32_t val = (((int)dir)<<16);
	if (local)
		points[index] |= val;
	else
		points[index] &= (~val);
	return cnt;
}

void GridData::AddMove(unsigned int x, unsigned int y, moveDir dir, bool local)
{
	points[indexFromXY(x, y)] |= (((int)dir)<<8);
	uint32_t val = (((int)dir)<<16);
	if (local)
		points[indexFromXY(x, y)] |= val;
	else
		points[indexFromXY(x, y)] &= (~val);
}

int GridData::ReMove(unsigned index, moveDir dir)
{
	assert(index < gSectorSize*gSectorSize);
	int cnt = 0;
	if (points[index]&(((int)dir)<<8))
		cnt = 1;
	points[index] &= ~(((int)dir)<<8);
	return cnt;
}

void GridData::ReMove(unsigned int x, unsigned int y, moveDir dir)
{
	if ((x >= gSectorSize) || (y >= gSectorSize))
		return;
	uint16_t val = (((int)dir)<<8);
	points[indexFromXY(x, y)] &= (~val);
}

uint8_t GridData::GetMoves(int x, int y) const
{
	return (points[indexFromXY(x, y)]>>8)&0xFF;
}

uint8_t GridData::GetMoves(int offset) const
{
	return (points[offset]>>8)&0xFF;
}

uint8_t GridData::GetMoveLocality(int x, int y) const
{
	return (points[indexFromXY(x, y)]>>16)&0xFF;
}

uint8_t GridData::GetMoveLocality(int offset) const
{
	return (points[offset]>>16)&0xFF;	
}


void GridData::ClearMoves(int x, int y)
{
	points[indexFromXY(x, y)] &= 0xFFFF00FF;
}

bool GridData::AddPoint(int x, int y, int height)
{
	SetHeightOffset(x, y, height);
	return true;
}

void GridData::RemovePoint(int x, int y)
{
	points[indexFromXY(x, y)] = points[indexFromXY(x, y)]|(kUnpassableHeight<<3);
	//SetHeightOffset(x, y, kUnpassableHeight);
}

bool GridData::IsPointPassable(unsigned int x, unsigned int y) const
{
	if ((x >= gSectorSize) || (y >= gSectorSize))
		return false;
	return ((points[indexFromXY(x, y)]>>3)&0x1F)!=kUnpassableHeight;
}

bool GridData::CanPass(unsigned int x, unsigned int y, int offsetx, int offsety) const
{
	return (IsPointPassable(x, y) && IsPointPassable(x+offsetx, y+offsety) &&
			abs(GetHeightOffset(x, y)-GetHeightOffset(x+offsetx, y+offsety)) <= 1);
}


int GridData::GetHeightOffset(int offset) const
{
	return (points[offset]>>3)&0x1F;
}

int GridData::GetHeightOffset(unsigned int x, unsigned int y) const
{
	if ((x >= gSectorSize) || (y >= gSectorSize))
		return 0x1F;
	return (points[indexFromXY(x, y)]>>3)&0x1F;
}

void GridData::SetHeightOffset(int offset, int height)
{
	assert(height < 31 && height >= 0);
	points[offset] = (points[offset]&0xFF07)|height<<3;
	//	return points[indexFromXY(x, y)]>>4; 
}

void GridData::SetHeightOffset(int x, int y, int height)
{
	assert(height < 31 && height >= 0);
	points[indexFromXY(x, y)] = (points[indexFromXY(x, y)]&0xFF07)|height<<3;
//	return points[indexFromXY(x, y)]>>4; 
}

int GridData::GetBlocked(int x, int y)
{
	return points[indexFromXY(x, y)]&7; 
}

void GridData::AddBlocked(int x, int y)
{
	int curr = points[indexFromXY(x, y)]&7;
	if (curr < 7)
		points[indexFromXY(x, y)]++;
}

void GridData::SubtractBlocked(int x, int y)
{
	int curr = points[indexFromXY(x, y)]&7;
	if (curr > 0) 
		points[indexFromXY(x, y)]--;
}

int GridData::LabelAreas(std::vector<int> &labels, std::vector<int> &counts, std::vector<int> &heights)
{
	labels.resize(gSectorSize*gSectorSize);
	heights.resize(gSectorSize*gSectorSize);
	counts.resize(0);
	for (unsigned int x = 0; x < gSectorSize*gSectorSize; x++)
		labels[x] = -1;
	int nextID = 0;
	for (unsigned int x = 0; x < gSectorSize*gSectorSize; x++)
	{
		int height = GetHeightOffset(x);
		heights[x] = height;
		if ((labels[x] == -1) && (height != kUnpassableHeight))
		{
			counts.push_back(BFS(labels, x, nextID));
			nextID++;
		}
	}
	return nextID;
}

int GridData::BFS(std::vector<int> &labels, int offset, int label)
{
	static std::vector<int> q;
	q.resize(0);
	q.push_back(offset);
	int cnt = 0;
	while (q.size() != 0)
	{
		int nextLoc = q.back();
		q.pop_back();
		if (labels[nextLoc] != -1)
		{
			assert(labels[nextLoc] == label);
			continue;
		}
		cnt++;
		labels[nextLoc] = label;
		int height = GetHeightOffset(nextLoc);
		if (nextLoc > gSectorSize)
		{
			if ((labels[nextLoc-gSectorSize] == -1) &&
				(abs(GetHeightOffset(nextLoc-gSectorSize)-height) <= 1))
				q.push_back(nextLoc-gSectorSize);
		}
		if (nextLoc%gSectorSize != 0)
		{
			if ((labels[nextLoc-1] == -1) &&
				(abs(GetHeightOffset(nextLoc-1)-height) <= 1))
				q.push_back(nextLoc-1);
		}
		if ((nextLoc%gSectorSize) != gSectorSize-1)
		{
			if ((labels[nextLoc+1] == -1) &&
				(abs(GetHeightOffset(nextLoc+1)-height) <= 1))
				q.push_back(nextLoc+1);
		}
		if ((nextLoc < gSectorSize*gSectorSize-gSectorSize) != 0)
		{
			if ((labels[nextLoc+gSectorSize] == -1) &&
				(abs(GetHeightOffset(nextLoc+gSectorSize)-height) <= 1))
				q.push_back(nextLoc+gSectorSize);
		}
	}
	return cnt;
}

// at the top of regions we have:
// [8 bits] last edge
// [8 bits] center location
// [16 bits] grid location
// [8 bits] base height
// [8 bits] # points in region
// [8 bits] # number partially blocked points

// each edge has 16 bits
// 3 bits direction
// 5 bits cost(?)
// 8 bits target region

//int RegionData::GetRegionEdgeCount(int region)
//{
//	loc = data->at(7*region);
//	if (region > 1)
//		loc -= data->at(7*(region-1));
//	return loc;
//}
//
//int RegionData::GetRegionGridIndex(int region)
//{
//	return (data->at(7*region+2)<<8)|data->at(7*region+3);
//}
//
//int RegionData::GetRegionEdge(int region, int edge)
//{
////	data->at(7*numRegions)+
//}
//
//int RegionData::GetRegionBaseHeight(int region)
//{
//}
//
//void RegionData::GetRegionCenter(int region, int &x, int &y)
//{
//}
//
//int RegionData::GetRegionPoints(int region)
//{
//}
//
//int RegionData::GetRegionBlocked(int region)
//{
//}

bool RegionData::LowerBase()
{
	int maxHeight = 0;
	for (int x = 0; x < gSectorSize*gSectorSize; x++)
	{
		int val = grid.GetHeightOffset(x);
		if ((val != kUnpassableHeight) && (val > maxHeight))
			maxHeight = val;
	}
	if (maxHeight < 30)
	{
		baseHeight--;
		for (int x = 0; x < gSectorSize*gSectorSize; x++)
		{
			int val = grid.GetHeightOffset(x);
			if (val != kUnpassableHeight)
				grid.SetHeightOffset(x, val+1);
		}
		return true;
	}
	return false;
}

// points passed in local region coordinates
bool RegionData::CanAddPoint(int x, int y, int z)
{
	// check max height and other sanity checks
	if (pointCount != 0)
	{
		if (z-baseHeight > 30)
			return false;
	}
	if (grid.IsPointPassable(x, y))
		return false;
	// check if next to another passable point
	if ((grid.IsPointPassable(x+1, y  ) && abs(grid.GetHeightOffset(x+1, y  )-(z-baseHeight)) < 2) ||
			(grid.IsPointPassable(x-1, y  ) && abs(grid.GetHeightOffset(x-1, y  )-(z-baseHeight)) < 2) ||
			(grid.IsPointPassable(x  , y-1) && abs(grid.GetHeightOffset(x  , y-1)-(z-baseHeight)) < 2) ||
			(grid.IsPointPassable(x  , y+1) && abs(grid.GetHeightOffset(x  , y+1)-(z-baseHeight)) < 2))
	{
		if (z < baseHeight)
			return LowerBase();
		return true;
	}
	return false;
	
	// if region does not exist, create it & add point (done)
	
	// if region exists, check if neighbor is on grid
	
	// if not, create new region & add point (done)
	
	// if point is blocked, create new region & add connections (done)
}

bool RegionData::AddPoint(int x, int y, int z)
{
	if (pointCount == 0)
	{
		baseHeight = z;
		centerLocation = y*gSectorSize+x;
	}
	else {
		RedoCenterLocation();
		//		int newx=0, newy=0, cnt=0;
//		for (int t = 0; t < gSectorSize*gSectorSize; t++)
//		{
//			if (grid.GetHeightOffset(t) != kUnpassableHeight)
//			{
//				int tmpx, tmpy;
//				xyFromIndex(t, tmpx, tmpy);
//				newx += tmpx;
//				newy += tmpy;
//				cnt++;
//			}
//		}
//		centerLocation = indexFromXY(newx/cnt, newy/cnt);
	}
	pointCount++;
	//int offset = indexFromXY(x, y);
	return grid.AddPoint(x, y, z-baseHeight);
}

void RegionData::RedoCenterLocation()
{
	int newx=0, newy=0, cnt=0;
	for (int t = 0; t < gSectorSize*gSectorSize; t++)
	{
		if (grid.GetHeightOffset(t) != kUnpassableHeight)
		{
			int tmpx, tmpy;
			xyFromIndex(t, tmpx, tmpy);
			newx += tmpx;
			newy += tmpy;
			cnt++;
		}
	}
	if (cnt != 0)
		centerLocation = indexFromXY(newx/cnt, newy/cnt);
}

void RegionData::AddEdge(EdgeData &e)
{
	for (unsigned int x = 0; x < edges.size(); x++)
	{
		if ((e.sector == edges[x].sector) && (e.region == edges[x].region))
		{
			edges[x].support += e.support;
			return;
		}
	}
	edges.push_back(e);
}

void RegionData::RemoveEdge(EdgeData &e)
{
	for (unsigned int x = 0; x < edges.size(); x++)
	{
		if ((e.sector == edges[x].sector) && (e.region == edges[x].region))
		{
			edges[x].support -= e.support;
			if (edges[x].support == 0)
			{
				edges[x] = edges.back();
				edges.resize(edges.size()-1);
			}
			return;
		}
	}
}


bool SectorData::AddPoint(int x, int y, int z)
{
	int regionX = x%gSectorSize;
	int regionY = y%gSectorSize;
	//int addedTo = -1;
	for (unsigned int t = 0; t < regions.size(); t++)
	{
		if (regions[t].CanAddPoint(regionX, regionY, z))
		{
			// don't know how to handle failure
			assert(regions[t].AddPoint(regionX, regionY, z));
			//addedTo = t;
			return true;
		}
		else {
//			regions[t].AddInterRegionEdges(regionX, regionY);
		}
	}
	regions.resize(regions.size()+1);
	return regions.back().AddPoint(regionX, regionY, z);
}

bool SectorData::RemovePoint(int x, int y, int z)
{
	int regionX = x%gSectorSize;
	int regionY = y%gSectorSize;
	for (unsigned int t = 0; t < regions.size(); t++)
	{
		if (regions[t].grid.GetHeightOffset(regionX, regionY) + regions[t].baseHeight == z)
		{
			regions[t].grid.RemovePoint(regionX, regionY);
			return true;
		}
	}
	return false;
}

Map3DGrid::Map3DGrid(Map *map, int theSectorSize)
{
	drawGrid = false;
	mWidth = map->GetMapWidth();
	mHeight = map->GetMapHeight();
	gSectorSize = theSectorSize;
//	gInvSectorSize = 1.0/gSectorSize;
	mXSectors = ceil((double)mWidth/(double)gSectorSize);
	mYSectors = ceil((double)mHeight/(double)gSectorSize);
	sectors.resize(mXSectors*mYSectors);
	AddMap(map, 0);
}

Map3DGrid::Map3DGrid(int width, int height, int theSectorSize)
:mWidth(width), mHeight(height)
{
	drawGrid = false;
	gSectorSize = theSectorSize;
	mXSectors = ceil((double)width/(double)gSectorSize);
	mYSectors = ceil((double)height/(double)gSectorSize);
	sectors.resize(mXSectors*mYSectors);
}


void Map3DGrid::AddMap(Map *map, int elevation)
{
	std::vector<bool> visited(map->GetMapWidth()*map->GetMapHeight());
	
	for (unsigned int loc = 0; loc < visited.size(); loc++)
	{
		if (!visited[loc] && (map->GetTerrainType(loc%map->GetMapWidth(), loc/map->GetMapWidth())>>terrainBits) == (kGround>>terrainBits))
		{
			AddMapPoints(map, visited, loc%map->GetMapWidth(), loc/map->GetMapWidth(), elevation);
		}
	}
}

void Map3DGrid::AddMapPoints(Map *map, std::vector<bool> &visited, int x, int y, int elevation)
{
	std::deque<int> nextx;
	std::deque<int> nexty;

	nextx.push_back(x);
	nexty.push_back(y);
	
	int sector = GetSector(x, y);
	int xLoc, yLoc;
	while (nextx.size() > 0)
	{
		xLoc = nextx.front();
		yLoc = nexty.front();
		nextx.pop_front();
		nexty.pop_front();

		if (visited[yLoc*map->GetMapWidth()+xLoc])
			continue;
		if (GetSector(xLoc, yLoc) != sector)
			continue;
					  
		visited[yLoc*map->GetMapWidth()+xLoc] = true;
		AddPoint(xLoc, yLoc, elevation);
		
		if (map->GetTerrainType(xLoc+1, yLoc)>>terrainBits == (kGround>>terrainBits))
		{
			nextx.push_back(xLoc+1);
			nexty.push_back(yLoc);
		}
		if (map->GetTerrainType(xLoc-1, yLoc)>>terrainBits == (kGround>>terrainBits))
		{
			nextx.push_back(xLoc-1);
			nexty.push_back(yLoc);
		}
		if (map->GetTerrainType(xLoc, yLoc+1)>>terrainBits == (kGround>>terrainBits))
		{
			nextx.push_back(xLoc);
			nexty.push_back(yLoc+1);
		}
		if (map->GetTerrainType(xLoc, yLoc-1)>>terrainBits == (kGround>>terrainBits))
		{
			nextx.push_back(xLoc);
			nexty.push_back(yLoc-1);
		}
	}
}

void Map3DGrid::GetSuccessors(const state3d &nodeID, std::vector<state3d> &neighbors) const
{
	if (nodeID.GetOffset() != -1) // on the actual grid
	{
		static const moveDir m[8] = {kWest, kEast, kNorth, kSouth, kNorthEast, kNorthWest, kSouthEast, kSouthWest};
		moveDir dirs = (moveDir)sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.GetMoves(nodeID.GetOffset());
		//uint8_t local = (moveDir)sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.GetMoveLocality(nodeID.GetOffset());
		action3d a;
		state3d s;
		a.destRegion = 0xFF;
		neighbors.resize(0);

		int x = -1, y, z;
		
		for (int t = 0; t < 8; t++)
		{
			if (dirs & m[t])
			{
				s = nodeID;
				if ((sectors[nodeID.GetSector()].regions.size() == 1) &&
					(nodeID.GetOffset() >= gSectorSize) && (nodeID.GetOffset() < gSectorSize*gSectorSize-gSectorSize) &&
					((nodeID.GetOffset()%gSectorSize) != 0) && (nodeID.GetOffset()%gSectorSize) != gSectorSize-1)
					//if (0 && local & m[t])// && ((dirs^local) == 0))
				{
					switch (m[t])
					{
						case kWest:
						{ s.offset--; break; }
						case kEast: 
						{ s.offset++; break; }
						case kNorth: 
						{ s.offset-=gSectorSize; break; }
						case kSouth: 
						{ s.offset+=gSectorSize; break; }
						case kNorthEast: 
						{ s.offset=s.offset-gSectorSize+1; break; }
						case kNorthWest: 
						{ s.offset=s.offset-gSectorSize-1; break; }
						case kSouthEast: 
						{ s.offset=s.offset+gSectorSize+1; break; }
						case kSouthWest: 
						{ s.offset=s.offset+gSectorSize-1; break; }
					}
				}
				else {
					if (x == -1)
						GetXYZFromState(nodeID, x, y, z);

					switch (m[t])
					{
						case kWest: 
							GetStateFromXYZ(s, x-1, y, z); break;
						case kEast: 
							GetStateFromXYZ(s, x+1, y, z); break;
						case kNorth: 
							GetStateFromXYZ(s, x, y-1, z); break;
						case kSouth: 
							GetStateFromXYZ(s, x, y+1, z); break;
						case kNorthEast: 
							GetStateFromXYZ(s, x+1, y-1, z); break;
						case kNorthWest: 
							GetStateFromXYZ(s, x-1, y-1, z); break;
						case kSouthEast: 
							GetStateFromXYZ(s, x+1, y+1, z); break;
						case kSouthWest: 
							GetStateFromXYZ(s, x-1, y+1, z); break;
					}
				}
				neighbors.push_back(s);
			}
		}
//		static std::vector<action3d> acts;
//		GetActions(nodeID, acts);
//		neighbors.resize(0);
//
//		if ((nodeID.GetOffset() >= gSectorSize) && (nodeID.GetOffset() < gSectorSize*gSectorSize-gSectorSize) &&
//			((nodeID.GetOffset()%gSectorSize) != 0) && (nodeID.GetOffset()%gSectorSize) != gSectorSize-1)
//		{
//			if (sectors[nodeID.GetSector()].regions.size() == 1)
//			{
//				for (unsigned int t = 0; t < acts.size(); t++)
//				{
//					state3d s(nodeID);
//					switch (acts[t].direction)
//					{
//						case kWest: 
//						{ s.offset--; break; }
//						case kEast: 
//						{ s.offset++; break; }
//						case kNorth: 
//						{ s.offset-=gSectorSize; break; }
//						case kSouth: 
//						{ s.offset+=gSectorSize; break; }
//						case kNorthEast: 
//						{ s.offset=s.offset-gSectorSize+1; break; }
//						case kNorthWest: 
//						{ s.offset=s.offset-gSectorSize-1; break; }
//						case kSouthEast: 
//						{ s.offset=s.offset+gSectorSize+1; break; }
//						case kSouthWest: 
//						{ s.offset=s.offset+gSectorSize-1; break; }
//					}
//					//			state3d s = nodeID;
//					//			ApplyAction(s, acts[x]);
//					neighbors.push_back(s);
//				}
//			}
//			else {
//				// just generate and check if neighbor is on the same grid with an appropriate height
//				// if so we're done
//				int h2 = sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.GetHeightOffset(nodeID.GetOffset());
//				int x = -1, y, z;
//				for (unsigned int t = 0; t < acts.size(); t++)
//				{
//					state3d s(nodeID);
//					switch (acts[t].direction)
//					{
//						case kWest: 
//						{
//							int h1 = sectors[s.GetSector()].regions[s.GetRegion()].grid.GetHeightOffset(s.GetOffset()-1);
//							if (abs(h1-h2)<=1 && (h1 != kUnpassableHeight))
//							{
//								s.offset--;
//							}
//							else {
//								if (x == -1)
//									GetXYZFromState(nodeID, x, y, z);
//								GetStateFromXYZ(s, x-1, y, z);
//							}
//							break;
//						}
//						case kEast: 
//						{
//							int h1 = sectors[s.GetSector()].regions[s.GetRegion()].grid.GetHeightOffset(s.GetOffset()+1);
//							if (abs(h1-h2)<=1 && (h1 != kUnpassableHeight))
//							{
//								s.offset++;
//							}
//							else {
//								if (x == -1)
//									GetXYZFromState(nodeID, x, y, z);
//								GetStateFromXYZ(s, x+1, y, z);
//							}
//							break;
//
////							s.offset++; break; 
//						}
//						case kNorth: 
//						{
//							int h1 = sectors[s.GetSector()].regions[s.GetRegion()].grid.GetHeightOffset(s.GetOffset()-gSectorSize);
//							if (abs(h1-h2)<=1 && (h1 != kUnpassableHeight))
//							{
//								s.offset-=gSectorSize;
//							}
//							else {
//								if (x == -1)
//									GetXYZFromState(nodeID, x, y, z);
//								GetStateFromXYZ(s, x, y-1, z);
//							}
//							break;
//							//s.offset-=gSectorSize; break;
//						}
//						case kSouth: 
//						{
//							int h1 = sectors[s.GetSector()].regions[s.GetRegion()].grid.GetHeightOffset(s.GetOffset()+gSectorSize);
//							if (abs(h1-h2)<=1 && (h1 != kUnpassableHeight))
//							{
//								s.offset+=gSectorSize;
//							}
//							else {
//								if (x == -1)
//									GetXYZFromState(nodeID, x, y, z);
//								GetStateFromXYZ(s, x, y+1, z);
//							}
//							break;
//							//s.offset+=gSectorSize; break;
//						}
//						case kNorthEast: 
//						{ s.offset=s.offset-gSectorSize+1; break; }
//						case kNorthWest: 
//						{ s.offset=s.offset-gSectorSize-1; break; }
//						case kSouthEast: 
//						{ s.offset=s.offset+gSectorSize+1; break; }
//						case kSouthWest: 
//						{ s.offset=s.offset+gSectorSize-1; break; }
//					}
//					//			state3d s = nodeID;
//					//			ApplyAction(s, acts[x]);
//					neighbors.push_back(s);
//				}
//			}
//		}
//		else {
//			int x, y, z;
//			GetXYZFromState(nodeID, x, y, z);
//			for (unsigned int t = 0; t < acts.size(); t++)
//			{
//				state3d s(nodeID);
//				switch (acts[t].direction)
//				{
//					case kWest: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, -1, 0))
//	//					{ s.offset--; break; }
//						GetStateFromXYZ(s, x-1, y, z); break;
//					case kEast: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, +1, 0))
//	//					{ s.offset++; break; }
//						GetStateFromXYZ(s, x+1, y, z); break;
//					case kNorth: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, 0, -1))
//	//					{ s.offset-=gSectorSize; break; }
//						GetStateFromXYZ(s, x, y-1, z); break;
//					case kSouth: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, 0, 1))
//	//					{ s.offset+=gSectorSize; break; }
//						GetStateFromXYZ(s, x, y+1, z); break;
//					case kNorthEast: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, 1, -1))
//	//					{ s.offset=s.offset-gSectorSize+1; break; }
//						GetStateFromXYZ(s, x+1, y-1, z); break;
//					case kNorthWest: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, -1, -1))
//	//					{ s.offset=s.offset-gSectorSize-1; break; }
//						GetStateFromXYZ(s, x-1, y-1, z); break;
//					case kSouthEast: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, 1, 1))
//	//					{ s.offset=s.offset+gSectorSize+1; break; }
//						GetStateFromXYZ(s, x+1, y+1, z); break;
//					case kSouthWest: 
//	//					if (sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.CanPass(x, y, -1, 1))
//	//					{ s.offset=s.offset+gSectorSize-1; break; }
//						GetStateFromXYZ(s, x-1, y+1, z); break;
//				}
//				
//				
//	//			state3d s = nodeID;
//	//			ApplyAction(s, acts[x]);
//				neighbors.push_back(s);
//			}
//		}
		//moveDir = sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.GetMoves(nodeID.GetOffset());
	}
	else {
		neighbors.resize(0);
		unsigned int cnt = sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].edges.size();
		for (unsigned int x = 0; x < cnt; x++)
		{
			neighbors.push_back(state3d(sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].edges[x].sector,
										sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].edges[x].region,
										-1));
		}
	}
}

void Map3DGrid::GetActions(const state3d &nodeID, std::vector<action3d> &actions) const
{
	static const moveDir m[8] = {kWest, kEast, kNorth, kSouth, kNorthEast, kNorthWest, kSouthEast, kSouthWest};
	
	if (nodeID.GetOffset() != -1) // on the actual grid
	{
		actions.resize(0);
		moveDir dirs = (moveDir)sectors[nodeID.GetSector()].regions[nodeID.GetRegion()].grid.GetMoves(nodeID.GetOffset());
		action3d a;
		a.destRegion = 0xFF;
		for (int x = 0; x < 8; x++)
		{
			if (dirs & m[x])
			{
				a.direction = m[x];
				actions.push_back(a);
			}
		}
	}
}

void Map3DGrid::ApplyAction(state3d &s, action3d a) const
{
	// grid move
	if (s.GetOffset() != -1)
	{
		int x, y, z;
		GetXYZFromState(s, x, y, z);
		switch (a.direction)
		{
			case kWest: x--; break;
			case kEast: x++; break;
			case kNorth: y--; break;
			case kSouth: y++; break;
			case kNorthEast: y--; x++; break;
			case kNorthWest: y--; x--; break;
			case kSouthEast: y++; x++; break;
			case kSouthWest: y++; x--; break;
		}
		GetStateFromXYZ(s, x, y, z);
	}
	else { // abstraction move
		s.Init(a.destSector, a.destRegion, -1);
	}
}

double Map3DGrid::HCost(const state3d &node1, const state3d &node2) const
{
	assert(false);
	return 1;
}

double Map3DGrid::GCost(const state3d &node1, const state3d &node2) const
{
	assert(false);
	return 1;
}

double Map3DGrid::GCost(const state3d &node, const action3d &act) const
{
	assert(false);
	return 1;
}

bool Map3DGrid::GoalTest(const state3d &node, const state3d &goal) const
{
	return (node == goal);
}


uint64_t Map3DGrid::GetStateHash(const state3d &node) const
{
	assert(false);
	return 0;
}

uint64_t Map3DGrid::GetActionHash(action3d act) const
{
	assert(false);
	return 0;
}

bool Map3DGrid::AddPoint(int x, int y, int z)
{
	if (x >= mWidth)
		return false;
	if (y >= mHeight)
		return false;
	if ((x < 0) || (y < 0))
		return false;
		
	bool result = sectors[GetSector(x, y)].AddPoint(x, y, z);

	state3d neighborhood[9];
	int found[9];

	found[0] = FindNearState(x-1, y-1, z, neighborhood[0]);
	if (found[0]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[0], x1, y1);
		assert(x1 == x-1);
		assert(y1 == y-1);
	}
	found[1] = FindNearState(x  , y-1, z, neighborhood[1]);
	if (found[1]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[1], x1, y1);
		assert(x1 == x);
		assert(y1 == y-1);
	}
	found[2] = FindNearState(x+1, y-1, z, neighborhood[2]);
	if (found[2]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[2], x1, y1);
		assert(x1 == x+1);
		assert(y1 == y-1);
	}
	
	found[3] = FindNearState(x-1, y, z, neighborhood[3]);
	if (found[3]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[3], x1, y1);
		assert(x1 == x-1);
		assert(y1 == y);
	}
	found[4] = FindNearState(x  , y, z, neighborhood[4]);
	if (found[4]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[4], x1, y1);
		assert(x1 == x);
		assert(y1 == y);
	}
	found[5] = FindNearState(x+1, y, z, neighborhood[5]);
	if (found[5]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[5], x1, y1);
		assert(x1 == x+1);
		assert(y1 == y);
	}
	
	found[6] = FindNearState(x-1, y+1, z, neighborhood[6]);
	if (found[6]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[6], x1, y1);
		assert(x1 == x-1);
		assert(y1 == y+1);
	}
	found[7] = FindNearState(x  , y+1, z, neighborhood[7]);
	if (found[7]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[7], x1, y1);
		assert(x1 == x);
		assert(y1 == y+1);
	}
	found[8] = FindNearState(x+1, y+1, z, neighborhood[8]);
	if (found[8]  != -1) 
	{
		int x1, y1;
		GetXYFromState(neighborhood[8], x1, y1);
		assert(x1 == x+1);
		assert(y1 == y+1);
	}
	
	assert(found[4] != -1);
	
	// connect horizontals
	if (found[1] != -1)
	{
//		printf("Add 4-1\n");
		AddEdge(neighborhood[4], neighborhood[1]);
	}
	if (found[3] != -1)
	{
//		printf("Add 4-3 [%d-%d]\n", found[4], found[3]);
		AddEdge(neighborhood[4], neighborhood[3]);
	}
	if (found[5] != -1)
	{
//		printf("Add 4-5\n");
		AddEdge(neighborhood[4], neighborhood[5]);
	}
	if (found[7] != -1)
	{
//		printf("Add 4-7\n");
		AddEdge(neighborhood[4], neighborhood[7]);
	}

	// connect diagonals
	if ((found[0] != -1) && (found[1] != -1) && (found[3] != -1) &&
		(abs(found[0]-found[1]) <= 1) && (abs(found[0]-found[3]) <= 1))
	{
		AddEdge(neighborhood[4], neighborhood[0]);
		if (abs(found[1]-found[3]) <= 1)
			AddEdge(neighborhood[1], neighborhood[3]);
	}

	if ((found[1] != -1) && (found[2] != -1) && (found[5] != -1) &&
		(abs(found[1]-found[2]) <= 1) && (abs(found[2]-found[5]) <= 1))
	{
		AddEdge(neighborhood[4], neighborhood[2]);
		if (abs(found[1]-found[5]) <= 1)
			AddEdge(neighborhood[1], neighborhood[5]);
	}

	if ((found[3] != -1) && (found[6] != -1) && (found[7] != -1) &&
		(abs(found[3]-found[6]) <= 1) && (abs(found[6]-found[7]) <= 1))
	{
		AddEdge(neighborhood[4], neighborhood[6]);
		if (abs(found[3]-found[7]) <= 1)
			AddEdge(neighborhood[3], neighborhood[7]);
	}

	if ((found[5] != -1) && (found[8] != -1) && (found[7] != -1) &&
		(abs(found[5]-found[8]) <= 1) && (abs(found[8]-found[7]) <= 1))
	{
		AddEdge(neighborhood[4], neighborhood[8]);
		if (abs(found[5]-found[7]) <= 1)
			AddEdge(neighborhood[5], neighborhood[7]);
	}
	
	return result;
}

void Map3DGrid::AddEdge(int sec1, int reg1, int sec2, int reg2, int weight)
{
//	printf("Adding edge between sec/reg %d/%d and %d/%d\n", sec1, reg1, sec2, reg2);
	assert(sectors.size() > sec1);
	assert(sectors.size() > sec2);
	assert(reg1 != -1);
	assert(reg2 != -1);
	EdgeData e1(sec1, reg1, weight);
	EdgeData e2(sec2, reg2, weight);
	sectors[sec1].regions[reg1].AddEdge(e2);
	sectors[sec2].regions[reg2].AddEdge(e1);
}

void Map3DGrid::AddEdge(state3d &from, state3d &to)
{
	assert(from.GetOffset() != -1);
	assert(from.GetSector() != -1);
	assert(from.GetRegion() != -1);
	assert(to.GetOffset() != -1);
	assert(to.GetSector() != -1);
	assert(to.GetRegion() != -1);
	if ((from.sector == to.sector) && (from.region == to.region))
	{
		AddGridEdge(from, to, true);
		return;
	}
	else if (from.sector == to.sector) // but different regions
	{
//		printf("Adding between %d/%d/%d and %d/%d/%d\n",
//			   from.GetSector(), from.GetRegion(), from.GetOffset(),
//			   to.GetSector(), to.GetRegion(), to.GetOffset());
		int cnt = AddGridEdge(from, to, false);
		AddEdge(from.sector, from.region, to.sector, to.region, cnt);
	}
	else {
//		printf("Adding between %d/%d/%d and %d/%d/%d\n",
//			   from.GetSector(), from.GetRegion(), from.GetOffset(),
//			   to.GetSector(), to.GetRegion(), to.GetOffset());
		int cnt = AddSectorEdge(from, to);
		AddEdge(from.sector, from.region, to.sector, to.region, cnt);
	}
}

int Map3DGrid::AddGridEdge(state3d &from, state3d &to, bool local)
{
	int val = from.offset-to.offset;
	if (val == 1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kWest, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kEast, local);
	if (val ==  -1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kEast, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kWest, local);
	if (val ==  gSectorSize)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorth, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouth, local);
	if (val ==  -gSectorSize)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouth, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorth, local);
	// diagonal moves
	if (val ==  gSectorSize+1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorthWest, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouthEast, local);
	if (val ==  gSectorSize-1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorthEast, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouthWest, local);
	if (val ==  -gSectorSize+1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouthWest, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorthEast, local);
	if (val ==  -gSectorSize-1)
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouthEast, local)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorthWest, local);
	return 0;
}

int Map3DGrid::AddSectorEdge(state3d &from, state3d &to)
{
	int x1, y1, x2, y2;
	GetXYFromState(from, x1, y1);
	GetXYFromState(to, x2, y2);
	int offsetDiff = (y1*mWidth+x1)-(y2*mWidth+x2);
	// cardinal moves
	if (offsetDiff == 1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kWest, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kEast, false);
	}
	else if (offsetDiff == -1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kEast, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kWest, false);
	}
	else if (offsetDiff == mWidth)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorth, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouth, false);
	}
	else if (offsetDiff == -mWidth)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouth, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorth, false);
	}
	// diagonal moves
	else if (offsetDiff == mWidth+1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorthWest, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouthEast, false);
	}
	else if (offsetDiff == mWidth-1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kNorthEast, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kSouthWest, false);
	}
	else if (offsetDiff == -mWidth+1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouthWest, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorthEast, false);
	}
	else if (offsetDiff == -mWidth-1)
	{
		return sectors[from.sector].regions[from.region].grid.AddMove(from.offset, kSouthEast, false)+
		sectors[to.sector].regions[to.region].grid.AddMove(to.offset, kNorthWest, false);
	}
	return 0;
}

bool Map3DGrid::RemovePoint(int x, int y, int z, bool split)
{
	int theRegion = InternalRemovePoint(x, y, z);
	if (theRegion == -1)
		return false;
	
	if (!split)
		return true;
	
	// now check if we need to split the sector
	static std::vector<int> labels;
	static std::vector<int> counts;
	static std::vector<int> heights;
	int regions = sectors[GetSector(x, y)].regions[theRegion].grid.LabelAreas(labels, counts, heights);
	if (regions > 1)
	{
		printf("%d regions; split needed\n", regions);
		for (int t = 0; t < gSectorSize; t++)
		{
			for (int u = 0; u < gSectorSize; u++)
			{
				printf("%2d", labels[t*gSectorSize+u]);
			}
			printf("\n");
		}
		int basex = x-(x%gSectorSize);
		int basey = y-(y%gSectorSize);
		while (regions > 1)
		{
			int smallest = 0;
			// find smallest region
			for (int t = 1; t < counts.size(); t++)
			{
				if (counts[t] < counts[smallest])
					smallest = t;
			}
			
			// remove points from region
			for (unsigned int t = 0; t < gSectorSize*gSectorSize; t++)
			{
				if (labels[t] == smallest)
					InternalRemovePoint(basex+t%gSectorSize, basey + t/gSectorSize, heights[t]);
			}
			
			// re-add (will appear in new region)
			for (unsigned int t = 0; t < gSectorSize*gSectorSize; t++)
			{
				if (labels[t] == smallest)
					AddPoint(basex+t%gSectorSize, basey + t/gSectorSize, heights[t]);
			}
			regions--;
		}
	}
	return true;
}

// returns region of x/y/z point
int Map3DGrid::InternalRemovePoint(int x, int y, int z)
{
	// first remove all edges; THEN remove the actual point
	state3d neighborhood[9];
	int found[9];
	
	
	found[0] = FindNearState(x-1, y-1, z, neighborhood[0]);
	found[1] = FindNearState(x  , y-1, z, neighborhood[1]);
	found[2] = FindNearState(x+1, y-1, z, neighborhood[2]);

	found[3] = FindNearState(x-1, y, z, neighborhood[3]);
	found[4] = FindNearState(x  , y, z, neighborhood[4]);
	found[5] = FindNearState(x+1, y, z, neighborhood[5]);

	found[6] = FindNearState(x-1, y+1, z, neighborhood[6]);
	found[7] = FindNearState(x  , y+1, z, neighborhood[7]);
	found[8] = FindNearState(x+1, y+1, z, neighborhood[8]);
	
	if (found[4] == -1)
		return -1;
	//assert(found[4] != -1);
	
	// connect horizontals
	if (found[1] != -1)
		RemoveEdge(neighborhood[4], neighborhood[1]);
	if (found[3] != -1)
		RemoveEdge(neighborhood[4], neighborhood[3]);
	if (found[5] != -1)
		RemoveEdge(neighborhood[4], neighborhood[5]);
	if (found[7] != -1)
		RemoveEdge(neighborhood[4], neighborhood[7]);
	
	// connect diagonals
	if ((found[0] != -1) && (found[1] != -1) && (found[3] != -1) &&
		(abs(found[0]-found[1]) <= 1) && (abs(found[0]-found[3]) <= 1))
	{
		RemoveEdge(neighborhood[4], neighborhood[0]);
		if (abs(found[1]-found[3]) <= 1)
			RemoveEdge(neighborhood[1], neighborhood[3]);
	}
	
	if ((found[1] != -1) && (found[2] != -1) && (found[5] != -1) &&
		(abs(found[1]-found[2]) <= 1) && (abs(found[2]-found[5]) <= 1))
	{
		RemoveEdge(neighborhood[4], neighborhood[2]);
		if (abs(found[1]-found[5]) <= 1)
			RemoveEdge(neighborhood[1], neighborhood[5]);
	}
	
	if ((found[3] != -1) && (found[6] != -1) && (found[7] != -1) &&
		(abs(found[3]-found[6]) <= 1) && (abs(found[6]-found[7]) <= 1))
	{
		RemoveEdge(neighborhood[4], neighborhood[6]);
		if (abs(found[3]-found[7]) <= 1)
			RemoveEdge(neighborhood[3], neighborhood[7]);
	}
	
	if ((found[5] != -1) && (found[8] != -1) && (found[7] != -1) &&
		(abs(found[5]-found[8]) <= 1) && (abs(found[8]-found[7]) <= 1))
	{
		RemoveEdge(neighborhood[4], neighborhood[8]);
		if (abs(found[5]-found[7]) <= 1)
			RemoveEdge(neighborhood[5], neighborhood[7]);
	}
	
	// actually remove point
	sectors[GetSector(x, y)].RemovePoint(x, y, z);

	sectors[neighborhood[4].GetSector()].regions[neighborhood[4].GetRegion()].RedoCenterLocation();
	return neighborhood[4].GetRegion();
}

void Map3DGrid::RemoveEdge(int sec1, int reg1, int sec2, int reg2, int weight)
{
	EdgeData e1(sec1, reg1, weight);	
	EdgeData e2(sec2, reg2, weight);	
	sectors[sec1].regions[reg1].RemoveEdge(e2);
	sectors[sec2].regions[reg2].RemoveEdge(e1);
}

void Map3DGrid::RemoveEdge(state3d &from, state3d &to)
{
	if ((from.sector == to.sector) && (from.region == to.region))
	{
		RemoveGridEdge(from, to);
		return;
	}
	else if (from.sector == to.sector) // but different regions
	{
		int cnt = RemoveGridEdge(from, to);
		RemoveEdge(from.sector, from.region, to.sector, to.region, cnt);
	}
	else {
		int cnt = RemoveSectorEdge(from, to);
		RemoveEdge(from.sector, from.region, to.sector, to.region, cnt);
	}
}

int Map3DGrid::RemoveGridEdge(state3d &from, state3d &to)
{
	int val = from.offset-to.offset;
	if (val == 1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kEast);
	if (val == -1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kWest);
	if (val == gSectorSize)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorth)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouth);
	if (val == -gSectorSize)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouth)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorth);
	// diagonal moves
	if (val == gSectorSize+1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorthWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouthEast);
	if (val == gSectorSize-1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorthEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouthWest);
	if (val == -gSectorSize+1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouthWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorthEast);
	if (val == -gSectorSize-1)
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouthEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorthWest);
	assert(false);
	return 0;

}

int Map3DGrid::RemoveSectorEdge(state3d &from, state3d &to)
{
	int x1, y1, x2, y2;
	GetXYFromState(from, x1, y1);
	GetXYFromState(to, x2, y2);
	int offsetDiff = (y1*mWidth+x1)-(y2*mWidth+x2);
	// cardinal moves
	if (offsetDiff == 1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kEast);
	}
	else if (offsetDiff == -1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kWest);
	}
	else if (offsetDiff == mWidth)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorth)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouth);
	}
	else if (offsetDiff == -mWidth)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouth)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorth);
	}
	// diagonal moves
	else if (offsetDiff == mWidth+1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorthWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouthEast);
	}
	else if (offsetDiff == mWidth-1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kNorthEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kSouthWest);
	}
	else if (offsetDiff == -mWidth+1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouthWest)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorthEast);
	}
	else if (offsetDiff == -mWidth-1)
	{
		return sectors[from.sector].regions[from.region].grid.ReMove(from.offset, kSouthEast)+
		sectors[to.sector].regions[to.region].grid.ReMove(to.offset, kNorthWest);
	}
	assert(false);
	return 0;
}


int Map3DGrid::FindNearState(int x, int y, int z, state3d &s) const
{
	if ((x >= mWidth) || (x < 0) || (y >= mHeight) || (y < 0))
		return -1;
	int sect = GetSector(x, y);
	for (unsigned t = 0; t < sectors[sect].regions.size(); t++)
	{
		int height = sectors[sect].regions[t].grid.GetHeightOffset(x%gSectorSize, y%gSectorSize);
		if ((height != kUnpassableHeight) &&
			abs(z-(height+sectors[sect].regions[t].baseHeight)) < 2)
		{
			s.Init(sect, t, indexFromXY(x%gSectorSize, y%gSectorSize));
			//printf("Found (%d, %d, %d) (%d)\n", x, y, z, height);
			return height+sectors[sect].regions[t].baseHeight;
		}
	}
	//printf("\n");
	return -1;
}

void Map3DGrid::GetXYFromState(const state3d &s, int &x, int &y) const
{
	int regx, regy;
	xyFromIndex(s.GetOffset(), regx, regy);
	x = (s.GetSector()%mXSectors)*gSectorSize+regx;
	y = (s.GetSector()/mXSectors)*gSectorSize+regy; 
}

void Map3DGrid::GetXYZFromState(const state3d &s, int &x, int &y, int &z) const
{
	GetXYFromState(s, x, y);
	z = sectors[s.GetSector()].regions[s.GetRegion()].baseHeight+
	sectors[s.GetSector()].regions[s.GetRegion()].grid.GetHeightOffset(s.GetOffset());
}

void Map3DGrid::GetStateFromXYZ(state3d &s, int x, int y, int z) const
{
	// TODO: this is only for 2D worlds
//	s.sector = GetSector(x, y);
//	s.region = 0;
//	s.offset = (x%gSectorSize)+(y%gSectorSize)*gSectorSize;
//	return;
	int val = FindNearState(x, y, z, s);
	assert(val != -1);
}

void Map3DGrid::GetPointFromCoordinate(point3d loc, int &px, int &py, int &pz) const
{
	int xOffset = max(mHeight, mWidth)-mWidth;
	int yOffset = max(mHeight, mWidth)-mHeight;
	double scale = max(mHeight, mWidth);
	scale = 1.0/scale;

	px = (loc.x-(-1+scale+xOffset*scale))/(2.0*scale);
	py = (loc.y-(-1+scale+yOffset*scale))/(2.0*scale);
	pz = -loc.z/(scale*0.1);
}


//void Map3DGrid::OpenGLDraw() const
//{
//	glColor4f(0.0, 1.0, 0.0, 1.0);
////	glBegin(GL_LINE_LOOP);
////	glVertex3f(-1, -1, 0);
////	glVertex3f(-1,  1, 0);
////	glVertex3f( 1,  1, 0);
////	glVertex3f( 1, -1, 0);
////	glEnd();
//	GLdouble x1, y1, z1, rad, h;
////	GLdouble x2, y2, z2, rad2, h2;
//	GLdouble left, right, top, bottom;
//	GetCoord(0, 0, 0, left, top, z1, rad, h);
//	GetCoord(mWidth, mHeight, 0, right, bottom, z1, rad, h);
//	glBegin(GL_LINES);
//	for (int x = 0; x < mWidth; x+=gSectorSize)
//	{
//		GetCoord(x, 0, 0, x1, y1, z1, rad, h);
//		glVertex3f(x1-rad, top, z1);
//		glVertex3f(x1-rad, bottom, z1);
//	}
//	GetCoord(mWidth, 0, 0, x1, y1, z1, rad, h);
//	glVertex3f(x1-rad, top, z1);
//	glVertex3f(x1-rad, bottom, z1);
//
//	for (int y = 0; y < mHeight; y+=gSectorSize)
//	{
//		GetCoord(0, y, 0, x1, y1, z1, rad, h);
//		glVertex3f(left, y1-rad, z1);
//		glVertex3f(right,  y1-rad, z1);
//	}
//	GetCoord(0, mHeight, 0, x1, y1, z1, rad, h);
//	glVertex3f(left, y1-rad, z1);
//	glVertex3f(right,  y1-rad, z1);
//	glEnd();
//
//	glColor3f(0, 0, 0);
//	glBegin(GL_QUADS);
//	glVertex3f(-1, -1, 0.01);
//	glVertex3f( 1, -1, 0.01);
//	glVertex3f( 1,  1, 0.01);
//	glVertex3f(-1,  1, 0.01);
//	glEnd();
//	
//	glEnable(GL_LIGHTING);
//	for (unsigned int x = 0; x < sectors.size(); x++)
//	{
//		//printf("%d regions in sector %d\n", sectors[x].regions.size(), x);
//		for (unsigned int y = 0; y < sectors[x].regions.size(); y++)
//		{
//			//SetColor(0.7, 0.7, 0.7, 0.7);
//			SetColor(1.0, 1.0, 1.0, 1.0);
//			glTranslatef(0, 0, -5*h);
//			for (unsigned int z = 0; z < sectors[x].regions[y].edges.size(); z++)
//			{
//				state3d s1, s2;
//				s1.Init(x, y, sectors[x].regions[y].centerLocation);
//				s2.Init(sectors[x].regions[y].edges[z].sector,
//						sectors[x].regions[y].edges[z].region,
//						sectors[sectors[x].regions[y].edges[z].sector].regions[sectors[x].regions[y].edges[z].region].centerLocation);
//				glLineWidth(2+sectors[x].regions[y].edges[z].support/4);
//				GLDrawLine(s1, s2);
//			}
//			glTranslatef(0, 0, 5*h);
//			glLineWidth(1.0);
//
//			GLdouble r=0, g=0, b=0;
//			switch ((y+x)%4)
//			{
//				case 0: g=0.25;r=0.5;b=0.15; break;
//				case 1: g=0.35;r=0.5;b=0.15; break;
//				case 2: g=0.25;r=0.4;b=0.15; break;
//				case 3: g=0.25;r=0.5;b=0.25; break;
//			}
//			for (unsigned int z = 0; z < gSectorSize*gSectorSize; z++)
//			{
//				int xLoc, yLoc, zLoc;
//				int xRegLoc, yRegLoc;
//				xyFromIndex(z, xLoc, yLoc);
//				xRegLoc = xLoc;
//				yRegLoc = yLoc;
//				if (!sectors[x].regions[y].grid.IsPointPassable(xRegLoc, yRegLoc))
//					continue;
//				
//				zLoc = sectors[x].regions[y].baseHeight + sectors[x].regions[y].grid.GetHeightOffset(xRegLoc, yRegLoc);
//				xLoc += (x%mXSectors)*gSectorSize;
//				yLoc += (x/mXSectors)*gSectorSize;
//				GetCoord(xLoc, yLoc, zLoc, x1, y1, z1, rad, h);
//				glColor3f(r, g, b);
//				glBegin(GL_QUADS);
//				glNormal3f(0, 0, -1);
//				glVertex3f(x1-rad, y1-rad, z1);
//				glVertex3f(x1-rad, y1+rad, z1);
//				glVertex3f(x1+rad, y1+rad, z1);
//				glVertex3f(x1+rad, y1-rad, z1);
//				glEnd();
//				
//				if (drawGrid)
//				{
//					std::vector<state3d> suc;
//					state3d currState;
//					currState.Init(x, y, indexFromXY(xRegLoc, yRegLoc));
//					GetSuccessors(currState, suc);
//					for (unsigned int t = 0; t < suc.size(); t++)
//					{
//	//					printf("Drawing line between (%d/%d/%d) and (%d/%d/%d)\n", 
//	//						   currState.GetSector(), currState.GetRegion(), currState.GetOffset(),
//	//						   suc[t].GetSector(), suc[t].GetRegion(), suc[t].GetOffset());
//						SetColor(1, 1, 1, 1);
//						GLDrawLine(currState, suc[t]);
//					}
//				}
//			}
//		}
//	}
//}
//
//
//void Map3DGrid::OpenGLDraw(const state3d &s) const
//{
////	s.part1
//}
//
//void Map3DGrid::GLDrawLine(const state3d &a, const state3d &b) const
//{
//	GLdouble x1, y1, z1, rad, h;
//	GLfloat rr, gg, bb, tt;
//	int xLoc, yLoc, zLoc;
//
//	GetXYZFromState(a, xLoc, yLoc, zLoc);
//	GetCoord(xLoc, yLoc, zLoc, x1, y1, z1, rad, h);
//
//	//glLineWidth(2.0);
//	glDisable(GL_LIGHTING);
//	GetColor(rr, gg, bb, tt);
//	glColor4f(rr, gg, bb, tt);
//	glBegin(GL_LINES);
//	glVertex3f(x1, y1, z1-h);
//
//	GetXYZFromState(b, xLoc, yLoc, zLoc);
//	GetCoord(xLoc, yLoc, zLoc, x1, y1, z1, rad, h);
//	
//	glVertex3f(x1, y1, z1-h);
//	glEnd();
//	glEnable(GL_LIGHTING);
//	//glLineWidth(1.0);
//}
//
//
//void Map3DGrid::GetCoord(int xLoc, int yLoc, int zLoc,
//							   GLdouble &x, GLdouble &y, GLdouble &z,
//							   GLdouble &r, GLdouble &h) const
//{
//	int xOffset = max(mHeight, mWidth)-mWidth;
//	int yOffset = max(mHeight, mWidth)-mHeight;
//	double scale = max(mHeight, mWidth);
//	scale = 1.0/scale;
//	x = 2.0*xLoc*scale-1+scale+xOffset*scale;
//	y = 2.0*yLoc*scale-1+scale+yOffset*scale;
//	h = scale*0.2;
//	z = -zLoc*h;
//	r = scale;
//}

void Map3DGrid::PrintStats()
{
	std::vector<int> regCnt;
	int regions = 0;
	printf("%lu sectors\n", sectors.size());
	for (unsigned int x = 0; x < sectors.size(); x++)
	{
		if (sectors[x].regions.size() >= regCnt.size())
			regCnt.resize(sectors[x].regions.size()+1);
		regCnt[sectors[x].regions.size()]++;
		regions += sectors[x].regions.size();
	}
	printf("%d regions [avg: %1.2f]\n", regions, (float)regions/sectors.size());
	for (unsigned int x = 0; x < regCnt.size(); x++)
	{
		printf("%d : %d\n", x, regCnt[x]);
	}
}

int Map3DGrid::GetAbstractionBytesUsed()
{
	int mem = sectors.size() * 4;

	for (unsigned int x = 0; x < sectors.size(); x++)
	{
		mem += sectors[x].regions.size()*12;
		for (int y = 0; y < sectors[x].regions.size(); y++)
			mem += sectors[x].regions[y].edges.size()*4;
	}
	return mem;
}

int Map3DGrid::GetGridBytesUsed()
{
	int mem = 0;
	
	for (unsigned int x = 0; x < sectors.size(); x++)
	{
		mem += sectors[x].regions.size()*gSectorSize*gSectorSize*2;
	}
	return mem;
}
