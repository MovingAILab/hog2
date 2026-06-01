/*
 * MinimalSectorAbstraction.h
 *
 * Copyright (c) 2007, Nathan Sturtevant
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Alberta nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NATHAN STURTEVANT ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef _MSC_VER
#include "stdafx.h"
#else
#include <stdint.h>
#endif

#include <vector>
#include "Map.h"
#include "Graphics.h"

#ifndef MINIMALSECTORABSTRACTION_H
#define MINIMALSECTORABSTRACTION_H

const int maxNumParents    = 14;
const int parentBits       =  4;
//const int sectorSize       = 16;
///extern int sectorSize;
const int sectorOffsetBits =  4;
const int maxNumEdges      =  7;

// each abstract node needs 32 bits...2^16 memory fixed = 64k
// number of parents/regions (8 bits)
// initial address in memory (16 bits)
// total number of edges (8 bits)

// each parent has (16 bits)
// sectorOffsetBits = 8 -- this is 0..16 the square in the sector
// edge end         = 8

// each edge has (8 bits)
// direction           = 2
// parent num          = 6

// pad everything to the word boundary
// abstraction size info = 2*parents + edges + (4-(2*parents + edges)%4)

struct sectorInfo {
	uint8_t numRegions;
	uint8_t numEdges;
	uint16_t memoryAddress;
};

struct tempEdgeData {
	int from, to, direction;
};

inline bool operator==(const tempEdgeData& x, const tempEdgeData& y)
{
	return ((x.to == y.to) && (x.from == y.from) && (x.direction == y.direction));
}

/**
 * MinimalSectorAbstraction
 *
 * \brief Builds a map abstraction
 *
 * Builds a map abstraction as described in AI Game Programming Wisdom 4.
 *
 * The abstraction is optimized to use relatively little memory.
 * This is the first version of the abstraction that I wrote.
 * The code could be a bit cleaner...but it works
 */
class MinimalSectorAbstraction {
public:
	MinimalSectorAbstraction(Map *map, int sectorSize);
//	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	int GetSector(int x, int y);
	int GetRegion(int x, int y);
	void GetXYLocation(unsigned int sector, unsigned int region,
					   unsigned int &x, unsigned int &y) const;
	void GetNeighbors(unsigned int sector, unsigned int region,
					  std::vector<tempEdgeData> &edges) const;
	int GetAdjacentSector(unsigned int sector, int direction);
	
	void OptimizeRegionLocations();
	void InitializeOptimization();
	bool PerformOneOptimizationStep();
	int GetAbstractionBytesUsed() { return sectors.size()*4+memory.size(); }
private:
	void BuildAbstraction();
	void GetEdges(std::vector<std::vector<int> > &areas,
				  int xSector, int ySector,
				  std::vector<tempEdgeData> &edges);
	
	void GetEdgeHelper(std::vector<int> &startRegion,
					   int startIndex, int startOffset,
					   std::vector<int> &targetRegion,
					   int tarGetIndex, int targetOffset,
					   std::vector<tempEdgeData> &edges,
					   int direction);
	int GetSectorRegions(std::vector<int> &area,
						 int absXSector,
						 int absYSector);
	void LabelRegion(std::vector<int> &area,
					 int x, int y, int label);
	
	void StoreSectorInMemory(sectorInfo &si,
							 std::vector<int> &area,
							 std::vector<tempEdgeData> &edges);
	uint8_t GetAbstractEdge(tempEdgeData &data);
	uint8_t GetAbstractLocation(std::vector<int> area, int value);
	int FindParentRegion(int startLoc,
						 std::vector<int> &parents,
						 int mapXOffset, int mapYOffset);
	double GetRegionError(int fromSector, int fromRegion,
						  int sx, int sy, double limit);
	void MoveRegionCenter(int sector, int region);
	void ComputePotentialMemorySavings();
	void ResetAbstractCenter(int sector, int region);
	
	int numXSectors, numYSectors;
	std::vector<sectorInfo> sectors;
	std::vector<uint8_t> memory;
	std::vector<std::vector<double> > regionError;
	Map *map;
	std::vector<std::vector<int> > areas;
	int sectorSize;
	int optimizationIndex;
};

#endif
