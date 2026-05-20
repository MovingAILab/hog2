/*
 * MinimalSectorAbstraction.cpp
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

#include "MinimalSectorAbstraction.h"
#include <queue>
//#include "GenericAStar.h"
#include "Map2DEnvironment.h"

//int sectorSize = 11;

#define DIAG_MOVES

#ifndef _MSC_VER
#define max(a, b) (((a)>(b))?(a):(b))
#endif

/**
 * MinimalSectorAbstraction::MinimalSectorAbstraction()
 *
 * \brief Constructor for minimal sector abstraction
 *
 * \param m The map for which the abstraction is created
 * \return none
 */

MinimalSectorAbstraction::MinimalSectorAbstraction(Map *m, int theSectorSize)
:map(m)
{
	sectorSize = theSectorSize;
    numYSectors = ((map->GetMapHeight()+sectorSize-1)/sectorSize);
    numXSectors = ((map->GetMapWidth()+sectorSize-1)/sectorSize);
    int absSize = numXSectors*numYSectors;
    printf("%d x sectors, %d y sectors\n", numXSectors, numYSectors);
    sectors.resize(absSize);
    BuildAbstraction();
    optimizationIndex = (int)sectors.size();
}

/**
 * MinimalSectorAbstraction::BuildAbstraction()
 *
 * \brief Start the abstraction buildling process.
 *
 * The abstraction is built in 3 steps. First, we identify the regions
 * in each sector. Then, we find the edges between each region.
 * Finally we store a compact representation of the abstraction.
 * An optimization phase can optionally be run.
 *
 * As this isn't production code, we don't actually free any of the
 * memory allocated during the construction of the abstraction.
 *
 * \param none
 * \return none
 */
void MinimalSectorAbstraction::BuildAbstraction()
{
    areas.clear();
    areas.resize(numXSectors*numYSectors);
    for (int x = 0; x < numXSectors; x++)
    {
        for (int y = 0; y < numYSectors; y++)
        {
            int numRegions = GetSectorRegions(areas[y*numXSectors+x], x, y);
            assert(numRegions < 256);
            sectors[y*numXSectors+x].numRegions = numRegions;
        }
    }
    
    for (int x = 0; x < numXSectors; x++)
    {
        for (int y = 0; y < numYSectors; y++)
        {
            std::vector<tempEdgeData> edges;
            GetEdges(areas, x, y, edges);
            assert(edges.size() < 256);
            sectors[y*numXSectors+x].numEdges = (uint8_t)edges.size();
            StoreSectorInMemory(sectors[y*numXSectors+x], areas[y*numXSectors+x], edges);
        }
    }
    printf("Total bytes used for abstraction: %d\n", (int)memory.size());
    printf("Extra memory used: %d\n", (int)(sectors.size()*sizeof(sectorInfo)));
    int nodes = 0, edges = 0;
    for (unsigned int x = 0; x < sectors.size(); x++)
    {
        nodes += sectors[x].numRegions;
        edges += sectors[x].numEdges;
    }
    printf("%d regions and %d edges\n", nodes, edges);
    //OptimizeRegionLocations();
    ComputePotentialMemorySavings();
}

/**
 * MinimalSectorAbstraction::GetEdges()
 *
 * \brief Check for edges in and out of a sector.
 *
 * This is the high-level code which checks for edges between sectors
 *
 * \param areas An array which marks the region for each piece of the map
 * \param xSector The x-sector which we are checking
 * \param ySector The y-sector which we are checking
 * \param edges A vector of edge data holding the edges from this particular region
 * \return none
 */
void MinimalSectorAbstraction::GetEdges(std::vector<std::vector<int> > &areas,
                                        int xSector, int ySector,
                                        std::vector<tempEdgeData> &edges)
{
    bool up = false, down = false, left = false, right = false;
    if (ySector > 0) // up
    {
        up = true;
        // get edge helper does the actual work of finding edges between adjacent sectors
        GetEdgeHelper(areas[xSector + numXSectors*ySector], 0, 1,
                      areas[xSector + numXSectors*(ySector-1)], sectorSize*(sectorSize-1), 1,
                      edges, 0);
    }
    if (xSector > 0) // left
    {
        left = true;
        GetEdgeHelper(areas[xSector + numXSectors*ySector], 0, sectorSize,
                      areas[(xSector-1) + numXSectors*ySector], sectorSize-1, sectorSize,
                      edges, 3);
    }
    if (ySector < numYSectors-1) // down
    {
        down = true;
        GetEdgeHelper(areas[xSector + numXSectors*ySector], sectorSize*(sectorSize-1), 1,
                      areas[xSector + numXSectors*(ySector+1)], 0, 1,
                      edges, 2);
    }
    if (xSector < numXSectors-1) // right
    {
        right = true;
        GetEdgeHelper(areas[xSector + numXSectors*ySector], sectorSize-1, sectorSize,
                      areas[(xSector+1) + numXSectors*ySector], 0, sectorSize,
                      edges, 1);
    }
#ifdef DIAG_MOVES
    // for diagonal moves, we directly check for diagonal moves -- all four points
    // on the corner of the sector must be free to allow an edge to be created.
    if (up && right)
    {
        if ((areas[xSector + numXSectors*(ySector)][sectorSize-1] != -1) && // current
            (areas[xSector + numXSectors*(ySector-1)][sectorSize*sectorSize-1] != -1) && // up
            (areas[xSector + 1 + numXSectors*(ySector-1)][sectorSize*(sectorSize-1)] != -1) && // up right
            (areas[xSector + 1 + numXSectors*(ySector)][0] != -1)) // right
        {
            tempEdgeData ted;
            ted.from = areas[xSector + numXSectors*(ySector)][sectorSize-1];
            ted.to = areas[xSector + 1 + numXSectors*(ySector-1)][sectorSize*(sectorSize-1)];
            ted.direction = 4;
            edges.push_back(ted);
            //            printf("Adding diagonal edge between %d:(%d, %d) and %d:(%d, %d)\n",
            //                         ted.from, xSector, ySector, ted.to, xSector+1, ySector-1);
        }
    }
    if (up && left)
    {
        if ((areas[xSector + numXSectors*(ySector)][0] != -1) && // current
            (areas[xSector + numXSectors*(ySector-1)][sectorSize*(sectorSize-1)] != -1) && // up
            (areas[xSector - 1 + numXSectors*(ySector-1)][sectorSize*sectorSize-1] != -1) && // up left
            (areas[xSector - 1 + numXSectors*(ySector)][sectorSize-1] != -1)) // left
        {
            tempEdgeData ted;
            ted.from = areas[xSector + numXSectors*(ySector)][0];
            ted.to = areas[xSector - 1 + numXSectors*(ySector-1)][sectorSize*sectorSize-1];
            ted.direction = 7;
            edges.push_back(ted);
            //            printf("Adding diagonal edge between %d:(%d, %d) and %d:(%d, %d)\n",
            //                         ted.from, xSector, ySector, ted.to, xSector-1, ySector-1);
        }
    }
    if (down && left)
    {
        if ((areas[xSector + numXSectors*(ySector)][sectorSize*(sectorSize-1)] != -1) && // current
            (areas[xSector + numXSectors*(ySector+1)][0] != -1) && // down
            (areas[xSector - 1 + numXSectors*(ySector+1)][sectorSize-1] != -1) && // down left
            (areas[xSector - 1 + numXSectors*(ySector)][sectorSize*sectorSize-1] != -1)) // left
        {
            tempEdgeData ted;
            ted.from = areas[xSector + numXSectors*(ySector)][sectorSize*(sectorSize-1)];
            ted.to = areas[xSector - 1 + numXSectors*(ySector+1)][sectorSize-1];
            ted.direction = 6;
            edges.push_back(ted);
            //            printf("Adding diagonal edge between %d:(%d, %d) and %d:(%d, %d)\n",
            //                         ted.from, xSector, ySector, ted.to, xSector-1, ySector+1);
        }
    }
    if (down && right)
    {
        if ((areas[xSector + numXSectors*(ySector)][sectorSize*sectorSize-1] != -1) && // current
            (areas[xSector + numXSectors*(ySector+1)][sectorSize-1] != -1) && // down
            (areas[xSector + 1 + numXSectors*(ySector+1)][0] != -1) && // down right
            (areas[xSector + 1 + numXSectors*(ySector)][sectorSize*(sectorSize-1)] != -1)) // right
        {
            tempEdgeData ted;
            ted.from = areas[xSector + numXSectors*(ySector)][sectorSize*sectorSize-1];
            ted.to = areas[xSector + 1 + numXSectors*(ySector+1)][0];
            ted.direction = 5;
            edges.push_back(ted);
            //            printf("Adding diagonal edge between %d:(%d, %d) and %d:(%d, %d)\n",
            //                         ted.from, xSector, ySector, ted.to, xSector+1, ySector+1);
        }
    }
#endif
}

/**
 * MinimalSectorAbstraction::GetEdgeHelper()
 *
 * \brief Generic procedure for iterating along the edge of a sector checking
 * for edges.
 *
 * GetEdgeHelper takes two (assumed) adjacent sectors and checks those sectors
 * for all edges between them. For each sector a startIndex and offset is used
 * so that the same code can be used for any two edges. Checks begin between
 * the startIndex and targetIndex and at each step the indices are updated by
 * the offset.
 *
 * \param startRegion This vector holds the region data for the starting sector
 * \param startIndex The first index to check in the startRegion
 * \param startOffset The offset (step) between indices we want to compare
 * \param targetRegion The region data for the goal sector
 * \param targetIndex The first index to check in the target region
 * \param targetOffset The offset (step) between indices we want to compare
 * \param edges The storage for any edges we find
 * \param direction The direction that all edges will go
 * \return none
 */
void MinimalSectorAbstraction::GetEdgeHelper(std::vector<int> &startRegion,
                                             int startIndex, int startOffset,
                                             std::vector<int> &targetRegion,
                                             int targetIndex, int targetOffset,
                                             std::vector<tempEdgeData> &edges,
                                             int direction)
{
    int first = (int)edges.size();
    int last = -1;
    for (int x = 0; x < sectorSize; x++)
    {
        if ((startRegion[startIndex] != -1) &&
            (targetRegion[targetIndex] != -1))
        {
            if ((last == -1) ||
                (edges[edges.size()-1].from != startRegion[startIndex]) ||
                (edges[edges.size()-1].to   != targetRegion[targetIndex]))
            {
                bool here = false;
                tempEdgeData ted;
                ted.from = startRegion[startIndex];
                ted.to = targetRegion[targetIndex];
                ted.direction = direction;
                for (unsigned int y = first; y < edges.size(); y++)
                {
                    if (edges[y] == ted)
                    {
                        here = true;
                        break;
                    }
                }
                if (!here)
                {
                    edges.push_back(ted);
                    last = (int)edges.size()-1;
                }
            }
        }
        startIndex += startOffset;
        targetIndex += targetOffset;
    }
}

/**
 * MinimalSectorAbstraction::StoreSectorInMemory()
 *
 * \brief Store a sector and its edges/regions into the compressed data structure
 *
 * Store a sector into the compressed representation. Takes the edges
 * already computed and also computes the center of each region.
 *
 * \param si The data structure with the high-level sector information
 * \param area The region we are storing information for
 * \param edges a list of edges from this sector
 * \return none
 */
void MinimalSectorAbstraction::StoreSectorInMemory(sectorInfo &si,
                                                   std::vector<int> &area,
                                                   std::vector<tempEdgeData> &edges)
{
    // count number of edges for each region
    std::vector<int> counts(si.numRegions);
    for (unsigned int y = 0; y < counts.size(); y++)
    {
        for (unsigned int x = 0; x < edges.size(); x++)
            if (edges[x].from == (int)y+1)
                counts[y]++;
    }
    
    // now we know the size needed to store this, and
    // we can compute the rest of the abstraction info
    assert(memory.size() < (1<<16));
    si.memoryAddress = (uint16_t)memory.size();
    int sum = 0;
    for (int x = 0; x < si.numRegions; x++)
    {
        sum += counts[x];
        // regions are numbered 0..n, but they are labelled 1..n+1
        // inside the area information
        memory.push_back(GetAbstractLocation(area, x+1));
        memory.push_back(sum); // last edge of this parent
    }
    //    printf("Sum is %d, total is %d\n", sum, si.numEdges);
    assert(sum == si.numEdges);
    // Add all the edges;
    // this can be done faster, but I'm lazy
    for (unsigned int y = 0; y < counts.size(); y++)
    {
        for (unsigned int x = 0; x < edges.size(); x++)
        {
            if (edges[x].from == (int)y+1)
                memory.push_back(GetAbstractEdge(edges[x]));
        }
    }
}

/**
 * MinimalSectorAbstraction::GetAbstractEdge()
 *
 * \brief Turn the edge data structure into a compressed edge.
 *
 * A quick helper function for turning tempEdgeData into a compressed (1byte)
 * edge representation.
 *
 * \param data edge data
 * \return A 1 byte representation of an edge
 */
uint8_t MinimalSectorAbstraction::GetAbstractEdge(tempEdgeData &data)
{
#ifdef DIAG_MOVES
    return (data.direction<<5)|((data.to-1)&0x1F);
#else
    return (data.direction<<6)|((data.to-1)&0x3F);
#endif
}

/**
 * MinimalSectorAbstraction::GetAbstractLocation()
 *
 * \brief Choose a region center.
 *
 * Find a region center, placing it at the point closest to the
 * average weight of all points in the region.
 *
 * \param area The sector data for the computation
 * \param value The region number we are trying to analyze
 * \return The offset of the region center from the sector start
 */
uint8_t MinimalSectorAbstraction::GetAbstractLocation(std::vector<int> area, int value)
{
    // too simple, finds any point
    //    for (unsigned int x = 0; x < area.size(); x++)
    //    {
    //        if (area[x] == value)
    //        {
    //            return x;
    //        }
    //    }
    //    assert(false);
    
    // find center of mass and get the point closest to it
    int xaverage = 0;
    int yaverage = 0;
    int count = 0;
    for (unsigned int x = 0; x < area.size(); x++)
    {
        if (area[x] == value)
        {
            count++;
            xaverage+=x%sectorSize;
            yaverage+=x/sectorSize;
        }
    }
	assert(count != 0);
    xaverage/=count;
    yaverage/=count;
    int best = sectorSize*sectorSize;
    int index = -1;
    for (unsigned int x = 0; x < area.size(); x++)
    {
        if (area[x] == value)
        {
            int score = ((x%sectorSize)-xaverage)*((x%sectorSize)-xaverage) +
            (x/sectorSize - yaverage)*(x/sectorSize - yaverage);
            if (score < best)
            {
                best = score;
                index = x;
            }
        }
    }
    assert(index != -1);
    return index;
}

///**
// * MinimalSectorAbstraction::OpenGLDraw()
// *
// * \brief Draw the abstraction using OpenGL
// *
// * Draws the abstraction & edges using OpenGL. If the abstraction
// * is being optimized, draws the edges being optimized in red.
// *
// * \param none
// * \return none
// */
//void MinimalSectorAbstraction::OpenGLDraw()
//{
//    static bool draw = false;
//    
//    //  this will draw the sectors for the map
//    GLdouble xx, yy, zz, rr;
//    glColor4f(0.5, 0.0, 0.0, 0.5);
//    map->GetCoord(0, 0, xx, yy, zz, rr);
//    
//    // draw grid lines for sectors
//    glLineWidth(2);
//    glBegin(GL_LINES);
//    for (int y = 0; y <= numYSectors; y++)
//    {
//        glVertex3f(xx-rr, yy-rr+2*y*rr*sectorSize, zz-1*rr);
//        glVertex3f(xx+2*numXSectors*rr*sectorSize, yy-rr+2*y*sectorSize*rr, zz-1*rr);
//    }
//    for (int x = 0; x <= numXSectors; x++)
//    {
//        glVertex3f(xx-rr+2*x*rr*sectorSize, yy-rr, zz-1*rr);
//        glVertex3f(xx-rr+2*x*rr*sectorSize, yy-rr+2*numYSectors*rr*sectorSize, zz-1*rr);
//    }
//    glEnd();
//    glLineWidth(1);
//    
//    if (draw) printf("===BEGIN DRAW\n");
//    // now draw the abstraction itself
//    for (unsigned int x = 0; x < sectors.size(); x++)
//    {
//        for (unsigned int y = 0; y < sectors[x].numRegions; y++)
//        {
//            if (draw) printf("===DRAW SECTOR %d region %d\n", x, y);
//            
//            unsigned int loc1, loc2;
//            std::vector<tempEdgeData> neighbors;
//            GetNeighbors(x, y, neighbors);
//            
//            glBegin(GL_LINES);
//            GetXYLocation(x, y, loc1, loc2);
//            map->GetCoord((int)loc1, (int)loc2, xx, yy, zz, rr);
//            for (unsigned int z = 0; z < neighbors.size(); z++)
//            {
//                // if the current sector is being optimized, change the color
//                if ((int)x == optimizationIndex)
//                    glColor3f(1.0, 0.0, 0.0);
//                else
//                    glColor4f(0.2, 0.8, 0.2, 1.00);
//                
//                if (draw) printf("Got edge from region %d to region %d (going dir %d)\n",
//                                 neighbors[z].from, neighbors[z].to, neighbors[z].direction);
//                if (draw) printf("Drawing from (%d, %d)\n", loc1, loc2);
//                
//                GLdouble xx2, yy2, zz2, rr2;
//                // only draw half of the directions, since edges are represented twice
//                switch (neighbors[z].direction)
//                {
//                    case 0:
//                        glVertex3f(xx, yy, zz-5.1*rr);
//                        if ((int)x-numXSectors == optimizationIndex)
//                            glColor3f(1.0, 0.0, 0.0);
//                        else
//                            glColor4f(0.2, 0.8, 0.2, 1.00);
//                        GetXYLocation(x-numXSectors, neighbors[z].to, loc1, loc2);
//                        map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//                        if (draw) printf("Drawing to (%d, %d)\n", loc1, loc2);
//                            glVertex3f(xx2, yy2, zz2-5.1*rr2);
//                        break;
//                    case 1:
//                        glVertex3f(xx, yy, zz-5.1*rr);
//                        if ((int)x+1 == optimizationIndex)
//                            glColor3f(1.0, 0.0, 0.0);
//                        else
//                            glColor4f(0.2, 0.8, 0.2, 1.00);
//                        GetXYLocation(x+1, neighbors[z].to, loc1, loc2);
//                        map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//                        if (draw) printf("Drawing to (%d, %d)\n", loc1, loc2);
//                            glVertex3f(xx2, yy2, zz2-5.1*rr2);
//                        break;
//                    case 2:
//                    case 3: break;
//                    case 4:
//                        glVertex3f(xx, yy, zz-5.1*rr);
//                        if ((int)x-numXSectors+1 == optimizationIndex)
//                            glColor3f(1.0, 0.0, 0.0);
//                        else
//                            glColor4f(0.2, 0.8, 0.2, 1.00);
//                        GetXYLocation(x-numXSectors+1, neighbors[z].to, loc1, loc2);
//                        map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//                        if (draw) printf("Drawing to (%d, %d)\n", loc1, loc2);
//                            glVertex3f(xx2, yy2, zz2-5.1*rr2);
//                        break;
//                    case 5:
//                        glVertex3f(xx, yy, zz-5.1*rr);
//                        if ((int)x+numXSectors+1 == optimizationIndex)
//                            glColor3f(1.0, 0.0, 0.0);
//                        else
//                            glColor4f(0.2, 0.8, 0.2, 1.00);
//                        GetXYLocation(x+numXSectors+1, neighbors[z].to, loc1, loc2);
//                        map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//                        if (draw) printf("Drawing to (%d, %d)\n", loc1, loc2);
//                            glVertex3f(xx2, yy2, zz2-5.1*rr2);
//                        break;
//                    default: 
//                        break; // otherwise we draw all edges twice
//                }
//            }
//            glEnd();
//        }
//    }
//    draw = false;
//}

void MinimalSectorAbstraction::Draw(Graphics::Display &display) const
{
	static bool draw = false;
	static float lineScale = 0.5f;
	//  this will draw the sectors for the map
	double xx, yy, zz, rr;
//	glColor4f(0.5, 0.0, 0.0, 0.5);
	map->GetCoord(0, 0, xx, yy, zz, rr);
	
	// draw grid lines for sectors
//	glLineWidth(2);
//	glBegin(GL_LINES);
	for (int y = 0; y <= numYSectors; y++)
	{
		display.DrawLine({static_cast<float>(xx-rr), static_cast<float>(yy-rr+2*y*rr*sectorSize), static_cast<float>(zz-1*rr)},
						 {static_cast<float>(xx+2*numXSectors*rr*sectorSize), static_cast<float>(yy-rr+2*y*sectorSize*rr), static_cast<float>(zz-1*rr)},
						 lineScale*rr, Colors::yellow);
//		glVertex3f(xx-rr, yy-rr+2*y*rr*sectorSize, zz-1*rr);
//		glVertex3f(xx+2*numXSectors*rr*sectorSize, yy-rr+2*y*sectorSize*rr, zz-1*rr);
	}
	for (int x = 0; x <= numXSectors; x++)
	{
		display.DrawLine({static_cast<float>(xx-rr+2*x*rr*sectorSize), static_cast<float>(yy-rr), static_cast<float>(zz-1*rr)},
						 {static_cast<float>(xx-rr+2*x*rr*sectorSize), static_cast<float>(yy-rr+2*numYSectors*rr*sectorSize), static_cast<float>(zz-1*rr)},
						 lineScale*rr, Colors::yellow);
//		glVertex3f(xx-rr+2*x*rr*sectorSize, yy-rr, zz-1*rr);
//		glVertex3f(xx-rr+2*x*rr*sectorSize, yy-rr+2*numYSectors*rr*sectorSize, zz-1*rr);
	}
//	glEnd();
//	glLineWidth(1);
	
	rgbColor abstractionColor = Colors::bluegreen;
	
//	if (draw) printf("===BEGIN DRAW\n");
	// now draw the abstraction itself
	for (unsigned int x = 0; x < sectors.size(); x++)
	{
		for (unsigned int y = 0; y < sectors[x].numRegions; y++)
		{
//			if (draw) printf("===DRAW SECTOR %d region %d\n", x, y);
			
			unsigned int loc1, loc2;
			std::vector<tempEdgeData> neighbors;
			GetNeighbors(x, y, neighbors);
			
//			glBegin(GL_LINES);
			GetXYLocation(x, y, loc1, loc2);
			map->GetCoord((int)loc1, (int)loc2, xx, yy, zz, rr);
			for (unsigned int z = 0; z < neighbors.size(); z++)
			{
				double xx2, yy2, zz2, rr2;
				// only draw half of the directions, since edges are represented twice
				switch (neighbors[z].direction)
				{
					case 0:
//						glVertex3f(xx, yy, zz-5.1*rr);
						GetXYLocation(x-numXSectors, neighbors[z].to, loc1, loc2);
						map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//						glVertex3f(xx2, yy2, zz2-5.1*rr2);
						display.DrawLine({static_cast<float>(xx), static_cast<float>(yy), static_cast<float>(zz-5.1*rr)},
										 {static_cast<float>(xx2), static_cast<float>(yy2), static_cast<float>(zz2-5.1*rr2)}, lineScale*rr2, abstractionColor);
						break;
					case 1:
//						glVertex3f(xx, yy, zz-5.1*rr);
						GetXYLocation(x+1, neighbors[z].to, loc1, loc2);
						map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//						glVertex3f(xx2, yy2, zz2-5.1*rr2);
						display.DrawLine({static_cast<float>(xx), static_cast<float>(yy), static_cast<float>(zz-5.1*rr)},
										 {static_cast<float>(xx2), static_cast<float>(yy2), static_cast<float>(zz2-5.1*rr2)}, lineScale*rr2, abstractionColor);
						break;
					case 2:
					case 3: break;
					case 4:
//						glVertex3f(xx, yy, zz-5.1*rr);
						GetXYLocation(x-numXSectors+1, neighbors[z].to, loc1, loc2);
						map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//						glVertex3f(xx2, yy2, zz2-5.1*rr2);
						display.DrawLine({static_cast<float>(xx), static_cast<float>(yy), static_cast<float>(zz-5.1*rr)},
										 {static_cast<float>(xx2), static_cast<float>(yy2), static_cast<float>(zz2-5.1*rr2)}, lineScale*rr2, abstractionColor);
						break;
					case 5:
//						glVertex3f(xx, yy, zz-5.1*rr);
						GetXYLocation(x+numXSectors+1, neighbors[z].to, loc1, loc2);
						map->GetCoord((int)loc1, (int)loc2, xx2, yy2, zz2, rr2);
//						glVertex3f(xx2, yy2, zz2-5.1*rr2);
						display.DrawLine({static_cast<float>(xx), static_cast<float>(yy), static_cast<float>(zz-5.1*rr)},
										 {static_cast<float>(xx2), static_cast<float>(yy2), static_cast<float>(zz2-5.1*rr2)}, lineScale*rr2, abstractionColor);
						break;
					default:
						break; // otherwise we draw all edges twice
				}
			}
//			glEnd();
		}
	}
//	draw = false;
}

/**
 * MinimalSectorAbstraction::GetXYLocation()
 *
 * \brief Given a sector and region, compute the x/y location.
 *
 * Given a sector and region, compute the x/y location.
 *
 * \param sector the sector to use
 * \param region the region to use
 * \param x the x-coordinate of the region center
 * \param y the y-coordinate of the region center
 * \return none
 */
void MinimalSectorAbstraction::GetXYLocation(unsigned int sector,
                                             unsigned int region,
                                             unsigned int &x,
                                             unsigned int &y) const
{
    uint8_t loc = memory[sectors[sector].memoryAddress+region*2];
    x = loc%sectorSize;
    y = loc/sectorSize;
    x += (sector%numXSectors)*sectorSize;
    y += ((int)sector/numXSectors)*sectorSize;
}

/**
 * MinimalSectorAbstraction::GetNeighbors()
 *
 * \brief Find the edges in/out of the given sector
 *
 * Extract the edges from the abstraction and return them.
 *
 * \param sector The sector to use
 * \param region The region to use
 * \param edges On return contains the edges from this sector/region
 * \return none
 */
void MinimalSectorAbstraction::GetNeighbors(unsigned int sector,
                                            unsigned int region,
                                            std::vector<tempEdgeData> &edges) const
{
    edges.resize(0);
    int sectorAddress = sectors[sector].memoryAddress;
    int numRegions = sectors[sector].numRegions;
    int numEdges, edgeStart;
    if (region == 0)
    {
        edgeStart = 0;
        numEdges = memory[sectorAddress+region*2+1];
    }
    else {
        edgeStart = memory[sectorAddress+(region-1)*2+1];
        numEdges = memory[sectorAddress+region*2+1]-edgeStart;
    }
    for (int x = edgeStart; x < edgeStart+numEdges; x++)
    {
        tempEdgeData ted;
        ted.from = region;
#ifdef DIAG_MOVES
        ted.direction = (memory[sectorAddress+2*numRegions+x]>>5)&0x7;
        ted.to = memory[sectorAddress+2*numRegions+x]&0x1F;
#else
        ted.direction = (memory[sectorAddress+2*numRegions+x]>>6)&0x3;
        ted.to = memory[sectorAddress+2*numRegions+x]&0x3F;
#endif
        edges.push_back(ted);
    }
}

/**
 * MinimalSectorAbstraction::GetAdjacentSector()
 *
 * \brief Returns the sector in a given direction
 *
 * Given a direction and a sector, compute which sector is immediately
 * adjacent in that direction.
 *
 * \param sector The sector to use
 * \param direction The direction to use
 * \return The sector in the particular direction
 */
int MinimalSectorAbstraction::GetAdjacentSector(unsigned int sector,
                                                int direction)
{
    switch (direction) {
        case 0: return sector-numXSectors; // up
        case 1: return sector+1; // right
        case 2: return sector+numXSectors; // down
        case 3: return sector-1; // left
#ifdef DIAG_MOVES
        case 4: return sector - numXSectors + 1; // up right
        case 5: return sector + numXSectors + 1; // down right
        case 6: return sector + numXSectors - 1;// down left
        case 7: return sector - numXSectors - 1;// up left
#endif
    }
    assert(false);
    return -1;
}

/**
 * MinimalSectorAbstraction::GetSector()
 *
 * \brief Given an x/y coordinate, return the sector
 *
 * Given an x/y coordinate, return the sector
 *
 * \param x The x-coordinate for the computation
 * \param y The y-coordinate for the computation
 * \return The sector of the x/y coordinate
 */
int MinimalSectorAbstraction::GetSector(int x, int y)
{
    if ((x < 0) || (y < 0))
        return -1;
    int xsector = x/sectorSize;
    int ysector = y/sectorSize;
    if ((xsector >= numXSectors) ||
        (ysector >= numYSectors))
        return -1;
    return ysector*numXSectors+xsector;
}

/**
 * MinimalSectorAbstraction::GetRegion()
 *
 * \brief Find the region associated with an x/y coordinate
 *
 * Returns the region of the sector that the x/y coordinate is in.
 * If coordinates are invalid, returns -1.
 *
 * \param x The x-coordinate for the computation
 * \param y The y-coordinate for the computation
 * \return The region for this coordinate
 */
int MinimalSectorAbstraction::GetRegion(int x, int y)
{
    if (map->GetTerrainType(x, y) != kGround)
        return -1;
    int sector = GetSector(x, y);
    if (sectors[sector].numRegions == 1) // only one region, we must be in region 0
        return 0;
    
    std::vector<int> regions(sectors[sector].numRegions);
    for (unsigned int t = 0; t < sectors[sector].numRegions; t++)
    {
        regions[t] = memory[sectors[sector].memoryAddress+t*2];
        //        printf("Region %d location %d\n", t, regions[t]);
    }
    return FindParentRegion(x%sectorSize+((y%sectorSize)*sectorSize), regions,
                            x-x%sectorSize, y-y%sectorSize);
}

/**
 * MinimalSectorAbstraction::FindParentRegion()
 *
 * \brief Helper function for GetRegion()
 *
 * Does a breadth-first search looking for the region associated with
 * a particular point.
 *
 * \param startLoc The initial offset in the region
 * \param parents The possible region centers
 * \param mapXOffset The x offset of the top of the sector
 * \param mapYOffset The y offset of the top of the sector
 * \return The region the point is in
 */
int MinimalSectorAbstraction::FindParentRegion(int startLoc,
                                               std::vector<int> &parents,
                                               int mapXOffset, int mapYOffset)
{
    int current = startLoc;
    std::vector<int> stack;
    std::queue<int> Q;
    std::vector<int> markers(sectorSize*sectorSize);
    markers[current] = 1;
    //    printf("Starting on %d\n", startLoc);
    while (1)
    {
        for (unsigned int x = 0; x < parents.size(); x++)
            if (current == parents[x])
                return (int)x;
        
        if ((current+sectorSize < sectorSize*sectorSize) && (markers[current+sectorSize] == 0))
        {
            //stack.push_back(current+sectorSize);
            Q.push(current+sectorSize);
            markers[current+sectorSize] = 1;
        }
        if ((current-sectorSize >= 0) && (markers[current-sectorSize] == 0))
        {
            //stack.push_back(current-sectorSize);
            Q.push(current-sectorSize);
            markers[current-sectorSize] = 1;
        }
        if ((((current+1)%sectorSize) != 0) && (markers[current+1] == 0))
        {
            //stack.push_back(current+1);
            Q.push(current+1);
            markers[current+1] = 1;
        }
        if (((current%sectorSize) != 0) && (markers[current-1] == 0))
        {
            //stack.push_back(current-1);
            Q.push(current-1);
            markers[current-1] = 1;
        }
        
        do {
            //            printf("Stack size: %d\n", Q.size());
            assert(Q.size() != 0);
            current = Q.front();
            Q.pop();
            //            printf("Checking %d next\n", current);
        } while ((markers[current] != 0) &&
                 (map->GetTerrainType(mapXOffset+current%sectorSize,
                                      mapYOffset+current/sectorSize) != kGround));
    }
    return -1;
}

/**
 * MinimalSectorAbstraction::GetSectorRegions()
 *
 * \brief Do a BFS to find and label the regions in a sector
 *
 * Does a BFS to find and label all the regions in a sector.
 * Used when building the abstraction
 *
 * \param area Data structure containing the labelled points
 * \param absXSector The x offset of the sector
 * \param absYSector The y offset of the sector
 * \return The number of regions in the sector
 */
int MinimalSectorAbstraction::GetSectorRegions(std::vector<int> &area,
                                               int absXSector,
                                               int absYSector)
{
    area.resize(0);
    area.resize(sectorSize*sectorSize);
    
    // initialize sector map to 0 for unsearched, -1 for unreachable
    for (int x = 0; x < sectorSize; x++)
    {
        for (int y = 0; y < sectorSize; y++)
        {
            if (map->GetTerrainType(absXSector*sectorSize+x,
                                    absYSector*sectorSize+y) != kGround)
            {
                area[y*sectorSize+x] = -1;
            }
        }
    }
    
    int nextLabel = 1;
    for (int x = 0; x < sectorSize; x++)
    {
        for (int y = 0; y < sectorSize; y++)
        {
            if (area[y*sectorSize+x] == 0)
            {
                LabelRegion(area, x, y, nextLabel);
                nextLabel++;
            }
        }
    }
    return nextLabel-1;
}

/**
 * MinimalSectorAbstraction::LabelRegion()
 *
 * \brief Do a BFS within a region to label it
 *
 * Performs the BFS within one region to label that region.
 *
 * \param area The stored labels
 * \param x The starting location for the BFS
 * \param y The starting location for the BFS
 * \param label The label to use
 * \return none
 */
void MinimalSectorAbstraction::LabelRegion(std::vector<int> &area,
                                           int x, int y, int label)
{
    if ((y < 0) || (x < 0) || (y >= sectorSize) || (x >= sectorSize))
        return;
    if (area[y*sectorSize+x] != 0)
        return;
    
    area[y*sectorSize+x] = label;
    LabelRegion(area, x+1, y, label);
    LabelRegion(area, x-1, y, label);
    LabelRegion(area, x, y+1, label);
    LabelRegion(area, x, y-1, label);
}

/**
 * MinimalSectorAbstraction::InitializeOptimization()
 *
 * \brief Initialize variables for optimization
 *
 * Optimization can be run in one step or incrementally. This initializes
 * the variables to run the optimization incrementally.
 *
 * \param none
 * \return none
 */
void MinimalSectorAbstraction::InitializeOptimization()
{
    regionError.resize(sectors.size());
    optimizationIndex = 0;
}

/**
 * MinimalSectorAbstraction::PerformOneOptimizationStep()
 *
 * \brief Perform one optimization step. Must be called after InitializeOptimization().
 *
 * Perform a single optimization on the next sector to be optimized.
 *
 * \param none
 * \return true if the optimization is finished, false otherwise
 */
bool MinimalSectorAbstraction::PerformOneOptimizationStep()
{
    if (optimizationIndex >= (int)sectors.size())
        return true;
//
//    regionError[optimizationIndex].resize(sectors[optimizationIndex].numRegions);
//    for (unsigned int y = 0; y < sectors[optimizationIndex].numRegions; y++)
//    {
//        unsigned int sx, sy;
//        ResetAbstractCenter(optimizationIndex, y);
//        GetXYLocation(optimizationIndex, y, sx, sy);
//        GetRegionError(optimizationIndex, y, sx, sy, sectorSize*2+1);
//        if (regionError[optimizationIndex][y] > sectorSize*1.5)
//            MoveRegionCenter(optimizationIndex, y);
//    }
//    
//    optimizationIndex++;
    return false;
}

/**
 * MinimalSectorAbstraction::ResetAbstractCenter()
 *
 * \brief Reset a region to the weighted average location.
 *
 * Resets a region center to the weighted average of the region. This is used
 * when optimizing an abstraction a second time to reset the location of the
 * abstract center.
 * 
 * Sometimes more than one region center will get moved to correct for some
 * topology feature, when only one region center really needs to be
 * moved. By moving the region center back to the default location we help
 * improve this case when running optimization a second time
 *
 * \param sector The sector to reset
 * \param region The region in the sector to reset
 * \return none
 */
void MinimalSectorAbstraction::ResetAbstractCenter(int sector, int region)
{
    uint8_t defaultCenter = GetAbstractLocation(areas[sector], region+1);
    memory[sectors[sector].memoryAddress+2*region] = defaultCenter;
}

/**
 * MinimalSectorAbstraction::OptimizeRegionLocations()
 *
 * \brief Perform complete optimization on map.
 *
 * \param none
 * \return none
 */
void MinimalSectorAbstraction::OptimizeRegionLocations()
{
    InitializeOptimization();
    while (!PerformOneOptimizationStep());
}

/**
 * MinimalSectorAbstraction::MoveRegionCenter()
 *
 * \brief Optimize a region center
 *
 * Try every possible location for a region center. Choose the one
 * which minimizes the error returned by GetRegionError.
 *
 * \param sector The sector to test
 * \param region The region in the sector to test
 * \return none
 */
void MinimalSectorAbstraction::MoveRegionCenter(int sector, int region)
{
    double error = regionError[sector][region];
    int best = -1;
    for (unsigned int x = 0; x < areas[sector].size(); x++)
    {
        if (areas[sector][x] == region+1)
        {
            int xoff = (sector%numXSectors)*sectorSize;
            int yoff = (sector/numXSectors)*sectorSize;
            double err = GetRegionError(sector, region, xoff+(x%sectorSize),
                                        yoff+x/sectorSize, error);
            if (fless(err, error))
            {
                error = err;
                best = x;
            }
        }
    }
    printf("For %d:%d, error improved to %f at offset %d\n", sector, region, error, best);
    if (best != -1)
        memory[sectors[sector].memoryAddress+2*region] = best;
}

/**
 * MinimalSectorAbstraction::GetRegionError()
 *
 * \brief Compute the "error" for the current region center
 *
 * Given a point within a region, compute the cost between that point
 * and every neighboring region. Use that cost to formulate an "error"
 * term which is returned.
 *
 * In this case we compuate the number of nodes expanded above what is
 * optimal (the optimal path length)
 *
 * An error bound is used to reduce computation. If this point is too expensive
 * (above the error limit) we can stop early.
 *
 * \param fromSector The sector we start in
 * \param fromRegion The region we start in
 * \param sx The start x-location in the region
 * \param sy The start y-location in the region
 * \param limit If the error exceeds this limit we can stop
 * \return none
 */
double MinimalSectorAbstraction::GetRegionError(int fromSector, int fromRegion,
                                                int sx, int sy, double limit)
{
//    GenericAStar gas;
//    std::vector<tempEdgeData> edges;
//    edges.resize(0);
//    GetNeighbors(fromSector, fromRegion, edges);
//    regionError[fromSector][fromRegion] = 0.0;
//    for (unsigned int z = 0; z < edges.size(); z++)
//    {
//        MapEnvironment env(map);
//        int toSector = GetAdjacentSector(fromSector, edges[z].direction);
//        int toRegion = edges[z].to;
//        unsigned int gx, gy;
//        GetXYLocation(toSector, toRegion, gx, gy);
//
//        std::vector<uint32_t> thePath;
//        // compute the cost to travel between the given sector/region and a neighboring region
//        // We measure the cost as the number of nodes expanded over and above the optimal path
//        // length
//        gas.GetPath(&env, (gx<<16)|gy, (sx<<16)|sy, thePath);
//        assert(thePath.size() != 0);
//        regionError[fromSector][fromRegion] = max(regionError[fromSector][fromRegion],
//                                                  gas.GetNodesExpanded()-thePath.size());
//        if (fgreater(regionError[fromSector][fromRegion], limit))
//            return regionError[fromSector][fromRegion];
//
//        // now do it again the other way, because we could go either way along this edge
//        // and the costs can be highly assymetrical
//        thePath.resize(0);
//        gas.GetPath(&env, (sx<<16)|sy, (gx<<16)|gy, thePath);
//        assert(thePath.size() != 0);
//        regionError[fromSector][fromRegion] = max(regionError[fromSector][fromRegion],
//                                                  gas.GetNodesExpanded()-thePath.size());
//        if (fgreater(regionError[fromSector][fromRegion], limit))
//            return regionError[fromSector][fromRegion];
//    }
//    return regionError[fromSector][fromRegion];
	assert(!"Commented out code here.");
	return 0;
}

/**
 * MinimalSectorAbstraction::ComputePotentialMemorySavings()
 *
 * \brief print how much memory would be saved using 'default' sectors
 *
 * print how much memory would be saved using 'default' sectors
 *
 * \param none
 * \return none
 */
void MinimalSectorAbstraction::ComputePotentialMemorySavings()
{
    int defaultSectors = 0;
    for (unsigned int x = 0; x < sectors.size(); x++)
    {
        //std::vector<sectorInfo> sectors;
        if (sectors[x].numRegions != 1)
            continue;
        
        if (sectors[x].numEdges != 8)
            continue;
        
        std::vector<tempEdgeData> edges;
        std::vector<int> edgedir(8);
        GetNeighbors(x, 0, edges);
        for (unsigned int y = 0; y < edges.size(); y++)
            edgedir[edges[y].direction] = 1;
        
        // if there is one edge in each direction, this is a default sector
        int sum = 0;
        for (unsigned int y = 0; y < edgedir.size(); y++)
            sum += edgedir[y];
        if (sum == 8)
            defaultSectors++;
    }
    printf("%d default sectors would save %d bytes\n", defaultSectors, (8+2)*defaultSectors);
}
