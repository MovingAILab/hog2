//
//  MapGenerator.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/5/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#include "MapGenerators.h"
#include <vector>
#include <algorithm>
#include <random>

/**
 * MakeMaze(map, pathsize)
 *
 * A cheap function I hacked together to make psuedo mazes.
 * The only good values for pathSize are 1 and 3.
 */
void MakePseudoMaze(Map *map, int pathSize)
{
	int width = map->GetMapWidth();
	int height = map->GetMapHeight();
	map->SetRectHeight(0, 0, width-1, height-1, 0, kGround);
	int pathWidth = 1;
	for (int t = 0; (pathSize>>t); t++)
		pathWidth = t+1;
	for (int i = 0; i < 6*width*height; i++)
	{
		long x = (random()%width)&(~pathSize); // only path on even tiles
		long y = (random()%height)&(~pathSize);
		
		if (map->GetHeight(x, y) <= 1)
		{
			switch(random()%4)
			{
				case 0: // NORTH
					if ((x >= 2*pathWidth) && (map->GetHeight(x-2*pathWidth, y)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x-2, y+t, map->GetHeight(x-2, y)+1);
							map->SetHeight(x-1, y+t, map->GetHeight(x-1, y)+1);
							map->SetHeight(x, y+t, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 1: // SOUTH
					if ((x < width-2*pathWidth) && (map->GetHeight(x+2*pathWidth, y)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+2, y+t, map->GetHeight(x+2, y)+1);
							map->SetHeight(x+1, y+t, map->GetHeight(x+1, y)+1);
							map->SetHeight(x, y+t, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 2: // WEST
					if ((y >= 2*pathWidth) && (map->GetHeight(x, y-2*pathWidth)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+t, y-2, map->GetHeight(x, y-2)+1);
							map->SetHeight(x+t, y-1, map->GetHeight(x, y-1)+1);
							map->SetHeight(x+t, y, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 3: // EAST
					if ((y < height-2*pathWidth) && (map->GetHeight(x, y+2*pathWidth)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+t, y+2, map->GetHeight(x, y+2)+1);
							map->SetHeight(x+t, y+1, map->GetHeight(x, y+1)+1);
							map->SetHeight(x+t, y, map->GetHeight(x, y)+1);
						}
					}
					break;
			}
		}
	}
	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
			if (map->GetHeight(x, y) > 0)
			{
				map->SetHeight(x, y, 0);
				map->SetTerrainType(x, y, kOutOfBounds);
			}
}

/**
 * MakeMaze(map)
 */
void MakeMaze(Map *map, int corridorWidth, int startx, int starty)
{
	int width = map->GetMapWidth();
	int height = map->GetMapHeight();
	int boxSize = corridorWidth+1;
	if (startx == -1)
		startx = width/(2*boxSize);
	else
		startx = startx-startx%boxSize;
	if (starty == -1)
		starty = height/(2*boxSize);
	else
		starty = starty-starty%boxSize;
	// fill the whole map empty
	map->SetRectHeight(0, 0, width-1, height-1, 0, kGround);
	// block all the walls
	for (int x = 0; x < width; x+=boxSize)
		map->SetRectHeight(x, 0, x, height-1, 0, kOutOfBounds);
	for (int y = 0; y < height; y+=boxSize)
		map->SetRectHeight(0, y, width-1, y, 0, kOutOfBounds);
	// mark univisited states
	for (int x = 0; x < width; x+=boxSize)
		for (int y = 0; y < height; y += boxSize)
			map->SetTerrainType(x+1, y+1, kOutOfBounds);
	std::vector<int> x;
	std::vector<int> y;
	
	x.push_back(startx);
	y.push_back(starty);
	//	x.push_back(0);
	//	y.push_back(0);
	//	map->SetHeight(0, 0, 1);
	map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
	
	while (x.size() > 0)
	{
		bool lastRandom = false;
		//		map->Print();
		int val;
		
		if (lastRandom || (1 != random()%10))
		{
			val = x.size()-1;
//			if (1 == random()%30)
//				lastRandom = false;
		}
		else {
			// bias towards end of array
			val = ((random()%(3*x.size()/2))+x.size()/2)%x.size();
//			lastRandom = true;
		}
		
		//printf("Trying to extend (%d, %d)\n", x[val], y[val]);
		// raise up a corridor and open paths
		switch (random()%4)
		{
			case 0: // NORTH
			{
				if ((y[val] > 0) && (map->GetTerrainType(x[val]*boxSize+1, (y[val]-1)*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize+1+t, y[val]*boxSize, kGround);
					}
					x.push_back(x[val]);
					y.push_back(y[val]-1);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended NORTH to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 1: // SOUTH
			{
				if ((y[val] < (height+boxSize-1)/boxSize) && (map->GetTerrainType(x[val]*boxSize+1, (y[val]+1)*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize+1+t, (y[val]+1)*boxSize, kGround);
					}
					x.push_back(x[val]);
					y.push_back(y[val]+1);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended SOUTH to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 2: // EAST
			{
				if (((x[val]+1)*boxSize+1 < width) && (map->GetTerrainType((x[val]+1)*boxSize+1, y[val]*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType((x[val]+1)*boxSize, y[val]*boxSize+1+t, kGround);
					}
					x.push_back(x[val]+1);
					y.push_back(y[val]);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended EAST to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 3: // WEST
			{
				if ((x[val] > 0) && (map->GetTerrainType((x[val]-1)*boxSize+1, y[val]*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize, y[val]*boxSize+1+t, kGround);
					}
					x.push_back(x[val]-1);
					y.push_back(y[val]);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended WEST to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
				}
				break;
			}
		}
		
		// check to see if node is blocked on all sides
		if (((x[val] == 0) || map->GetTerrainType((x[val]-1)*boxSize+1, y[val]*boxSize+1) == kGround) && // blocked left
			(((x[val]+1)*boxSize+1 >= width) || map->GetTerrainType((x[val]+1)*boxSize+1, y[val]*boxSize+1) == kGround) && // blocked right
			((y[val] == 0) || map->GetTerrainType(x[val]*boxSize+1, (y[val]-1)*boxSize+1) == kGround) && // blocked up
			(((y[val]+1)*boxSize+1 >= height) || map->GetTerrainType(x[val]*boxSize+1, (y[val]+1)*boxSize+1) == kGround)) // blocked down
		{
			//	printf("(%d, %d) now blocked: %d left\n", x[val], y[val], x.size()-1);
			x[val] = x.back();
			y[val] = y.back();
			x.pop_back();
			y.pop_back();
		}
	}
}

void BuildRandomRoomMap(Map *map, int roomSize, int openingProbability)
{
    int width = map->GetMapWidth();
    int height = map->GetMapHeight();
	
	map->SetRectHeight(0, 0, width-1, height-1, 0, kGround);
	//map->SetTerrainType(0, 0, width-1, height-1, kGround);
    for (int x = 0; x < height; x += roomSize)
    {
        // draw a horizontal line
        map->SetTerrainType(0, x, width-1, x, kOutOfBounds);
        // then punch a bunch of holes in it
        for (int y = 0; y < width; y += roomSize)
		{
			if ((random()%100) < openingProbability) // chance of creating hole
			{
				int val = roomSize/8;
				if (val == 0)
					val = 1;
				for (int z = 0; z < val; z++)
				{
					map->SetTerrainType(y+1+random()%(roomSize-1), x, kGround);
				}
			}
		}
    }
    for (int x = 0; x < width; x += roomSize)
    {
        // draw a vertical line
        map->SetTerrainType(x, 0, x, height-1, kOutOfBounds);
        // then punch a bunch of holes in it
        for (int y = 0; y < height; y += roomSize)
		{
			if ((random()%100) < openingProbability) // chance of creating hole
			{
				int val = roomSize/8;
				if (val == 0)
					val = 1;
				for (int z = 0; z < val; z++)
				{
					map->SetTerrainType(x, y+1+random()%(roomSize-1), kGround);
				}
			}
		}
    }
}

void MakeRandomMap(Map *map, int obstacles)
{
	int total = map->GetMapWidth()*map->GetMapHeight()*obstacles/100;
	for (int x = 0; x < total; x++)
	{
		int xloc, yloc;
		while (1)
		{
			xloc = random()%map->GetMapWidth();
			yloc = random()%map->GetMapHeight();
			if (map->GetTerrainType(xloc, yloc) == kGround)
				break;
		}
		map->SetTerrainType(xloc, yloc, kOutOfBounds);
	}
}

void MakeDesignedMap(Map *map, int obstacleSize, int type)
{
    int xloc, yloc, tmp;
    xloc = map->GetMapWidth()/2;
    yloc = map->GetMapHeight()/2;
    switch (type)
    {
    case 0:
        /* Only cosists a square of obstacles in the center of empty map */
        for(int i=xloc-obstacleSize; i<=xloc+obstacleSize; i++)
            for(int j=yloc-obstacleSize; j<=yloc+obstacleSize; j++)
                map->SetTerrainType(i, j, kOutOfBounds);
        break;
    
    case 1:
        /* Only cosists a square of swamp states in the center of empty map */
        for(int i=xloc-obstacleSize; i<=xloc+obstacleSize; i++)
            for(int j=yloc-obstacleSize; j<=yloc+obstacleSize; j++)
                map->SetTerrainType(i, j, kSwamp);
        break;

    case 2:
        /* Only cosists a diamond of obstacles in the center of empty map */
        tmp = obstacleSize;
        for(int i=yloc-obstacleSize; i<=yloc+obstacleSize; i++)
        {    for(int k=xloc-abs(obstacleSize-tmp)/2; k<=xloc+abs(obstacleSize-tmp)/2; k++)
                map->SetTerrainType(k, i, kOutOfBounds);
            if(i>=yloc)
                tmp -=2;
            else
                tmp +=2;
        }
        break;
    
    case 3:
        /* Only cosists a diamond of swamp states in the center of empty map */
        tmp = obstacleSize;
        for(int i=yloc-obstacleSize; i<=yloc+obstacleSize; i++)
        {    for(int k=xloc-abs(obstacleSize-tmp)/2; k<=xloc+abs(obstacleSize-tmp)/2; k++)
                map->SetTerrainType(k, i, kSwamp);
            if(i>=yloc)
                tmp -=2;
            else
                tmp +=2;
        }
        break;
    
    case 4:
        /* Only cosists a circle of obstacles in the center of empty map */
        for(int i=xloc-obstacleSize; i<=xloc+obstacleSize; i++)
            for(int j=yloc-obstacleSize; j<=yloc+obstacleSize; j++)
                if(sqrt(pow(i-xloc,2) + pow(j-yloc,2)) <= obstacleSize)
                    map->SetTerrainType(i, j, kOutOfBounds);
        break;
    
    case 5:
        /* Only cosists a circle of swamp states in the center of empty map */
        for(int i=xloc-obstacleSize; i<=xloc+obstacleSize; i++)
            for(int j=yloc-obstacleSize; j<=yloc+obstacleSize; j++)
                if(sqrt(pow(i-xloc,2) + pow(j-yloc,2)) <= obstacleSize)
                    map->SetTerrainType(i, j, kSwamp);
        break;

    case 6:
        /* Some columns of swamp states in the center of empty map */
        for(int i=xloc-obstacleSize; i<=xloc+obstacleSize; i++)
            for(int j=0; j<map->GetMapHeight(); j++)
                map->SetTerrainType(i, j, kSwamp);
        break;
    default:
        break;
    }
}


struct loc { int x; int y; };

void StraightDir(Map *m, loc l, std::vector<loc> &dirs)
{
	dirs.resize(0);
	if ((m->GetTerrainType(l.x+1, l.y) == kGround) && (m->GetTerrainType(l.x-2, l.y) == kTrees))
		dirs.push_back({l.x-2, l.y});
	if ((m->GetTerrainType(l.x-1, l.y) == kGround) && (m->GetTerrainType(l.x+2, l.y) == kTrees))
		dirs.push_back({l.x+2, l.y});
	if ((m->GetTerrainType(l.x, l.y+1) == kGround) && (m->GetTerrainType(l.x, l.y-2) == kTrees))
		dirs.push_back({l.x, l.y-2});
	if ((m->GetTerrainType(l.x, l.y-1) == kGround) && (m->GetTerrainType(l.x, l.y+2) == kTrees))
		dirs.push_back({l.x, l.y+2});

	if (dirs.size() != 1)
		dirs.resize(0);
}

void PossibileDirs(Map *m, loc l, std::vector<loc> &dirs)
{
	dirs.resize(0);
	if ((m->GetTerrainType(l.x+1, l.y) == kTrees) && (m->GetTerrainType(l.x+2, l.y) == kTrees))
		dirs.push_back({l.x+2, l.y});
	if ((m->GetTerrainType(l.x-1, l.y) == kTrees) && (m->GetTerrainType(l.x-2, l.y) == kTrees))
		dirs.push_back({l.x-2, l.y});
	if ((m->GetTerrainType(l.x, l.y+1) == kTrees) && (m->GetTerrainType(l.x, l.y+2) == kTrees))
		dirs.push_back({l.x, l.y+2});
	if ((m->GetTerrainType(l.x, l.y-1) == kTrees) && (m->GetTerrainType(l.x, l.y-2) == kTrees))
		dirs.push_back({l.x, l.y-2});
}

void Burrow(Map *m, loc l1, loc l2)
{
	m->SetTerrainType(l1.x, l1.y, kGround);
	m->SetTerrainType(l2.x, l2.y, kGround);
	m->SetTerrainType((l1.x+l2.x)/2, (l1.y+l2.y)/2, kGround);
}
void MakeMaze(Map *m, float straightPercent, float branchPercent)
{
	assert(branchPercent >= 0 && branchPercent <= 1 && straightPercent >= 0 && straightPercent <= 1);
	for (int y = 0; y < m->GetMapHeight(); y++)
		for (int x = 0; x < m->GetMapWidth(); x++)
			m->SetTerrainType(x, y, kTrees);
	std::vector<loc> places;
	int startx = ((int)random()%m->GetMapWidth())|1;
	int starty = ((int)random()%m->GetMapHeight())|1;
	places.push_back({startx, starty});
	
	std::vector<loc> straight;
	std::vector<loc> all;
	while (places.size() > 0)
	{
		// Get random location from eligible locations
		int which = (int)random()%places.size();
		//if (74 > random()%100)
		which = places.size()-1; // continue last direction
		
		loc currLoc = places[which];
		places.erase(places.begin()+which);
		
		StraightDir(m, currLoc, straight);
		PossibileDirs(m, currLoc, all);
		// with straightPercent, continue straight when possible
		if (straight.size() > 0 && (random()%10000) < 10000*straightPercent)
		{
			Burrow(m, currLoc, straight[0]);
			places.push_back(straight[0]);
		}
		else {
			// with probabiliy branchPercent, go in 2 directions
			if ((random()%10000) < 10000*branchPercent)
			{
				while (all.size() > 2)
					all.erase(all.begin()+random()%all.size());
			}
			else {
				while (all.size() > 1)
					all.erase(all.begin()+random()%all.size());
			}
			for (loc next : all)
			{
				Burrow(m, currLoc, next);
				places.push_back(next);
			}
            std::random_device rd;
            std::mt19937 g(rd());
			// std::random_shuffle ( places.begin(), places.end() );
            std::shuffle(places.begin(), places.end(), g);
		}
	}
}

Map *MakeWarehouseMap(int columns, int rows, int corridor, int shelfWidth, int shelfHeight, int leftMargin, int rightMargin)
{
	int mapWidth = leftMargin+rightMargin+columns*shelfWidth+(columns-1)*corridor+2;
	int mapHeight = rows*shelfHeight+(rows+1)*corridor+2;
	Map *map = new Map(mapWidth, mapHeight);
	// put a 1-width border on the whole map
	map->SetTerrainType(0, 0, 0, mapHeight-1, kTrees);
	map->SetTerrainType(mapWidth-1, 0, mapWidth-1, mapHeight-1, kTrees);
	map->SetTerrainType(0, mapHeight-1, mapWidth-1, mapHeight-1, kTrees);
	map->SetTerrainType(0, 0, mapWidth-1, 0, kTrees);

	for (int x = 1+leftMargin; x+shelfWidth+rightMargin < mapWidth; x += shelfWidth+corridor)
	{
		for (int y = 1+corridor; y+shelfHeight+corridor < mapHeight; y += shelfHeight+corridor)
		{
			map->SetRectHeight(x, y, x+shelfWidth-1, y+shelfHeight-1, 0, kTrees);
		}
	}
	return map;
}
