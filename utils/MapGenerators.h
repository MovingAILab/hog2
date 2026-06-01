//
//  Maze.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/5/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

#include "Map.h"

void MakeMaze(Map *map, int width = 1, int startx = -1, int starty = -1);
//void MakeMaze(Map *map, int pathSize);
void MakeMaze(Map *m, float straightPercent, float branchPercent);
void MakeRandomMap(Map *map, int obstacles);
void MakeDesignedMap(Map *map, int obstacleSize, int type);
void MakePseudoMaze(Map *map, int pathSize);
void BuildRandomRoomMap(Map *map, int roomSize, int openingProbability=80);
Map *MakeWarehouseMap(int columns, int rows, int corridor, int shelfWidth, int shelfHeight, int leftMargin, int rightMargin);

#endif /* defined(MAP_GENERATOR_H) */
