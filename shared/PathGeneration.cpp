/*
 *  $Id: pathGeneration.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/30/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "PathGeneration.h"
#include "Map.h"
#include "AStar.h"
#include "MapFlatAbstraction.h"

using namespace GraphAbstractionConstants;

// outputs algorithm/path length/time per step
void generatePaths(char *_map, int mapSizeX, int mapSizeY, int numBuckets, int bucketSize, int pathsPerBucket)
{
//	int numBuckets = (int)(mapSize/4.0);
//	const int pathsPerBucket = 10;
	
	int pathsLeft = numBuckets*pathsPerBucket;
	int iterations = 0;
	int maxIterations = 10*numBuckets*pathsPerBucket;
	Map *map = new Map(_map);
	if ((mapSizeX != -1) && (mapSizeY != -1))
		map->Scale(mapSizeX, mapSizeY);
	else {
		mapSizeX = map->GetMapWidth();
		mapSizeY = map->GetMapHeight();
	}
		
	MapFlatAbstraction *absMap = new MapFlatAbstraction(map);
	Graph *g = absMap->GetAbstractGraph(0);

	node *r1, *r2;
	//  10 maps/bigMaps/600.map  48 151  55 152 7.4142135624
	// bucket, map, sizex, sizey, startx, starty, endx, endy, a* length
	printf("# bucket\tmap\tsizex\tsizey\tfromx\tfromy\ttox\ttoy\tA*len\n");
	std::vector<int> buckets(numBuckets);
	for (int x = 0; x < numBuckets; x++)
		buckets[x] = 0;
	while ((pathsLeft > 0) && (iterations < maxIterations))
	{
		iterations++;
		// try to find two valid points
		do {
			r1 = g->GetRandomNode();
			r2 = g->GetRandomNode();
		} while (!absMap->Pathable(r1, r2) || (r1 == r2) ||
						 ((iterations > maxIterations/2) && (absMap->h(r1, r2) > numBuckets*bucketSize)));
		
		aStar a;

		path *p;
		p = a.GetPath(absMap, r1, r2);
		
		int cnt = 0;
		double length = 0;
		for (path *q = p; q; q = q->next)
		{
			if (q && q->next)
			{
				double t1, t2;
				t1 = q->n->GetLabelL(kFirstData)-q->next->n->GetLabelL(kFirstData);
				t2 = q->n->GetLabelL(kFirstData+1)-q->next->n->GetLabelL(kFirstData+1);
				length += sqrt(t1*t1+t2*t2);
			}
			cnt++;
		}
		if ((length > numBuckets*bucketSize) ||
				((iterations > maxIterations/2) && (buckets[(int)length/bucketSize] == pathsPerBucket)))
		{
			while (p && p->next)
			{
				double t1, t2;
				t1 = p->n->GetLabelL(kFirstData)-p->next->n->GetLabelL(kFirstData);
				t2 = p->n->GetLabelL(kFirstData+1)-p->next->n->GetLabelL(kFirstData+1);
				length -= sqrt(t1*t1+t2*t2);
				path *t = p;
				p = p->next;
				t->next = 0; delete t;
				if (p->next == 0)
				{
					delete p;
					p = 0;
				}
				if (p == 0)
					break;
				if (buckets[(int)length/numBuckets] < pathsPerBucket)
					break;
			}
			if (p && p->next)
			{
				r1 = p->n;
				for (path *q = p; q; q = q->next)
				{
					r2 = q->n;
				}
				delete p;

				p = a.GetPath(absMap, r1, r2);

				length = 0; cnt = 0;
				for (path *q = p; q; q = q->next)
				{
					if (q && q->next)
					{
						double t1, t2;
						t1 = q->n->GetLabelL(kFirstData)-q->next->n->GetLabelL(kFirstData);
						t2 = q->n->GetLabelL(kFirstData+1)-q->next->n->GetLabelL(kFirstData+1);
						length += sqrt(t1*t1+t2*t2);
					}
					cnt++;
				}
			}
		}
		if (p == 0)
			continue;
		if ((length/bucketSize < numBuckets) && (buckets[(int)length/bucketSize] < pathsPerBucket))
		{
			buckets[(int)length/bucketSize]++;
			pathsLeft--;
		}
		else {
			continue;
		}

		int x1, x2, y1, y2;
		x1 = r1->GetLabelL(kFirstData); y1 = r1->GetLabelL(kFirstData+1);
		x2 = r2->GetLabelL(kFirstData); y2 = r2->GetLabelL(kFirstData+1);
		printf("%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%1.2f\n",
					 (int)(length/bucketSize), _map, mapSizeX, mapSizeY, x1, y1, x2, y2, length);
		delete p;
	}
	delete absMap;
}
