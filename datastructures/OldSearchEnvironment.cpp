/*
 *  SearchEnvironment.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/5/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "OldSearchEnvironment.h"

namespace OldSearchCode {

	void MapSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
	{
		int x1, y1;
		bool up=false, down=false;
		x1 = nodeID>>16; y1 = nodeID&0xFFFF;
		if ((map->GetTerrainType(x1, y1+1) == kGround))
		{
			down = true;
			neighbors.push_back((x1<<16)|(y1+1));
		}
		if ((map->GetTerrainType(x1, y1-1) == kGround))
		{
			up = true;
			neighbors.push_back((x1<<16)|(y1-1));
		}
		if ((map->GetTerrainType(x1-1, y1) == kGround))
		{
			if ((up && (map->GetTerrainType(x1-1, y1-1) == kGround)))
				neighbors.push_back(((x1-1)<<16)|(y1-1));
			if ((down && (map->GetTerrainType(x1-1, y1+1) == kGround)))
				neighbors.push_back(((x1-1)<<16)|(y1+1));
			neighbors.push_back(((x1-1)<<16)| y1);
		}
		if ((map->GetTerrainType(x1+1, y1) == kGround))
		{
			if ((up && (map->GetTerrainType(x1+1, y1-1) == kGround)))
				neighbors.push_back(((x1+1)<<16)|(y1-1));
			if ((down && (map->GetTerrainType(x1+1, y1+1) == kGround)))
				neighbors.push_back(((x1+1)<<16)|(y1+1));
			neighbors.push_back(((x1+1)<<16)| y1);
		}
	}

	double MapSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
	{
		return heuristic(node1, node2);
	}

	double MapSearchEnvironment::heuristic(uint32_t node1, uint32_t node2)
	{
		int x1, x2, y1, y2;
		x1 = node1>>16; y1 = node1&0xFFFF;
		x2 = node2>>16; y2 = node2&0xFFFF;
		double a = ((x1>x2)?(x1-x2):(x2-x1));
		double b = ((y1>y2)?(y1-y2):(y2-y1));
		return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
		//return ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	}

	/** GraphSearchEnvironment
	*/

	void GraphSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
	{
		node *n1 = g->GetNode(nodeID);
		neighbor_iterator ni = n1->getNeighborIter();
		for (int next = n1->nodeNeighborNext(ni); next != -1; next = n1->nodeNeighborNext(ni))
		{
			neighbors.push_back(next);
		}
	}

	double GraphSearchEnvironment::heuristic(uint32_t node1, uint32_t node2)
	{
		edge *e = g->FindEdge(node1, node2);
		if (e)
			return e->GetWeight();
		return 0;
	}

	double GraphSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
	{
		return heuristic(node1, node2);
	}

	
	/** MapGraphSearchEnvironment
	 */
	
	void MapGraphSearchEnvironment::getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
	{
//		node *n1 = g->GetNode(nodeID);
//		neighbor_iterator ni = n1->getNeighborIter();
//		for (int next = n1->nodeNeighborNext(ni); next != -1; next = n1->nodeNeighborNext(ni))
//		{
//			neighbors.push_back(next);
//		}
		node *n = g->GetNode(nodeID);
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			neighbors.push_back(e->getTo());
		}
	}
	
	double MapGraphSearchEnvironment::heuristic(uint32_t state1, uint32_t state2)
	{
		int x1 = g->GetNode(state1)->GetLabelL(9);
		int y1 = g->GetNode(state1)->GetLabelL(10);
		int x2 = g->GetNode(state2)->GetLabelL(9);
		int y2 = g->GetNode(state2)->GetLabelL(10);
//		int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
//		int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
//		int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
//		int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
		
		double a = ((x1>x2)?(x1-x2):(x2-x1));
		double b = ((y1>y2)?(y1-y2):(y2-y1));
		return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	}
	
	double MapGraphSearchEnvironment::gcost(uint32_t node1, uint32_t node2)
	{
		return heuristic(node1, node2);
	}
	
}
