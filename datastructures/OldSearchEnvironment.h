/*
 *  SearchEnvironment.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/5/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef OLDSEARCHENVIRONMENT_H
#define OLDSEARCHENVIRONMENT_H

#include "Graph.h"
#include "Map.h"

namespace OldSearchCode {

	class SearchEnvironment
	{
	public:
		virtual ~SearchEnvironment() {}
		virtual void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors) = 0;
		virtual double heuristic(uint32_t node1, uint32_t node2) = 0;
		virtual double gcost(uint32_t node1, uint32_t node2) = 0;
		virtual bool goalTest(uint32_t node, uint32_t goal) { return (node == goal); }
	};

	class MapSearchEnvironment : public SearchEnvironment
	{
	public:
		MapSearchEnvironment(Map *_map) :map(_map) {  }
		~MapSearchEnvironment() {}
		void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors);
		double heuristic(uint32_t node1, uint32_t node2);
		double gcost(uint32_t node1, uint32_t node2);
	private:
		Map *map;
	};

	class GraphSearchEnvironment : public SearchEnvironment
	{
	public:
		GraphSearchEnvironment(Graph *_graph) :g(_graph) {  }
		~GraphSearchEnvironment() {}
		void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors);
		double heuristic(uint32_t node1, uint32_t node2);
		double gcost(uint32_t node1, uint32_t node2);
	private:
		Graph *g;
	};

	class MapGraphSearchEnvironment : public SearchEnvironment
	{
	public:
		MapGraphSearchEnvironment(Map *_map, Graph *_graph) :g(_graph), m(_map) {  }
		~MapGraphSearchEnvironment() {}
		void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors);
		double heuristic(uint32_t node1, uint32_t node2);
		double gcost(uint32_t node1, uint32_t node2);
	private:
		Graph *g;
		Map *m;
	};
	
}

#endif
