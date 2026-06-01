/*
 *  $Id: Heap.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/29/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

// HOG File

#ifndef HEAP_H
#define HEAP_H

#include <vector>
#include <list>

#define DEFAULT_SIZE 10
#include "Graph.h"

/**
 * A simple & efficient Heap class which uses Graph objects.
 */

class Heap {
public:
  Heap(int s = DEFAULT_SIZE);
  ~Heap();
	unsigned int size();
  void Add(graph_object *val);
  void DecreaseKey(graph_object *val);
  bool IsIn(graph_object *val);
  graph_object *Remove();
  bool Empty();
private:
  void HeapifyUp(int index);
  void HeapifyDown(int index);
  std::vector<graph_object *> _elts;
  int count;
};

#endif
