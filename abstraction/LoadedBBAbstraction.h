/*
 *  LoadedBBAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/12/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include "GraphAbstraction.h"
#include "GLUtil.h"

#include <cassert>

#ifndef LOADEDBBABSTRACTION_H
#define LOADEDBBABSTRACTION_H


class LoadedBBAbstraction : public GraphAbstraction {
public:
  LoadedBBAbstraction(char *, char *);
  virtual ~LoadedBBAbstraction();
	virtual void VerifyHierarchy();
  double h(node *a, node *b);
	//  virtual void draw();
	//  virtual void rebuild();
	//	virtual int GetRevision() { return 0; }
	
  bool Pathable(node *from, node *to);
  bool Pathable(unsigned int from, unsigned int to);
	//path *getQuickPath(node *from, node *to);
  void GetNumAbstractGraphs(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
  //node *GetNthParent(node *, int n);
  
  //void ClearMarkedNodes();
  void clearDisplayLists();
	void OpenGLDraw() const;
	void ToggleDrawAbstraction(int which);
	void DrawLevelConnections(node *n) const;
	void DrawGraph(Graph *g) const;
	recVec GetNodeLoc(node *n) const;
	
  // first step to allowing simple loaded adjustments...
//  virtual void RemoveNode(node *n);
//  void RemoveEdge(edge *e, unsigned int absLevel);
	virtual void AddNode(node *n);
	virtual void AddEdge(edge *e, unsigned int absLevel);
  // this must be called after RemoveEdge, but RemoveNode will take care of itself
  //void RepairAbstraction();
  node *findNodeParent(node *n);
  edge *findEdgeParent(edge *e, unsigned int absLevel);
private:
	class BoundingBox {
	public:
		bool pointInBox(double x, double y, double z);
		void OpenGLDraw() const;
		double x1, x2, y1, y2, z1, z2;
	};

		std::vector<BoundingBox> boxes;
	Graph *abstractGraph(Graph *g);
	void cleanMemory();
	void buildAbstractions(Graph *g);
	void loadBoxes(char *boxFile);
	Graph *loadGraph(char *fname);
	node *createNewParent(Graph *, node *n);
		void addNodeToParent(node *n, node *parent);
		void addNeighborsInBox(Graph *g, node *n, int which, node *parent);
		int findBoundingBox(node *n);
		unsigned long levelDraw;

		virtual void RemoveNode(node *) { assert(false); }
		/** remove edge from abstraction */
		virtual void RemoveEdge(edge *, unsigned int) { assert(false); }
		virtual void RepairAbstraction() { }
};

#endif
