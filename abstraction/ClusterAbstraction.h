/*
 *  $Id: ClusterAbstraction.h
 *  hog2
 *
 *  Created by Renee Jansen on 06/06/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef CLUSTERABSTRACTION_H
#define CLUSTERABSTRACTION_H

#include "MapAbstraction.h"
#include "Graph.h" 
#include "Path.h"

#include <unordered_map>
#include <vector>

typedef enum{HORIZONTAL,VERTICAL} Orientation;

const int MAX_ENTRANCE_WIDTH = 6;

/*
 * Set up hash map to cache paths. These are used to easily find a map-level path 
 * from an abstract path. Paths are hashed by pointers to the abstract edges. 
 */ 
namespace clusterUtil {

	struct EdgeEqual {
		bool operator()(const edge* e1, const edge* e2) const
		{return e1->getEdgeNum() == e2->getEdgeNum();}
	};

	struct EdgeHash {
		size_t operator()(const edge *e) const
		{ return (size_t)(e->getEdgeNum()); }
	};
      
  typedef std::unordered_map<edge*,path*,
															clusterUtil::EdgeHash, 
															clusterUtil::EdgeEqual > PathLookupTable;
}

class Cluster {
public:
  Cluster(int id, int row, int col, int horizOrigin, int vertOrigin,int width, int height)
    :m_id(id),m_row(row),m_column(col),
     m_horizOrigin(horizOrigin),m_vertOrigin(vertOrigin),
     m_width(width),m_height(height)
  {}

  void AddNode(int);
  int getIthNodeNum(int i) const { return nodes[i];}
  int GetNumNodes() const { return nodes.size(); }
  int getHOrig() const { return m_horizOrigin; }
  int getVOrig() const { return m_vertOrigin; }
  int GetHeight() const { return m_height; }
  int getWidth() const { return m_width; } 
	void addParent(node* n) { parents.push_back(n); } 
	std::vector<node*>&  getParents() { return parents; } 
	std::vector<node*> parents; // each connected component gets its own parent

private:
  int m_id;
  int m_row; // abstract row of this cluster (e.g., 1 for the second clusters horizontally)
  int m_column; // abstract col of this cluster (e.g., 1 for the second clusters vertically)
  int m_horizOrigin; // horizontal/column origin
  int m_vertOrigin; //vertical / row origin
  int m_width; // width of this cluster
  int m_height; // high of this cluster
  std::vector<int> nodes; // node numbers in abstract Graph of the entrances

  
};

class Entrance {
public:
  Entrance(int center1Row, int center1Col, 
					 int cl1Id, int cl2Id,
					 int center1Id, int center2Id,
					 int row, int col, int length,
					 Orientation orientation)
    :m_center1Row(center1Row), m_center1Col(center1Col),
     m_cluster1Id(cl1Id), m_cluster2Id(cl2Id),
     m_center1Id(center1Id), m_center2Id(center2Id),
     m_row(row), m_col(col), m_length(length),
     m_orientation(orientation) {}

  int getOrientation() {return m_orientation;}
  int getRow() {return m_row;}
  int getCol() {return m_col;}
  void setCluster1Id(int id) {m_cluster1Id=id;}
  void setCluster2Id(int id) {m_cluster2Id=id;}  
  int getCenter1Row() { return m_center1Row; }
  int getCenter1Col() { return m_center1Col; }
  int getCluster1Id() { return m_cluster1Id; }
  int getCluster2Id() { return m_cluster2Id; } 
  

private:
  int m_center1Row; //y-coordinate of entrance in top / left cluster
  int m_center1Col; //x-coordinate of entrance in top / left cluster
  int m_cluster1Id; //top / left cluster's id
  int m_cluster2Id; //bottom / right cluster's id
  int m_center1Id; //node number of the entrance in the top / left cluster
  int m_center2Id; //node number of the entrance in the bottom / right cluster
  int m_row; // abstract row of the leftmost adjacent cluster
  int m_col; // abstract col of the uppermost adjacent cluster
  int m_length; // length of the entrance
  Orientation m_orientation;

};


/** 
 * Cluster abstraction for HPA* algorithm as described in (Botea,Mueller,Schaeffer 2004). 
 * Source code based on HPA* code found at http://www.cs.ualberta.ca/~adib/Home/Download/hpa.tgz
 */
class ClusterAbstraction : public MapAbstraction {
public:
  ClusterAbstraction(Map *map, int _clusterSize);
  ~ClusterAbstraction();
	MapAbstraction* Clone(Map* map)
	{ return new ClusterAbstraction(map, clusterSize); }

	int getClusterSize() { return clusterSize; };  
  bool Pathable(node* start, node* goal);
  void VerifyHierarchy() {}
  void removeNodes(node* start, node* goal);
	void RemoveNode(node*) {}
  void RemoveEdge(edge*, unsigned int) {}
  void AddNode(node*) {}
  void AddEdge(edge*, unsigned int) {}
  void RepairAbstraction() {}
	node* insertNode(node* n, int& expanded, int& touched); 
	path* getCachedPath(edge* e);
	node* getLowLevelNode(node* abstract);
	int getClusterIdFromNode(node* n);


	//move back to private later
	int getClusterIdFromCoord(int row, int col) const;
	void printMapCoord(node* n);
	void printPathAsCoord(path* p);
	virtual void OpenGLDraw() const;

private:
  int min(int, int);
  void createClustersAndEntrances();
  void createAbstractGraph();  
  void addCluster(Cluster c);
  void createHorizEntrances(int, int, int, int, int);
  void createVertEntrances(int, int, int, int, int);
  void linkEntrancesAndClusters();
  void addAbsNodes(Graph* g);
  void computeClusterPaths(Graph* g);
  void addEntrance(Entrance e);
  int getClusterId(int row, int col) const;

  Cluster& getCluster(int id);

  int clusterSize;
  int rows; //rows of clusters
  int columns; //columns of clusters

  std::vector<Cluster> clusters;
  std::vector<Entrance> entrances;
  clusterUtil::PathLookupTable paths;
  clusterUtil::PathLookupTable temp;
		std::vector<path*> newPaths;
  int nodeExists(const Cluster& c,double x,double y, Graph* g);
  void setUpParents(Graph* g);

	void buildNodeIntoParent(node *n, node *parent);
	void abstractionBFS(node *which, node *parent, int cluster,int numOrigNodes,int numNodesAfter);
	void createConnectivityGraph();
	void connectedBFS(node *which, node *parent);
};



#endif
