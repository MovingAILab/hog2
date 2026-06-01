//
//  Voxels.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef Voxels_h
#define Voxels_h

#include "SearchEnvironment.h"
#include <vector>
#include <cstdint>

struct OctreeIndex {
	int layer : 4;
	int index : 22;
	int voxel : 6;
};

struct voxelState {
	OctreeIndex i;
	uint16_t x, y, z;
};

bool operator==(const voxelState &v1, const voxelState &v2);


enum voxelAction {
	kXpYpZp,
	kXpYpZm,
	kXpYpZs,
	kXpYmZp,
	kXpYmZm,
	kXpYmZs,
	kXpYsZp,
	kXpYsZm,
	kXpYsZs,
	kXmYpZp,
	kXmYpZm,
	kXmYpZs,
	kXmYmZp,
	kXmYmZm,
	kXmYmZs,
	kXmYsZp,
	kXmYsZm,
	kXmYsZs,
	kXsYpZp,
	kXsYpZm,
	kXsYpZs,
	kXsYmZp,
	kXsYmZm,
	kXsYmZs,
	kXsYsZp,
	kXsYsZm,
	kXsYsZs
};

class Voxels : public SearchEnvironment<voxelState, voxelAction> {
public:
	Voxels(const char *filename);
	~Voxels();
	void Export(const char *filename);
	void GetSuccessors(const voxelState &nodeID, std::vector<voxelState> &neighbors) const;
	void GetActions(const voxelState &nodeID, std::vector<voxelAction> &actions) const;
	void ApplyAction(voxelState &s, voxelAction a) const;
	bool InvertAction(voxelAction &a) const;
	
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const voxelState &node1, const voxelState &node2) const;
	double GCost(const voxelState &node1, const voxelState &node2) const;
	double GCost(const voxelState &node, const voxelAction &act) const;
	bool GoalTest(const voxelState &node, const voxelState &goal) const;
	
	uint64_t GetStateHash(const voxelState &node) const;
	void GetStateFromHash(uint64_t parent, voxelState &s);
	
	uint64_t GetActionHash(voxelAction act) const;
	
//	void OpenGLDraw() const;
//	void OpenGLDraw(const voxelState&) const;
//	void OpenGLDraw(const voxelState&, const voxelState&, float) const;
//	void OpenGLDraw(const voxelState&, const voxelAction&) const;
//	void GLLabelState(const voxelState&, const char *) const;
//	void GLDrawLine(const voxelState &x, const voxelState &y) const;

	void Draw(Graphics::Display &display) const;

private:
	
	struct voxelWorld {
		uint32_t header;
		float voxelSize;
		uint64_t numVoxelsGrids;
		float minbounds[4];
		float maxbounds[4];
		uint64_t *morton;
		uint64_t *grid;
	};
	
	std::vector<uint64_t> mLayer0VoxelGrids;
	
	struct OctreeNode {
		float coordinates[3]; //x,y,z
		OctreeIndex parent;
		OctreeIndex firstChild;
		OctreeIndex neighbors[6];
	};
	
	typedef std::vector<OctreeNode> OctreeLayer;
	std::vector<OctreeLayer> mOctreeLayers;

	void AddVoxelCubeToOctree(uint64_t values, point3d p);
	
	voxelWorld w;
	point3d GetVoxelCoordinate(uint64_t morton, float voxelSize, const float minbounds[4]) const;

	// "Insert" two 0 bits after each of the 10 low bits of x
	uint32_t Part1By2(uint32_t x) const;
	uint32_t EncodeMorton3(uint32_t x, uint32_t y, uint32_t z) const;
	uint32_t Compact1By2(uint32_t x) const;
	uint32_t DecodeMorton3X(uint32_t code) const;
	uint32_t DecodeMorton3Y(uint32_t code) const;
	uint32_t DecodeMorton3Z(uint32_t code) const;
	size_t GetIndex(size_t x, size_t y, size_t z) const;
	void GetCoordsForIndex(size_t i, size_t& x, size_t& y, size_t& z) const;
};


#endif /* Voxels_h */
