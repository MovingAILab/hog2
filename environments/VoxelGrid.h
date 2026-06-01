//
//  VoxelGrid.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef VoxelGrid_h
#define VoxelGrid_h

#include "SearchEnvironment.h"
#include <vector>
#include "BitMap.h"

struct voxelGridState {
	voxelGridState() {}
	voxelGridState(uint16_t a, uint16_t b, uint16_t c)
	:x(a), y(b), z(c) {}
	uint16_t x, y, z;
};

namespace std {
	template <>
	struct hash<voxelGridState>
	{
		std::size_t operator()(const voxelGridState& k) const
		{
			return ((size_t)k.x<<32)|((size_t)k.y<<16)|k.z;
		}
	};
	
}

bool operator==(const voxelGridState &v1, const voxelGridState &v2);

std::ostream &operator<<(std::ostream &out, const voxelGridState &v);

typedef uint8_t voxelGridAction;

class VoxelGrid : public SearchEnvironment<voxelGridState, voxelGridAction> {
public:
	VoxelGrid(int x, int y, int z);
	VoxelGrid(const char *filename);
	~VoxelGrid();
  
  void And(VoxelGrid *g);
  void Or(VoxelGrid *g);
  void Save(const char *filename);
	void SaveInMinBB(const char *filename);
	void GetSuccessors(const voxelGridState &nodeID, std::vector<voxelGridState> &neighbors) const;
	void GetActions(const voxelGridState &nodeID, std::vector<voxelGridAction> &actions) const;
	void ApplyAction(voxelGridState &s, voxelGridAction a) const;
	bool InvertAction(voxelGridAction &a) const;

	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const voxelGridState &node1, const voxelGridState &node2) const;
	double GCost(const voxelGridState &node1, const voxelGridState &node2) const;
	double GCost(const voxelGridState &node, const voxelGridAction &act) const;
	bool GoalTest(const voxelGridState &node, const voxelGridState &goal) const;

	uint64_t GetStateHash(const voxelGridState &node) const;
	void GetStateFromHash(uint64_t parent, voxelGridState &s);

	uint64_t GetActionHash(voxelGridAction act) const;

	bool IsBlocked(const voxelGridState &s) const;
	bool IsBlocked(uint16_t x, uint16_t y, uint16_t z) const
	{ return IsBlocked({x, y, z}); }
	void SetBlocked(const voxelGridState &s, bool block);
	void SetBlocked(uint16_t x, uint16_t y, uint16_t z, bool block)
	{ SetBlocked({x, y, z}, block); }
	void GetLimits(int &x, int &y, int &z) const { x = xWidth; y = yWidth; z = zWidth; }
	bool Legal(const voxelGridState &s) const;
	bool CanMove(const voxelGridState &s1, const voxelGridState &s2) const;
	voxelGridState GetRandomState();
	
//	void OpenGLDraw() const;
//	void OpenGLDraw(const voxelGridState&) const;
//	void OpenGLDraw(const voxelGridState&, const voxelGridState&, float) const;
//	void OpenGLDraw(const voxelGridState&, const voxelGridAction&) const;
//	void GLLabelState(const voxelGridState&, const char *) const;
//	void GLDrawLine(const voxelGridState &x, const voxelGridState &y) const;
	void Draw(Graphics::Display &display) const;

	void GetGLCoordinate(const voxelGridState &, point3d &) const;
	void GetGLCornerCoordinate(const voxelGridState &, point3d &) const;
	void Fill(voxelGridState);
	void Invert();
	BitMapPic *GetImage(int face);
	void SetUseEfficientDraw(bool e)
	{
		efficient = e;
		SetUpDrawBuffers();
	}
private:
	bool efficient;
	void SetUpDrawBuffers();
	void EfficientDraw() const;

	void GetActionOffsets(voxelGridAction a, int &x, int &y, int &z) const;
	voxelGridAction MakeAction(int &x, int &y, int &z) const;
	int GetIndex(const voxelGridState &s) const;
	int GetIndex(int x, int y, int z) const;
	void GetCoordinates(int index, int &x, int &y, int &z) const;
	void GetCoordinates(int index, voxelGridState &s) const;
	std::vector<bool> voxels;
	int xWidth, yWidth, zWidth;
	std::vector<float> vertices;
	std::vector<uint32_t> indices;
};


#endif /* VoxelGrid_h */
