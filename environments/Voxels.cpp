//
//  Voxels.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include "Voxels.h"
#include <cinttypes>

//// id at each layer of octree
//struct octTreeLink {
//	int layer : 3;
//	int index : 23;
//	int subid : 6; // only valid at layer 0
//};
//
//struct octTreeNode {
//	octTreeLink neighbors[6];
//	point3d origin;
//	float level;
//	octTreeLink parent;
//	octTreeLink firstChild; // (index) of first of 8 children
//};

// get successors:
//  * returns neighbors at the current level or a higher level if none at current level
//  * When we expand a node, we might just generate children as successors [PEA*]

bool operator==(const voxelState &v1, const voxelState &v2)
{
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}


Voxels::Voxels(const char *filename)
{
	printf("OctreeIndex size: %lu\n",  sizeof(OctreeIndex));
	FILE *f = fopen(filename, "r");
	//FILE *f = fopen("/Users/nathanst/Desktop/3dmaps/Simple_test3dnav.3dnav", "r");
	if (f == 0)
	{
		printf("Error opening file\n");
		exit(0);
	}
	point3d minPoint;
	//voxelWorld w;
	fread(&w.header, sizeof(w.header), 1, f);
	printf("Header is 0x%X\n", w.header);
	fread(&w.voxelSize, sizeof(w.voxelSize), 1, f);
	printf("Voxel size is %f\n", w.voxelSize);
	fread(&w.numVoxelsGrids, sizeof(w.numVoxelsGrids), 1, f);
	printf("%" PRId64 " voxel grids to follow\n", w.numVoxelsGrids);
	fread(w.minbounds, sizeof(w.minbounds[0]), 4, f);
	fread(w.maxbounds, sizeof(w.maxbounds[0]), 4, f);
	printf("Min bounds: ");
	minPoint.x = w.minbounds[0];
	minPoint.y = w.minbounds[1];
	minPoint.z = w.minbounds[2];
	int maxWidth = 0;
	for (int x = 0; x < 4; x++)
	{
		printf("%f ", w.minbounds[x]);
		maxWidth = std::max(maxWidth, int(w.maxbounds[x]-w.minbounds[x]));
	}
	maxWidth /= w.voxelSize;
	printf("\n");
	printf("Max bounds: ");
	for (int x = 0; x < 4; x++)
	{
		printf("%f ", w.maxbounds[x]);
	}
	printf("\n");
	printf("Max dimension width = %d\n", maxWidth);
	w.morton = new uint64_t[w.numVoxelsGrids];
	w.grid = new uint64_t[w.numVoxelsGrids];
	for (int x = 0; x < w.numVoxelsGrids; x++)
	{
		assert(fread(&w.morton[x], sizeof(w.morton[x]), 1, f) == 1);
		assert(fread(&w.grid[x], sizeof(w.grid[x]), 1, f) == 1);
		printf("0x%llX\n", w.morton[x]);
		point3d p = GetVoxelCoordinate(w.morton[x], w.voxelSize, w.minbounds);
		printf("(%f, %f, %f)\n", p.x, p.y, p.z);
		printf("0x%llX\n", w.grid[x]);
		p -= minPoint;
		p /= (int)w.voxelSize;
		AddVoxelCubeToOctree(w.grid[x], p);
		for (int i = 0; i < 64; i++)
		{
			size_t a, b, c;
			GetCoordsForIndex(i, a, b, c);
			if ((w.grid[x]>>i)&1) // blocked
			{
				//printf("{%f, %f, %f} ", (p.x+a*w.voxelSize)/8, (p.y+b*w.voxelSize)/8, (p.z+c*w.voxelSize)/8);
			}
		}
		printf("\n");
	}
	
	printf("\n");
	
	fclose(f);
}

Voxels::~Voxels()
{
	
}

void Voxels::Export(const char *filename)
{
	FILE *f = fopen(filename, "w+");
	if (f == 0)
		return;
	// number of voxels outside min bounded box
	int buffer = 50;
	int minx = (int)((w.maxbounds[0]-w.minbounds[0])/w.voxelSize);
	int miny = (int)((w.maxbounds[1]-w.minbounds[1])/w.voxelSize);
	int minz = (int)((w.maxbounds[2]-w.minbounds[2])/w.voxelSize);
	int maxx = 0;
	int maxy = 0;
	int maxz = 0;
	
	for (int x = 0; x < w.numVoxelsGrids; x++)
	{
		point3d p = GetVoxelCoordinate(w.morton[x], w.voxelSize, w.minbounds);
		for (int i = 0; i < 64; i++)
		{
			size_t a, b, c;
			if ((w.grid[x]>>i)&1) // blocked
			{
				GetCoordsForIndex(i, a, b, c);
				minx = std::min(minx, int(((p.x+a*w.voxelSize)-w.minbounds[0])/w.voxelSize));
				miny = std::min(miny, int(((p.y+b*w.voxelSize)-w.minbounds[1])/w.voxelSize));
				minz = std::min(minz, int(((p.z+c*w.voxelSize)-w.minbounds[2])/w.voxelSize));
				maxx = std::max(maxx, int(((p.x+a*w.voxelSize)-w.minbounds[0])/w.voxelSize));
				maxy = std::max(maxy, int(((p.y+b*w.voxelSize)-w.minbounds[1])/w.voxelSize));
				maxz = std::max(maxz, int(((p.z+c*w.voxelSize)-w.minbounds[2])/w.voxelSize));
			}
		}
	}

	
	fprintf(f, "voxel %d %d %d\n", maxx-minx+1+2*buffer, maxy-miny+1+2*buffer, maxz-minz+1+2*buffer);

	for (size_t x = 0; x < w.numVoxelsGrids; x++)
	{
		point3d p = GetVoxelCoordinate(w.morton[x], w.voxelSize, w.minbounds);
		for (int i = 0; i < 64; i++)
		{
			size_t a, b, c;
			if ((w.grid[x]>>i)&1) // blocked
			{
				GetCoordsForIndex(i, a, b, c);
				fprintf(f, "%d %d %d\n",
						int(((p.x+a*w.voxelSize)-w.minbounds[0])/w.voxelSize)-minx+buffer,
						int(((p.y+b*w.voxelSize)-w.minbounds[1])/w.voxelSize)-miny+buffer,
						int(((p.z+c*w.voxelSize)-w.minbounds[2])/w.voxelSize)-minz+buffer);
			}
		}
	}

	
	fclose(f);
}


void Voxels::GetSuccessors(const voxelState &nodeID, std::vector<voxelState> &neighbors) const
{
	
}

void Voxels::GetActions(const voxelState &nodeID, std::vector<voxelAction> &actions) const
{
	
}

void Voxels::ApplyAction(voxelState &s, voxelAction a) const
{
	
}

bool Voxels::InvertAction(voxelAction &a) const
{
	return false;
}


/** Heuristic value between two arbitrary nodes. **/
double Voxels::HCost(const voxelState &node1, const voxelState &node2) const
{
	return 0;
}

double Voxels::GCost(const voxelState &node1, const voxelState &node2) const
{
	return 1;
}

double Voxels::GCost(const voxelState &node, const voxelAction &act) const
{
	return 1;
}

bool Voxels::GoalTest(const voxelState &node, const voxelState &goal) const
{
	return node == goal;
}


uint64_t Voxels::GetStateHash(const voxelState &node) const
{
	return (((uint64_t)node.x)<<32)|(node.y<<16)|(node.z);
}

void Voxels::GetStateFromHash(uint64_t parent, voxelState &s)
{
	s.z = parent&0xFFFF;
	s.y = (parent>>16)&0xFFFF;
	s.x = (parent>>32)&0xFFFF;
}


uint64_t Voxels::GetActionHash(voxelAction act) const
{
	return (int)act;
}


point3d Voxels::GetVoxelCoordinate(uint64_t morton, float voxelSize, const float minbounds[4]) const
{
	point3d pt(DecodeMorton3X(morton), DecodeMorton3Y(morton), DecodeMorton3Z(morton));
	pt.x*=voxelSize*4;
	pt.x += minbounds[0];
	pt.y*=voxelSize*4;
	pt.y += minbounds[1];
	pt.z*=voxelSize*4;
	pt.z += minbounds[2];
	//const size_t ix = static_cast<size_t>(floorf((coords.x - mBounds.MinX()) / (voxelSize * 4)));
	return pt;
}

void Voxels::AddVoxelCubeToOctree(uint64_t values, point3d p)
{
//	int index = mLayer0VoxelGrids.size();
	mLayer0VoxelGrids.push_back(values);
}

//void Voxels::OpenGLDraw() const
//{
//	double xRange = max(w.maxbounds[0],-w.minbounds[0]);
//	double yRange = max(w.maxbounds[1],-w.minbounds[1]);
//	double zRange = max(w.maxbounds[2],-w.minbounds[2]);
//	double range = std::max(xRange, std::max(yRange, zRange));
//	for (size_t x = 0; x < w.numVoxelsGrids; x++)
//	{
//		point3d p = GetVoxelCoordinate(w.morton[x], w.voxelSize, w.minbounds);
//		bool drawFrame = false;
//		for (int i = 0; i < 64; i++)
//		{
//			size_t a, b, c;
//			if ((w.grid[x]>>i)&1) // blocked
//			{
//				GetCoordsForIndex(i, a, b, c);
//				drawFrame = true;
//				GLfloat rr, gg, bb;
//				rr = (1.+(p.x+a*w.voxelSize)/range)/2.0;
//				gg = (1.+(p.y+b*w.voxelSize)/range)/2.0;
//				bb = (1.+(p.z+c*w.voxelSize)/range)/2.0;
//				// 7?
//				rr = Colors::GetColor(rr, 0, 1, 7).r;
//				gg = Colors::GetColor(bb, 0, 1, 9).g*0.9;
//				bb = Colors::GetColor(bb, 0, 1, 9).b;
//				glColor3f(gg, rr, bb);
//				glEnable(GL_LIGHTING);
//				DrawBox((p.x+a*w.voxelSize+0.5*w.voxelSize)/range,
//						-(p.y+b*w.voxelSize+0.5*w.voxelSize)/range,
//						(p.z+c*w.voxelSize+0.5*w.voxelSize)/range,
//						1*(w.voxelSize/2.)/range);
//			}
//		}
//		//if (drawFrame)
//		if (0)
//		{
//			//DrawBoxFrame(p.x+w.voxelSize*2, p.y+w.voxelSize*2, p.z+w.voxelSize*2, w.voxelSize*2);
//			glDisable(GL_LIGHTING);
//			glColor4f(1, 1, 1, 1);
//			DrawBoxFrame((p.x+2*w.voxelSize)/range,
//						 (p.y+2*w.voxelSize)/range,
//						 (p.z+2*w.voxelSize)/range,
//						 2*w.voxelSize/range);
//		}
//	}
//
//}
//
//void Voxels::OpenGLDraw(const voxelState&) const
//{
//	
//}
//
//void Voxels::OpenGLDraw(const voxelState&, const voxelState&, float) const
//{
//	
//}
//
//void Voxels::OpenGLDraw(const voxelState&, const voxelAction&) const
//{
//	
//}
//
//void Voxels::GLLabelState(const voxelState&, const char *) const
//{
//	
//}
//
//void Voxels::GLDrawLine(const voxelState &x, const voxelState &y) const
//{
//	
//}

void Voxels::Draw(Graphics::Display &display) const
{
	
}


// "Insert" two 0 bits after each of the 10 low bits of x
uint32_t Voxels::Part1By2(uint32_t x) const
{
	x &= 0x000003ff;                  // x = ---- ---- ---- ---- ---- --98 7654 3210
	x = (x ^ (x << 16)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
	x = (x ^ (x <<  8)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
	x = (x ^ (x <<  4)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
	x = (x ^ (x <<  2)) & 0x09249249; // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
	return x;
}

uint32_t Voxels::EncodeMorton3(uint32_t x, uint32_t y, uint32_t z) const
{
	return (Part1By2(z) << 2) + (Part1By2(y) << 1) + Part1By2(x);
}

// Inverse of Part1By2 - "delete" all bits not at positions divisible by 3
uint32_t Voxels::Compact1By2(uint32_t x) const
{
	x &= 0x09249249;                  // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
	x = (x ^ (x >>  2)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
	x = (x ^ (x >>  4)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
	x = (x ^ (x >>  8)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
	x = (x ^ (x >> 16)) & 0x000003ff; // x = ---- ---- ---- ---- ---- --98 7654 3210
	return x;
}

uint32_t Voxels::DecodeMorton3X(uint32_t code) const
{
	return Compact1By2(code >> 0);
}

uint32_t Voxels::DecodeMorton3Y(uint32_t code) const
{
	return Compact1By2(code >> 1);
}

uint32_t Voxels::DecodeMorton3Z(uint32_t code) const
{
	return Compact1By2(code >> 2);
}

size_t Voxels::GetIndex(size_t x, size_t y, size_t z) const
{
	return(x * 16 + y * 4 + z);
}

void Voxels::GetCoordsForIndex(size_t i, size_t& x, size_t& y, size_t& z) const
{
	x = i >> 4;
	y = (i & 0x0F) >> 2;
	z = (i & 0x03);
}
