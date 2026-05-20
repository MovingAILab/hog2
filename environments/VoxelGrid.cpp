//
//  VoxelGrid.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <deque>
#include "VoxelGrid.h"
#include <unordered_map>
#include "VoxelTriangleExtraction.h"
#include <iostream>

bool operator==(const voxelGridState &v1, const voxelGridState &v2)
{
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

std::ostream &operator<<(std::ostream &out, const voxelGridState &v)
{
	out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
	return out;
}


VoxelGrid::VoxelGrid(int x, int y, int z)
{
	xWidth = x;
	yWidth = y;
	zWidth = z;
	efficient = false;
	voxels.resize(xWidth*yWidth*zWidth);
}

VoxelGrid::VoxelGrid(const char *filename)
{
	efficient = false;
	FILE *f = fopen(filename, "r");
	if (f == 0)
	{
		printf("Error opening file\n");
		exit(0);
	}
	int cnt = fscanf(f, "voxel %d %d %d\n", &xWidth, &yWidth, &zWidth);
	if (cnt != 3)
	{
		printf("Error reading dimensions\n");
		return;
	}
	voxels.resize(xWidth*yWidth*zWidth);
	int a, b, c;
	while ((cnt = fscanf(f, "%d %d %d\n", &a, &b, &c)) == 3)
	{
		voxels[GetIndex(a, b, c)] = true;
	}
	
	fclose(f);

}

VoxelGrid::~VoxelGrid()
{
	
}

void VoxelGrid::And(VoxelGrid *g)
{
  for (int y = 0; y < yWidth; y++)
    {
      for (int z = 0; z < zWidth; z++)
	{
	  for (int x = 0; x < xWidth; x++)
	    {
	      SetBlocked(x, y, z, IsBlocked(x, y, z) && g->IsBlocked(x, y, z));
	    }
	}
    }
}

void VoxelGrid::Or(VoxelGrid *g)
{
  for (int y = 0; y < yWidth; y++)
    {
      for (int z = 0; z < zWidth; z++)
	{
	  for (int x = 0; x < xWidth; x++)
	    {
	      if (g->IsBlocked(x, y, z))
		SetBlocked(x, y, z, true);
	    }
	}
    }
}

void VoxelGrid::Save(const char *filename)
{
	FILE *f = fopen(filename, "w+");
	if (f == 0)
	{
		printf("Error opening file\n");
		exit(0);
	}
	fprintf(f, "voxel %d %d %d\n", xWidth, yWidth, zWidth);
	for (int x = 0; x < xWidth; x++)
	{
		for (int y = 0; y < yWidth; y++)
		{
			for (int z = 0; z < zWidth; z++)
			{
				if (IsBlocked(x, y, z))
					fprintf(f, "%d %d %d\n", x, y, z);
			}
		}
	}
	fclose(f);
}

void VoxelGrid::SaveInMinBB(const char *filename)
{
	int xMax = 0;
	int yMax = 0;
	int zMax = 0;
	int xMin = xWidth;
	int yMin = yWidth;
	int zMin = zWidth;
	
	for (int x = 0; x < xWidth; x++)
	{
		for (int y = 0; y < yWidth; y++)
		{
			for (int z = 0; z < zWidth; z++)
			{
				if (IsBlocked(x, y, z))
				{
					xMax = std::max(xMax, x);
					xMin = std::min(xMin, x);
					yMax = std::max(yMax, y);
					yMin = std::min(yMin, y);
					zMax = std::max(zMax, z);
					zMin = std::min(zMin, z);
				}
			}
		}
	}

	FILE *f = fopen(filename, "w+");
	if (f == 0)
	{
		printf("Error opening file\n");
		exit(0);
	}
	fprintf(f, "voxel %d %d %d\n", xMax-xMin+1, yMax-yMin+1, zMax-zMin+1);
	for (int x = 0; x < xWidth; x++)
	{
		for (int y = 0; y < yWidth; y++)
		{
			for (int z = 0; z < zWidth; z++)
			{
				if (IsBlocked(x, y, z))
					fprintf(f, "%d %d %d\n", x-xMin, y-yMin, z-zMin);
			}
		}
	}
	fclose(f);
}

bool VoxelGrid::IsBlocked(const voxelGridState &s) const
{
	if ((s.x < xWidth) && (s.y < yWidth) && (s.z < zWidth))
		return voxels[GetIndex(s)];
	return true;
}

void VoxelGrid::SetBlocked(const voxelGridState &s, bool block)
{
	if ((s.x < xWidth) && (s.y < yWidth) && (s.z < zWidth))
		voxels[GetIndex(s)] = block;
}

int VoxelGrid::GetIndex(const voxelGridState &s) const
{
	return GetIndex(s.x, s.y, s.z);
}

int VoxelGrid::GetIndex(int x, int y, int z) const
{
	return xWidth*(y*zWidth+z)+x;
}

void VoxelGrid::GetCoordinates(int index, voxelGridState &s) const
{
	s.x = index%xWidth;
	index /= xWidth;
	s.z = index%zWidth;
	s.y = index/zWidth;
}

void VoxelGrid::GetCoordinates(int index, int &x, int &y, int &z) const
{
	x = index%xWidth;
	index /= xWidth;
	z = index%zWidth;
	y = index/zWidth;
}

bool VoxelGrid::Legal(const voxelGridState &s) const
{
	return ((s.x < xWidth) && (s.y < yWidth) && (s.z < zWidth));
}

bool VoxelGrid::CanMove(const voxelGridState &s1, const voxelGridState &s2) const
{
	return (IsBlocked(s1) ||
			IsBlocked(s2) ||
			IsBlocked({s2.x, s1.y, s1.z}) ||
			IsBlocked({s1.x, s2.y, s1.z}) ||
			IsBlocked({s1.x, s1.y, s2.z}) ||
			IsBlocked({s2.x, s2.y, s1.z}) ||
			IsBlocked({s2.x, s1.y, s2.z}) ||
			IsBlocked({s1.x, s2.y, s2.z})) == false;
}

void VoxelGrid::Fill(voxelGridState s)
{
	std::deque<voxelGridState> q;
	q.push_back(s);
	while (!q.empty())
	{
		voxelGridState n = q.front();
		q.pop_front();
		if (IsBlocked(n)) // blocked already
			continue;
		voxels[GetIndex(n)] = true;
		//std::cout << "Inverted " << n << "\n";
		q.push_back(voxelGridState(n.x+1, n.y, n.z));
		q.push_back(voxelGridState(n.x-1, n.y, n.z));
		q.push_back(voxelGridState(n.x, n.y+1, n.z));
		q.push_back(voxelGridState(n.x, n.y-1, n.z));
		q.push_back(voxelGridState(n.x, n.y, n.z+1));
		q.push_back(voxelGridState(n.x, n.y, n.z-1));
	}
	if (efficient)
	SetUpDrawBuffers();
}

void VoxelGrid::Invert()
{
	for (size_t x = 0; x < voxels.size(); x++)
		voxels[x] = !voxels[x];
	if (efficient)
		SetUpDrawBuffers();
}


void VoxelGrid::GetActionOffsets(voxelGridAction a, int &x, int &y, int &z) const
{
	x = (a>>4)&3;
	y = (a>>2)&3;
	z = (a>>0)&3;
	x--;
	y--;
	z--;
}

voxelGridAction VoxelGrid::MakeAction(int &x, int &y, int &z) const
{
	voxelGridAction v = 0;
	v |= (((x+1)&3)<<4);
	v |= (((y+1)&3)<<2);
	v |= (((z+1)&3)<<0);
	return v;
}


void VoxelGrid::GetSuccessors(const voxelGridState &nodeID, std::vector<voxelGridState> &neighbors) const
{
	assert(voxels[GetIndex(nodeID)] == false);
	neighbors.resize(0);
	for (int x = -1; x <= 1; x++)
	{
		for (int y = -1; y <= 1; y++)
		{
			for (int z = -1; z <= 1; z++)
			{
				// This is simple code, but inefficient, because the CanMove checks repeat a lot of tests
				if ((x|y|z) == 0)
					continue;
				voxelGridState s =
				{static_cast<uint16_t>(nodeID.x+x),
					static_cast<uint16_t>(nodeID.y+y),
					static_cast<uint16_t>(nodeID.z+z)};
				if (CanMove(nodeID, s))
					neighbors.push_back(s);
			}
		}
	}
}

void VoxelGrid::GetActions(const voxelGridState &nodeID, std::vector<voxelGridAction> &actions) const
{
	assert(voxels[GetIndex(nodeID)] == false);
	actions.resize(0);

	for (int x = -1; x <= 1; x++)
	{
		for (int y = -1; y <= 1; y++)
		{
			for (int z = -1; z <= 1; z++)
			{
				if ((x|y|z) == 0)
					continue;
				voxelGridState s =
				{static_cast<uint16_t>(nodeID.x+x),
					static_cast<uint16_t>(nodeID.y+y),
					static_cast<uint16_t>(nodeID.z+z)};
				if (CanMove(nodeID, s))
					actions.push_back(MakeAction(x, y, z));
			}
		}
	}
}

void VoxelGrid::ApplyAction(voxelGridState &s, voxelGridAction a) const
{
	int x, y, z;
	GetActionOffsets(a, x, y, z);
	s.x+=x;
	s.y+=y;
	s.z+=z;
}

bool VoxelGrid::InvertAction(voxelGridAction &a) const
{
	return false;
}


/** Heuristic value between two arbitrary nodes. **/
double VoxelGrid::HCost(const voxelGridState &node1, const voxelGridState &node2) const
{
	double xd = abs(node1.x-node2.x);
	double yd = abs(node1.y-node2.y);
	double zd = abs(node1.z-node2.z);
	double three = std::min(xd, std::min(yd, zd));
	xd -= three;
	yd -= three;
	zd -= three;
	double two;
	if (zd == 0)
	{
		two = std::min(xd, yd);
		xd-=two;
		yd-=two;
	}
	else if (xd == 0)
	{
		two = std::min(zd, yd);
		zd-=two;
		yd-=two;
	}
	else if (yd == 0)
	{
		two = std::min(xd, zd);
		xd-=two;
		zd-=two;
	}
	else {
		assert(!"Should not be able to get here.");
	}
	return three*ROOT_THREE + two*ROOT_TWO + xd+yd+zd;
}

double VoxelGrid::GCost(const voxelGridState &node1, const voxelGridState &node2) const
{
	int diff = 0;
	if (node1.x != node2.x)
		diff++;
	if (node1.y != node2.y)
		diff++;
	if (node1.z != node2.z)
		diff++;
	double v[4] = {0, 1, ROOT_TWO, ROOT_THREE};
	return v[diff];
}

double VoxelGrid::GCost(const voxelGridState &node, const voxelGridAction &act) const
{
	assert(!"Action consts not implemented yet");
	return 1;
}

bool VoxelGrid::GoalTest(const voxelGridState &node, const voxelGridState &goal) const
{
	return node == goal;
}


voxelGridState VoxelGrid::GetRandomState()
{
	voxelGridState v;
	do {
		v.x = random()%xWidth;
		v.y = random()%yWidth;
		v.z = random()%zWidth;
	} while (voxels[GetIndex(v)] == true);
	return v;
}


uint64_t VoxelGrid::GetStateHash(const voxelGridState &node) const
{
	return (uint64_t(node.x)<<32)|(uint64_t(node.y)<<16)|uint64_t(node.z);
}

void VoxelGrid::GetStateFromHash(uint64_t parent, voxelGridState &s)
{
	s.z = parent&0xFFFF;
	s.y = (parent>>16)&0xFFFF;
	s.x = (parent>>32)&0xFFFF;
}


uint64_t VoxelGrid::GetActionHash(voxelGridAction act) const
{
	return (int)act;
}

void VoxelGrid::GetGLCoordinate(const voxelGridState &v, point3d &p) const
{
	const double range = std::max(xWidth, std::max(yWidth, zWidth));
	p.x = 2.0*v.x/range-1.0+(-xWidth+range)/range + 1.0/range;
	p.y = 2.0*v.y/range-1.0+(-yWidth+range)/range + 1.0/range;
	p.z = 2.0*v.z/range-1.0+(-zWidth+range)/range + 1.0/range;
}

void VoxelGrid::GetGLCornerCoordinate(const voxelGridState &v, point3d &p) const
{
	const double range = std::max(xWidth, std::max(yWidth, zWidth));
	p.x = 2.0*v.x/range-1.0+(-xWidth+range)/range;
	p.y = 2.0*v.y/range-1.0+(-yWidth+range)/range;
	p.z = 2.0*v.z/range-1.0+(-zWidth+range)/range;
}


void VoxelGrid::SetUpDrawBuffers()
{
	printf("Building pre-computed draw buffers\n");
	vertices.resize(0);
	indices.resize(0);
	std::vector<VoxelUtils::triangle> data;
	VoxelUtils::GetTriangles(this, data);
	std::unordered_map<VoxelUtils::vn, int> index;
	int next = 0;
	for (size_t x = 0; x < data.size(); x++)
	{
		for (int y = 0; y < 3; y++)
		{
			VoxelUtils::vn v = data[x].GetVN(y);
			auto i = index.find(v);
			if (i == index.end())
			{
				index[v] = next;
				next++;

				point3d p;
				GetGLCornerCoordinate({v.v[0], v.v[1], v.v[2]}, p);

				vertices.push_back(p.x);
				vertices.push_back(p.y);
				vertices.push_back(p.z);
				vertices.push_back(v.normal[0]);
				vertices.push_back(v.normal[1]);
				vertices.push_back(v.normal[2]);
				const double range = std::max(xWidth, std::max(yWidth, zWidth))*2;
				if (v.normal[0])
				{
					vertices.push_back(1.0-2.0*v.v[0]/range);
					vertices.push_back(0.0+2.0*v.v[1]/range);
					vertices.push_back(0.5+v.v[2]/range);
				}
				else if (v.normal[1])
				{
					vertices.push_back(0.0+2.0*v.v[1]/range);
					vertices.push_back(0.5+v.v[2]/range);
					vertices.push_back(1.0-2.0*v.v[0]/range);
				}
				else {
					vertices.push_back(0.5+v.v[2]/range);
					vertices.push_back(1.0-2.0*v.v[0]/range);
					vertices.push_back(0.0+2.0*v.v[1]/range);
				}
			}
		}
	}
	printf("%d individual items\n", next);
	// get list of surface triangles, normals, and colors
	for (size_t x = 0; x < data.size(); x++)
	{
		for (int y = 0; y < 3; y++)
		{
			indices.push_back(index[data[x].GetVN(y)]);
		}
	}
}

//void VoxelGrid::EfficientDraw() const
//{
////	glEnable(GL_NORMALIZE);
//	// enable and specify pointers to vertex arrays
//	glEnable(GL_LIGHTING);
//	glEnableClientState(GL_VERTEX_ARRAY);
//	glEnableClientState(GL_NORMAL_ARRAY);
//	glEnableClientState(GL_COLOR_ARRAY);
//	glVertexPointer(3, GL_FLOAT, 9 * sizeof(GLfloat), &vertices[0]);
//	glNormalPointer(GL_FLOAT, 9 * sizeof(GLfloat), &vertices[0] + 3);
//	glColorPointer(3, GL_FLOAT, 9 * sizeof(GLfloat), &vertices[0] + 6);
//	
//	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, &indices[0]);
//	
//	glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
//	glDisableClientState(GL_COLOR_ARRAY);
//	glDisableClientState(GL_NORMAL_ARRAY);
//	glDisable(GL_LIGHTING);
////	glDisable(GL_NORMALIZE);
//}
//
//void VoxelGrid::OpenGLDraw() const
//{
//	if (efficient)
//	{
//		EfficientDraw();
//	}
//	else {
//		glEnable(GL_LIGHTING);
//		glColor3f(0.0, 1.0, 0.0);
//		const double range = std::max(xWidth, std::max(yWidth, zWidth));
//		for (uint16_t x = 0; x < xWidth; x++)
//		{
//			for (uint16_t y = 0; y < yWidth; y++)
//			{
//				for (uint16_t z = 0; z < zWidth; z++)
//				{
//					if (voxels[GetIndex(x, y, z)])
//					{
//						GLfloat rr, gg, bb;
//						rr = 1-(2.0*x/range-1.0+(-xWidth+range)/range);
//						gg = 1+(2.0*y/range-1.0+(-yWidth+range)/range);
//						bb = 1-(2.0*z/range-1.0+(-zWidth+range)/range);
//						// 7?
//						rr = Colors::GetColor(rr, 0, 2, 7).r;
//						gg = Colors::GetColor(bb, 0, 2, 9).g*0.9;
//						bb = Colors::GetColor(bb, 0, 2, 9).b;
//						glColor3f(gg, rr, bb);
//						
//						point3d p;
//						GetGLCoordinate({x, y, z}, p);
//						DrawBox(p.x, p.y, p.z, 1/range);
//					}
//				}
//			}
//		}
//		glDisable(GL_LIGHTING);
//	}
//}
//
//void VoxelGrid::OpenGLDraw(const voxelGridState&) const
//{
//	
//}
//
//void VoxelGrid::OpenGLDraw(const voxelGridState&, const voxelGridState&, float) const
//{
//	
//}
//
//void VoxelGrid::OpenGLDraw(const voxelGridState&, const voxelGridAction&) const
//{
//	
//}
//
//void VoxelGrid::GLLabelState(const voxelGridState&, const char *) const
//{
//	
//}
//
//void VoxelGrid::GLDrawLine(const voxelGridState &x, const voxelGridState &y) const
//{
//	point3d p1, p2;
//	GetGLCoordinate(x, p1);
//	GetGLCoordinate(y, p2);
//	glBegin(GL_LINES);
//	glVertex3f(p1.x, p1.y, p1.z);
//	glVertex3f(p2.x, p2.y, p2.z);
//	glEnd();
//}

void VoxelGrid::Draw(Graphics::Display &display) const
{
	const float range = std::max(xWidth, std::max(yWidth, zWidth));
	uint16_t minz = zWidth;
	uint16_t maxz = 0;
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	for (uint16_t x = 0; x < xWidth; x++)
	{
		for (uint16_t y = 0; y < yWidth; y++)
		{
			for (uint16_t z = 0; z < zWidth; z++)
			{
				if (voxels[GetIndex(x, y, z)])
				{
					minz = std::min(minz, z);
					maxz = std::max(maxz, z);
					break;
				}
			}
		}
	}
	if (minz == maxz)
	{
		maxz++;
		minz--;
	}
	for (uint16_t x = 0; x < xWidth; x++)
	{
		for (uint16_t y = 0; y < yWidth; y++)
		{
			for (uint16_t z = 0; z < zWidth; z++)
			{
				if (voxels[GetIndex(x, y, z)])
				{
					rgbColor c = {0.0f, 0.0f, 0.0f};
					c.b = 1.0f-(z-minz)/(float)(maxz-minz);
					c.r = c.b/2.0f;
					c.g = c.b/2.0f;

					point3d p;
					GetGLCoordinate({x, y, z}, p);
					Graphics::rect r(p.x-1.f/range, p.y-1.f/range, p.x+1.f/range, p.y+1.f/range); // 1% larger to cover rounding on borders
					//printf("(%d, %d) -> (%f, %f)\n", x, y, p.x, p.y);
					display.FillRect(r, c);
					break;
				}
			}
		}
	}
}

BitMapPic *VoxelGrid::GetImage(int face)
{
	BitMapPic *b = new BitMapPic(xWidth, yWidth);
	const float range = std::max(xWidth, std::max(yWidth, zWidth));
	uint16_t minz = zWidth;
	uint16_t maxz = 0;
	for (uint16_t x = 0; x < xWidth; x++)
	{
		for (uint16_t y = 0; y < yWidth; y++)
		{
			for (uint16_t z = 0; z < zWidth; z++)
			{
				if (voxels[GetIndex(x, y, z)])
				{
					minz = std::min(minz, z);
					maxz = std::max(maxz, z);
					break;
				}
			}
		}
	}
	if (minz == maxz)
	{
		maxz++;
		minz--;
	}
	for (uint16_t x = 0; x < xWidth; x++)
	{
		for (uint16_t y = 0; y < yWidth; y++)
		{
			b->SetPixel(x, yWidth-y-1, 255, 255, 255);
			for (uint16_t z = 0; z < zWidth; z++)
			{
				if (voxels[GetIndex(x, y, z)])
				{
					rgbColor c = {0.0f, 0.0f, 0.0f};
					c.b = 1.0f-(z-minz)/(float)(maxz-minz);
					c.r = c.b/2.0f;
					c.g = c.b/2.0f;
					
//					point3d p;
//					GetGLCoordinate({x, y, z}, p);
					//Graphics::rect r(p.x-1.f/range, p.y-1.f/range, p.x+1.f/range, p.y+1.f/range); // 1% larger to cover rounding on borders
					//printf("(%d, %d) -> (%f, %f)\n", x, y, p.x, p.y);
					b->SetPixel(x, yWidth-y-1, 255*c.r, 255*c.g, 255*c.b);
					//display.FillRect(r, c);
					break;
				}
			}
		}
	}
	return b;
}
