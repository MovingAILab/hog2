//
//  VoxelTriangleExtraction.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/19/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "SharedQueue.h"
#include "VoxelTriangleExtraction.h"
#include <thread>

namespace VoxelUtils {
	struct Point3D {
		int x, y, z;
	};
	enum plane {
		xPositive,
		xNegative,
		yPositive,
		yNegative,
		zPositive,
		zNegative
	};
	
	struct work {
		plane p;
		int coordinate;
	};
	
	bool operator==(const triangle &a, const triangle &b)
	{
		return (a.normal[0] == b.normal[0] &&
				a.normal[1] == b.normal[1] &&
				a.normal[2] == b.normal[2] &&
				a.v1[0] == b.v1[0] &&
				a.v1[1] == b.v1[1] &&
				a.v1[2] == b.v1[2] &&
				a.v2[0] == b.v2[0] &&
				a.v2[1] == b.v2[1] &&
				a.v2[2] == b.v2[2] &&
				a.v3[0] == b.v3[0] &&
				a.v3[1] == b.v3[1] &&
				a.v3[2] == b.v3[2]);
	}
	bool operator==(const vn &a, const vn &b)
	{
		return (a.normal[0] == b.normal[0] &&
				a.normal[1] == b.normal[1] &&
				a.normal[2] == b.normal[2] &&
				a.v[0] == b.v[0] &&
				a.v[1] == b.v[1] &&
				a.v[2] == b.v[2]);
	}
	
	
	void ThreadedWorker(VoxelGrid *s, std::vector<triangle> &data,
						SharedQueue<work> &queue, std::mutex &lock);
	
	void PXHelper(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, std::vector<triangle> &data);
	void AddPXFace(VoxelGrid *s, int x, std::vector<triangle> &data);
	void NXHelper(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, std::vector<triangle> &data);
	void AddNXFace(VoxelGrid *s, int x, std::vector<triangle> &data);
	void PYHelper(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, std::vector<triangle> &data);
	void AddPYFace(VoxelGrid *s, int y, std::vector<triangle> &data);
	void NYHelper(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, std::vector<triangle> &data);
	void AddNYFace(VoxelGrid *s, int y, std::vector<triangle> &data);
	void PZHelper(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, std::vector<triangle> &data);
	void AddPZFace(VoxelGrid *s, int z, std::vector<triangle> &data);
	void NZHelper(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, std::vector<triangle> &data);
	void AddNZFace(VoxelGrid *s, int z, std::vector<triangle> &data);
	
	void GetTriangles(VoxelGrid *s, std::vector<triangle> &data)
	{
		std::mutex lock;
		SharedQueue<work> queue;
		std::vector<std::thread> threads;
		int numThreads = std::thread::hardware_concurrency();
		
		for (int x = 0; x < numThreads; x++)
		{
			threads.push_back(std::thread(ThreadedWorker, s, std::ref(data), std::ref(queue), std::ref(lock)));
		}
		
		int x, y, z;
		s->GetLimits(x, y, z);
		for (int a = 0; a < x; a++)
		{
			queue.WaitAdd({xPositive, a});
			queue.WaitAdd({xNegative, a});
		}
		for (int b = 0; b < y; b++)
		{
			queue.WaitAdd({yPositive, b});
			queue.WaitAdd({yNegative, b});
		}
		for (int c = 0; c < z; c++)
		{
			queue.WaitAdd({zPositive, c});
			queue.WaitAdd({zNegative, c});
		}
		
		for (int x = 0; x < numThreads; x++)
			queue.WaitAdd({xPositive, -1});
		for (int x = 0; x < numThreads; x++)
		{
			threads[x].join();
		}
	}
	
	triangle MakeTriangle(Point3D normal, Point3D v1, Point3D v2, Point3D v3)
	{
		triangle t1;
		t1.normal[0] = normal.x;
		t1.normal[1] = normal.y;
		t1.normal[2] = normal.z;
		t1.v1[0] = v1.x;
		t1.v1[1] = v1.y;
		t1.v1[2] = v1.z;
		t1.v2[0] = v2.x;
		t1.v2[1] = v2.y;
		t1.v2[2] = v2.z;
		t1.v3[0] = v3.x;
		t1.v3[1] = v3.y;
		t1.v3[2] = v3.z;
		return t1;
	}
	
	void ThreadedWorker(VoxelGrid *s, std::vector<triangle> &data,
						SharedQueue<work> &queue, std::mutex &lock)
	{
		work w;
		std::vector<triangle> localData;
		while (true)
		{
			queue.WaitRemove(w);
			if (w.coordinate == -1)
				break;
			
			switch (w.p)
			{
				case xPositive: AddPXFace(s, w.coordinate, localData); break;
				case xNegative: AddNXFace(s, w.coordinate, localData); break;
				case yPositive: AddPYFace(s, w.coordinate, localData); break;
				case yNegative: AddNYFace(s, w.coordinate, localData); break;
				case zPositive: AddPZFace(s, w.coordinate, localData); break;
				case zNegative: AddNZFace(s, w.coordinate, localData); break;
				default: break; //printf("Failure: unknown case\n"); break;
			}
		}
		if (localData.size() > 0)
		{
			lock.lock();
			data.insert(data.end(), localData.begin(), localData.end());
			lock.unlock();
		}
	}
	
	int XMaxSquare(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, int xOffset)
	{
		if (y >= ymax || z >= zmax)
			return 0;
		bool valid = false;
		if (s->IsBlocked(x, y, z) && !s->IsBlocked(x+xOffset, y, z))
			valid = true;
		int count = 1;
		while (true)
		{
			if (y+count >= ymax || z+count >= zmax)
				break;
			int tmp = 0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x, y+t, z+count) && !s->IsBlocked(x+xOffset, y+t, z+count))?1:0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x, y+count, z+t) && !s->IsBlocked(x+xOffset, y+count, z+t))?1:0;
			if ((valid && tmp == (count+1)*2) || (!valid && tmp == 0))
			{
				count++;
				continue;
			}
			break;
		}
		if (!valid)
			return -count;
		return count;
	}
	
	std::pair<int, int> XMaxRect(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, int xOffset)
	{
		int minLen = XMaxSquare(s, x, y, ymax, z, zmax, xOffset);
		if (minLen <= 0)
			return {minLen, minLen};
		// Try extending one or more directions of the square.
		//	printf("Got face size %d. Extending\n", minLen);
		int yCount = 1;
		while (true)
		{
			if (y+minLen+yCount >= ymax)
			{
				yCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x, y+minLen+yCount, z+t) && !s->IsBlocked(x+xOffset, y+minLen+yCount, z+t))?1:0;
			if (tmp == minLen)
			{
				yCount++;
				continue;
			}
			yCount--;
			break;
		}
		//	printf("Y-extension: %d\n", yCount);
		int zCount = 1;
		while (true)
		{
			if (z+minLen+zCount >= zmax)
			{
				zCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x, y+t, z+minLen+zCount) && !s->IsBlocked(x+xOffset, y+t, z+minLen+zCount))?1:0;
			if (tmp == minLen)
			{
				zCount++;
				continue;
			}
			zCount--;
			break;
		}
		//	printf("Z-extension: %d\n", zCount);
		//	if (zCount > 0 || yCount > 0)
		//		printf("Made bigger by %d\n", std::max(yCount, zCount));
		if (zCount > yCount)
			return {minLen, minLen+zCount};
		return {minLen+yCount, minLen};
	}
	
	
	void PXHelper(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, std::vector<triangle> &data)
	{
		//int count = XMaxSquare(s, x, y, ymax, z, zmax, 1);
		auto v = XMaxRect(s, x, y, ymax, z, zmax, 1);
		int ySize = v.first;
		int zSize = v.second;
		if (ySize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({1, 0, 0}, {x+1, y, z}, {x+1, y+ySize, z}, {x+1, y+ySize, z+zSize}));
			data.push_back(MakeTriangle({1, 0, 0}, {x+1, y, z}, {x+1, y+ySize, z+zSize}, {x+1, y, z+zSize}));
		}
		else if (ySize < 0) { // nothing to export in size s
			ySize = -ySize;
			zSize = -zSize;
		}
		else if (ySize == 0){
			// ran out of space
			return;
		}
		PXHelper(s, x, y+ySize, ymax, z, z+zSize, data);
		PXHelper(s, x, y, y+ySize, z+zSize, zmax, data);
		PXHelper(s, x, y+ySize, ymax, z+zSize, zmax, data);
	}
	
	void AddPXFace(VoxelGrid *s, int x, std::vector<triangle> &data)
	{
		int t, y, z;
		s->GetLimits(t, y, z);
		PXHelper(s, x, 0, y, 0, z, data);
	}
	
	
	void NXHelper(VoxelGrid *s, int x, int y, int ymax, int z, int zmax, std::vector<triangle> &data)
	{
		//int count = XMaxSquare(s, x, y, ymax, z, zmax, -1);
		auto v = XMaxRect(s, x, y, ymax, z, zmax, -1);
		int ySize = v.first;
		int zSize = v.second;
		if (ySize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({-1, 0, 0}, {x, y, z}, {x, y+ySize, z+zSize}, {x, y+ySize, z}));
			data.push_back(MakeTriangle({-1, 0, 0}, {x, y, z}, {x, y, z+zSize}, {x, y+ySize, z+zSize}));
		}
		else if (ySize < 0) { // nothing to export in size s
			ySize = -ySize;
			zSize = -zSize;
		}
		else if (ySize == 0){
			// ran out of space
			return;
		}
		NXHelper(s, x, y+ySize, ymax, z, z+zSize, data);
		NXHelper(s, x, y, y+ySize, z+zSize, zmax, data);
		NXHelper(s, x, y+ySize, ymax, z+zSize, zmax, data);
	}
	
	void AddNXFace(VoxelGrid *s, int x, std::vector<triangle> &data)
	{
		int t, y, z;
		s->GetLimits(t, y, z);
		NXHelper(s, x, 0, y, 0, z, data);
	}
	
	int YMaxSquare(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, int yOffset)
	{
		if (x >= xmax || z >= zmax)
			return 0;
		bool valid = false;
		if (s->IsBlocked(x, y, z) && !s->IsBlocked(x, y+yOffset, z))
			valid = true;
		int count = 1;
		while (true)
		{
			if (x+count >= xmax || z+count >= zmax)
				break;
			int tmp = 0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x+t, y, z+count) && !s->IsBlocked(x+t, y+yOffset, z+count))?1:0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x+count, y, z+t) && !s->IsBlocked(x+count, y+yOffset, z+t))?1:0;
			if ((valid && tmp == (count+1)*2) || (!valid && tmp == 0))
			{
				count++;
				continue;
			}
			break;
		}
		if (!valid)
			return -count;
		return count;
	}
	
	std::pair<int, int> YMaxRect(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, int yOffset)
	{
		int minLen = YMaxSquare(s, x, xmax, y, z, zmax, yOffset);
		if (minLen <= 0)
			return {minLen, minLen};
		// Try extending one or more directions of the square.
		//	printf("Got face size %d. Extending\n", minLen);
		int xCount = 1;
		while (true)
		{
			if (x+minLen+xCount >= xmax)
			{
				xCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x+minLen+xCount, y, z+t) && !s->IsBlocked(x+minLen+xCount, y+yOffset, z+t))?1:0;
			if (tmp == minLen)
			{
				xCount++;
				continue;
			}
			xCount--;
			break;
		}
		//	printf("X-extension: %d\n", xCount);
		int zCount = 1;
		while (true)
		{
			if (z+minLen+zCount >= zmax)
			{
				zCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x+t, y, z+minLen+zCount) && !s->IsBlocked(x+t, y+yOffset, z+minLen+zCount))?1:0;
			if (tmp == minLen)
			{
				zCount++;
				continue;
			}
			zCount--;
			break;
		}
		//	printf("Z-extension: %d\n", zCount);
		//	if (zCount > 0 || xCount > 0)
		//		printf("Made bigger by %d\n", std::max(xCount, zCount));
		if (zCount > xCount)
			return {minLen, minLen+zCount};
		return {minLen+xCount, minLen};
	}
	
	void PYHelper(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, std::vector<triangle> &data)
	{
		auto v = YMaxRect(s, x, xmax, y, z, zmax, 1);
		int xSize = v.first;
		int zSize = v.second;
		
		if (xSize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({0, 1, 0}, {x, y+1, z}, {x+xSize, y+1, z+zSize}, {x+xSize, y+1, z}));
			data.push_back(MakeTriangle({0, 1, 0}, {x, y+1, z}, {x, y+1, z+zSize}, {x+xSize, y+1, z+zSize}));
		}
		else if (xSize < 0) { // nothing to export in size s
			xSize = -xSize;
			zSize = -zSize;
		}
		else if (xSize == 0){
			// ran out of space
			return;
		}
		PYHelper(s, x+xSize, xmax, y, z, z+zSize, data);
		PYHelper(s, x, x+xSize, y, z+zSize, zmax, data);
		PYHelper(s, x+xSize, xmax, y, z+zSize, zmax, data);
	}
	
	void AddPYFace(VoxelGrid *s, int y, std::vector<triangle> &data)
	{
		int x, t, z;
		s->GetLimits(x, t, z);
		PYHelper(s, 0, x, y, 0, z, data);
	}
	
	void NYHelper(VoxelGrid *s, int x, int xmax, int y, int z, int zmax, std::vector<triangle> &data)
	{
		auto v = YMaxRect(s, x, xmax, y, z, zmax, -1);
		int xSize = v.first;
		int zSize = v.second;
		
		if (xSize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({0, -1, 0}, {x, y, z}, {x+xSize, y, z}, {x+xSize, y, z+zSize}));
			data.push_back(MakeTriangle({0, -1, 0}, {x, y, z}, {x+xSize, y, z+zSize}, {x, y, z+zSize}));
		}
		else if (xSize < 0) { // nothing to export in size s
			xSize = -xSize;
			zSize = -zSize;
		}
		else if (xSize == 0){
			// ran out of space
			return;
		}
		NYHelper(s, x+xSize, xmax, y, z, z+zSize, data);
		NYHelper(s, x, x+xSize, y, z+zSize, zmax, data);
		NYHelper(s, x+xSize, xmax, y, z+zSize, zmax, data);
	}
	
	void AddNYFace(VoxelGrid *s, int y, std::vector<triangle> &data)
	{
		int x, t, z;
		s->GetLimits(x, t, z);
		NYHelper(s, 0, x, y, 0, z, data);
	}
	
	int ZMaxSquare(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, int zOffset)
	{
		if (x >= xmax || y >= ymax)
			return 0;
		bool valid = false;
		if (s->IsBlocked(x, y, z) && !s->IsBlocked(x, y, z+zOffset))
			valid = true;
		int count = 1;
		while (true)
		{
			if (x+count >= xmax || y+count >= ymax)
				break;
			int tmp = 0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x+t, y+count, z) && !s->IsBlocked(x+t, y+count, z+zOffset))?1:0;
			for (int t = 0; t <= count; t++)
				tmp += (s->IsBlocked(x+count, y+t, z) && !s->IsBlocked(x+count, y+t, z+zOffset))?1:0;
			if ((valid && tmp == (count+1)*2) || (!valid && tmp == 0))
			{
				count++;
				continue;
			}
			break;
		}
		if (!valid)
			return -count;
		return count;
	}
	
	std::pair<int, int> ZMaxRect(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, int zOffset)
	{
		int minLen = ZMaxSquare(s, x, xmax, y, ymax, z, zOffset);
		if (minLen <= 0)
			return {minLen, minLen};
		// Try extending one or more directions of the square.
		//	printf("Got face size %d. Extending\n", minLen);
		int xCount = 1;
		while (true)
		{
			if (x+minLen+xCount >= xmax)
			{
				xCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x+minLen+xCount, y+t, z) && !s->IsBlocked(x+minLen+xCount, y+t, z+zOffset))?1:0;
			if (tmp == minLen)
			{
				xCount++;
				continue;
			}
			xCount--;
			break;
		}
		//	printf("X-extension: %d\n", xCount);
		int yCount = 1;
		while (true)
		{
			if (y+minLen+yCount >= ymax)
			{
				yCount--;
				break;
			}
			int tmp = 0;
			for (int t = 0; t < minLen; t++)
				tmp += (s->IsBlocked(x+t, y+minLen+yCount, z) && !s->IsBlocked(x+t, y+minLen+yCount, z+zOffset))?1:0;
			if (tmp == minLen)
			{
				yCount++;
				continue;
			}
			yCount--;
			break;
		}
		//	printf("Y-extension: %d\n", yCount);
		//	if (yCount > 0 || xCount > 0)
		//		printf("Made bigger by %d\n", std::max(xCount, yCount));
		if (yCount > xCount)
			return {minLen, minLen+yCount};
		return {minLen+xCount, minLen};
	}
	
	void PZHelper(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, std::vector<triangle> &data)
	{
		auto v = ZMaxRect(s, x, xmax, y, ymax, z, 1);
		int xSize = v.first;
		int ySize = v.second;
		
		if (xSize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({0, 0, 1}, {x, y, z+1}, {x+xSize, y, z+1}, {x+xSize, y+ySize, z+1}));
			data.push_back(MakeTriangle({0, 0, 1}, {x, y, z+1}, {x+xSize, y+ySize, z+1}, {x, y+ySize, z+1}));
		}
		else if (xSize < 0) { // nothing to export in size s
			xSize = -xSize;
			ySize = -ySize;
		}
		else if (xSize == 0){
			// ran out of space
			return;
		}
		PZHelper(s, x+xSize, xmax, y, y+ySize, z, data);
		PZHelper(s, x, x+xSize, y+ySize, ymax, z, data);
		PZHelper(s, x+xSize, xmax, y+ySize, ymax, z, data);
	}
	
	void AddPZFace(VoxelGrid *s, int z, std::vector<triangle> &data)
	{
		int x, y, t;
		s->GetLimits(x, y, t);
		PZHelper(s, 0, x, 0, y, z, data);
	}
	
	void NZHelper(VoxelGrid *s, int x, int xmax, int y, int ymax, int z, std::vector<triangle> &data)
	{
		auto v = ZMaxRect(s, x, xmax, y, ymax, z, -1);
		
		int xSize = v.first;
		int ySize = v.second;
		
		if (xSize > 0) // fill at size s
		{
			data.push_back(MakeTriangle({0, 0, -1}, {x, y, z}, {x+xSize, y+ySize, z}, {x+xSize, y, z}));
			data.push_back(MakeTriangle({0, 0, -1}, {x, y, z}, {x, y+ySize, z}, {x+xSize, y+ySize, z}));
		}
		else if (xSize < 0) { // nothing to export in size s
			xSize = -xSize;
			ySize = -ySize;
		}
		else if (xSize == 0){
			// ran out of space
			return;
		}
		NZHelper(s, x+xSize, xmax, y, y+ySize, z, data);
		NZHelper(s, x, x+xSize, y+ySize, ymax, z, data);
		NZHelper(s, x+xSize, xmax, y+ySize, ymax, z, data);
	}
	
	void AddNZFace(VoxelGrid *s, int z, std::vector<triangle> &data)
	{
		int x, y, t;
		s->GetLimits(x, y, t);
		NZHelper(s, 0, x, 0, y, z, data);
	}

	
}
