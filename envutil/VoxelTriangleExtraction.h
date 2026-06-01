//
//  VoxelTriangleExtraction.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/19/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef VoxelTriangleExtraction_h
#define VoxelTriangleExtraction_h

#include <stdio.h>
#include <vector>
#include "VoxelGrid.h"

namespace VoxelUtils {
	
	struct vn{
		uint16_t normal[3];
		uint16_t v[3];
	};
	
	struct triangle {
		uint16_t normal[3];
		uint16_t v1[3];
		uint16_t v2[3];
		uint16_t v3[3];
		
		vn GetVN(int which)
		{
			switch (which)
			{
				case 0:
					return {normal[0], normal[1], normal[2], v1[0], v1[1], v1[2]};
				case 1:
					return {normal[0], normal[1], normal[2], v2[0], v2[1], v2[2]};
				case 2:
					return {normal[0], normal[1], normal[2], v3[0], v3[1], v3[2]};
				default:
					exit(0);
					return vn();
			}
		}
	};
	bool operator==(const triangle &a, const triangle &b);
	
	bool operator==(const vn &a, const vn &b);
	
	void GetTriangles(VoxelGrid *s, std::vector<triangle> &data);
	
}

namespace std {
	template <> struct hash<VoxelUtils::triangle>
	{
		size_t operator()(const VoxelUtils::triangle & x) const
		{
			uint64_t t =
			(((uint64_t)x.v1[0]<<0)|((uint64_t)x.v1[1]<<8)|((uint64_t)x.v1[2]<<16)|
			 ((uint64_t)x.v2[0]<<24)|((uint64_t)x.v2[1]<<32)|((uint64_t)x.v2[2]<<40)|
			 ((uint64_t)x.v3[0]<<48)|((uint64_t)x.v3[1]<<56));
			t ^= ((uint64_t)x.v3[2]<<28);
			t ^= (x.normal[0]<<7)|(x.normal[1]<<15)|(x.normal[2]<<24);
			return t;
		}
	};
	
	template <> struct hash<VoxelUtils::vn>
	{
		size_t operator()(const VoxelUtils::vn & x) const
		{
			uint64_t t =
			(((uint64_t)x.v[0]<<0)|((uint64_t)x.v[1]<<8)|((uint64_t)x.v[2]<<16)|
			 ((uint64_t)x.normal[0]<<24)|((uint64_t)x.normal[1]<<32)|((uint64_t)x.normal[2]<<40));
			return t;
		}
	};
	
}


#endif /* VoxelTriangleExtraction_h */
