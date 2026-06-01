//
//  TriangulationEnvironment.cpp
//  Triangulation
//
//  Created by Nathan Sturtevant on 6/11/24.
//  Copyright © 2024 NS Software. All rights reserved.
//

#include "TriangulationEnvironment.h"
using Graphics::point;
using Graphics::triangle;

Triangulation::Triangulation()
{
	triangulated = false;
}

void Triangulation::Reset()
{
	points.clear();
	frontier.clear();
	triangulation.clear();
	next = 0;
	triangulated = false;
}

//bool Triangulation::InsideShape(point p)
//{
//	// point not in bounds
//	if (!((p.x >= lowerLeft.x && p.y >= lowerLeft.y && p.z >= lowerLeft.z) &&
//		  (p.x <= upperRight.x && p.y <= upperRight.y && p.z <= upperRight.z)))
//		return false;
//
//	for (int t = 0; t < triangulation.size(); t++)
//	{
//		point p1 = triangulation[t].t.p1;
//		point p2 = triangulation[t].t.p2;
//		point p3 = triangulation[t].t.p3;
//		point center = (p1+p2+p3)/3;
//		
//		// point is inside with one winding
//		if ((InsideX(p1, p2, p) && InsideX(p2, p3, p) && InsideX(p3, p1, p)) ||
//			// also check alternate winding
//			(InsideX(p2, p1, p) && InsideX(p3, p2, p) && InsideX(p1, p3, p)))
//		{
//			float ld = std::min({DistToLine(p1, p2, p), DistToLine(p2, p3, p), DistToLine(p3, p1, p)});
//			return (p-center).Length()*threshold > ld*(1-threshold);
//			//		return ld*ld < threshold;
//		}
//	}
//	return false;
//	
////	float f = 0;
////	for (auto p1 : points)
////	{
////		// this is really bad - fix later
////		float tmp = point::Norm(p, p1);
////		f += 1/(tmp*tmp);
////	}
////	if (f > threshold)
////		return false;
////	return true;
//}

bool Triangulation::InsideX(point p1, point p2, point test)
{
	return (p2.x - p1.x)*(test.y - p1.y) >= (p2.y - p1.y)*(test.x - p1.x);
}


float Triangulation::DistToLine(point l1, point l2, point p)
{
	point a = l1;
	point n = (l2-l1);
	n.normalise();
	point vecToLine = (p-a) - n*point::Dot((p-a), n);
	//return magnitude(vecToLine);
	return vecToLine.length();
	
	
//	point a = l1;
//	point n = (l1-l2);
//	n.Normalize();
//	float dist = ((p-a)*n).Length();
//	return dist;
}
#include "Graph.h"
bool Triangulation::Triangulate()
{
	if (triangulated)
		return true;
	
	if (triangulation.size() == 0)
	{
		std::sort(points.begin(), points.end(), [](const point& lhs, const point& rhs){
			if (lhs.x < rhs.x)
				return true;
			if (lhs.x==rhs.x)
				return (lhs.y<rhs.y);
			return false;
		});
		
		triangulation.push_back({{points[0], points[1], points[2]}, {0, -1, -1}});
		frontier.push_back(points[0]);
		frontier.push_back(points[1]);
		frontier.push_back(points[2]);
		std::sort(frontier.begin(), frontier.end(), [](const point& lhs, const point& rhs){
			if (lhs.y < rhs.y)
				return true;
			return false;
		});

		std::cout << triangulation.back().t << "\n";
		next = 3;
		return false;
	}
//	return false;
	
	// For now, correct but inefficient
	//for (int p = 3; p < points.size(); p++)
	int p = next;
	next++;
	if (p < points.size())
	{
		bool first = true;
		// find edge from existing triangle convex hull
		for (int t = 0; t < frontier.size()-1; t++)
		{
			bool p1 = CanConnect(points[p], frontier[t]);
			bool p2 = CanConnect(points[p], frontier[t+1]);
			//bool p3 = CanConnect(points[p], triangulation[t].t.p3);
			if (p1 && p2)// && triangulation[t].neighbor[0] == -1)
			{
//				triangulation[t].neighbor[0] = (int)triangulation.size();
				triangulation.push_back({{points[p], frontier[t], frontier[t+1]}, {-1, t, -1}});
				std::cout << triangulation.back().t << "\n";
				
				if (first)
				{
					frontier.insert(frontier.begin()+t+1, points[p]);
					first = false;
					t++;
				}
				else {
					frontier.erase(frontier.begin()+t);
					t--;
					//					frontier[t] = points[p];
				}
				//				break;
				//				break;
			}
		}
		return false;
	}
	triangulated = true;
	return true;
}

bool Triangulation::CanConnect(point from, point target)
{
	for (int t = 0; t < triangulation.size(); t++)
	{
		if (Intersect(from, target, triangulation[t].t.p1, triangulation[t].t.p2))
			return false;
		if (Intersect(from, target, triangulation[t].t.p1, triangulation[t].t.p3))
			return false;
		if (Intersect(from, target, triangulation[t].t.p2, triangulation[t].t.p3))
			return false;
	}
	return true;
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the floats i_x and i_y.
bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
	float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

	float s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		if (i_x != NULL)
			*i_x = p0_x + (t * s1_x);
		if (i_y != NULL)
			*i_y = p0_y + (t * s1_y);
		return true;
	}

	return false; // No collision
}

bool Triangulation::Intersect(point l1a, point l1b, point l2a, point l2b)
{
	float x, y;
	bool result = get_line_intersection(l1a.x, l1a.y, l1b.x, l1b.y,
										l2a.x, l2a.y, l2b.x, l2b.y, &x, &y);
	if (fequal(x, l1a.x) && fequal(y, l1a.y))
		return false;
	if (fequal(x, l1b.x) && fequal(y, l1b.y))
		return false;
	if (fequal(x, l2a.x) && fequal(y, l2a.y))
		return false;
	if (fequal(x, l2b.x) && fequal(y, l2b.y))
		return false;
	return result;
}

void Triangulation::Draw(Graphics::Display &display) const
{
	for (const auto &t : triangulation)
		display.FrameTriangle(t.t, 0.02f, Colors::lightgray);
	
	for (const auto &p : points)
		display.FillCircle(p, 0.02f, Colors::white);

	for (int x = 0; x+1 < frontier.size(); x++)
		display.DrawLine(frontier[x], frontier[x+1], 0.01f, (x%2)?Colors::lightred:Colors::darkred);
}
