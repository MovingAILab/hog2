/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include "Graphics.h"
#include <string>
#include <random>
#include <cassert>
#include <algorithm>
#include "TriangulationEnvironment.h"

void KMeans(std::vector<Graphics::point> &points, int K, int iterations, std::vector<Graphics::point> &output);
std::vector<Graphics::point> points, means;
Triangulation t;

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<float> rand_hog(-0.8, 0.8);

// -------------- MAIN FUNCTION ----------- //
int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 800, 800);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "KMeans", "Re-run k-means", kAnyModifier, 'k');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Resest k-means", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Triangulate", "Init triangulation", kAnyModifier, 't');
//	InstallKeyboardHandler(MyDisplayHandler, "Step", "Step animation", kAnyModifier, 'o');
//	InstallKeyboardHandler(MyDisplayHandler, "Pause", "Pause animation", kAnyModifier, 'p');
//	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slower animation", kAnyModifier, '[');
//	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Faster animation", kAnyModifier, ']');
//	InstallKeyboardHandler(MyDisplayHandler, "Debug", "Toggle Debug", kAnyModifier, '|');

	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseDrag|kMouseDown|kMouseUp));
}


void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);

		for (int x = 0; x < 120; x++)
		{
			float p1 = rand_hog(mt);
			float p2 = rand_hog(mt);
			//db->AddPoint({p1, p2, 0});
			points.push_back({p1, p2, 0.12f});
		}
		KMeans(points, 10, 20, means);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::black);
	for (auto p : means)
		display.FillCircle(p, 0.025f, Colors::blue);
	for (auto p : points)
		display.FillCircle(p, 0.0125f, Colors::lightblue);
	t.Draw(display);
	setTextBufferVisibility(false);
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key) // handles keypresses that change display
{
	static bool init = false;
	switch (key)
	{
		case 'r':
			init = false;
			t.Reset();
			points.clear();
			means.clear();
			for (int x = 0; x < 120; x++)
			{
				float p1 = rand_hog(mt);
				float p2 = rand_hog(mt);
				//db->AddPoint({p1, p2, 0});
				points.push_back({p1, p2, 0.12f});
			}
		case 'k':
			KMeans(points, 10, 1, means);
			break;
		case 't':
			if (!init)
			{
				for (auto p : means)
					t.AddPoint(p);
				t.AddPoint({-1, -1, 0});
				t.AddPoint({1, -1, 0});
				t.AddPoint({-1, 1, 0});
				t.AddPoint({1, 1, 0});
				init = true;
			}
			t.Triangulate();
			break;
		default: break;
	}
}

/*
 * Code runs when user clicks or moves mouse in window
 *
 * Application does not currently need mouse support
 */
bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
//	Graphics::point p = WorldFromHOG(loc);
	
	switch (mType)
	{
		case kMouseDrag:
		{
			if (button == kLeftButton)
			{
			}
		}
			break;
		case kMouseDown:
		{
			if (button == kRightButton)
			{
			}
			else if (button == kLeftButton)
			{
			}
		}
			break;
		default:
		{
		}
			break;
	}
	return true;
}

// returns k centroids of the points selected
void KMeans(std::vector<Graphics::point> &points, int K, int iterations, std::vector<Graphics::point> &output)
{
	std::vector<int> assignment;
	// initialize
	if (output.size() == 0)
	{
		for (int x = 0; x < K; x++)
		{
			output.push_back(points[x]);
		}
	}
	assignment.resize(points.size());

	// iterations - doesn't need to be K
	for (int x = 0; x < iterations; x++)
	{
		// from each point
		for (int y = 0; y < points.size(); y++)
		{
			int best = 0;
			float bestDist = __FLT_MAX__;
			// find closest centroid
			for (int z = 0; z < output.size(); z++)
			{
				float d = (points[y]-output[z]).length();
				if (d < bestDist)
				{
					best = z;
					bestDist = d;
				}
			}
			assignment[y] = best;
		}
		// Compute new centroids
		for (int z = 0; z < K; z++)
		{
			Graphics::point tmp(0,0,0);
			int cnt = 0;
			for (int y = 0; y < points.size(); y++)
			{
				if (assignment[y] == z)
				{
					tmp += points[y];
					cnt++;
				}
			}
			output[z] = tmp/cnt;
		}
	}
//	for (auto i : output)
//		std::cout << i << "\n";
}
