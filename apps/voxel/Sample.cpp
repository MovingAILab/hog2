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
#include "Sample.h"
#include "ScenarioLoader.h"
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "Voxels.h"
#include <iostream>
#include "Timer.h"
#include "TemplateAStar.h"
#include "VoxelGrid.h"
//#include "GLUThog.h"
#include "SVGUtil.h"
#include <unordered_map>
#include "NBS.h"
#include <deque>

Graphics::point velocity;
std::string map;
void Maze3D(int d);
void CustomMaze();
void BuildBenchmarks(const char *map);
void SolveBenchmarks(const char *map, const char *bench);
void ExtractComponents(const char *map);

//struct voxelWorld {
//	uint32_t header;
//	float voxelSize;
//	uint64_t numVoxelsGrids;
//	float minbounds[4];
//	float maxbounds[4];
//	uint64_t *morton;
//	uint64_t *grid;
//};
//
//voxelWorld LoadData();
//void Draw(voxelWorld w);
bool mouseTracking = false;
bool recording = false;
bool capture = false;
int frames = 60;

//recVec cameraOffset(0,0,0);
//Voxels *v = 0;
VoxelGrid *vg = 0;
//voxelWorld theWorld;
std::vector<voxelGridState> path;

int main(int argc, char* argv[])
{
//	theWorld = LoadData();
//	CustomMaze();
	//	Maze3D(11);
	InstallHandlers();
	RunHOGGUI(argc, argv, 512, 512);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	vg = new VoxelGrid("/Users/nathanst/hog2/maps/warframe/Complex.3dmap");
	vg->SetUseEfficientDraw(true);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
//	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Cam", "Reset camera", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Begin/end movie recording", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Fly camera forward", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Fly camera back", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Slide camera left", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Slide camera right", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Fly", "Stop moving", kAnyModifier, ' ');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find random path", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Efficient", "Switch to efficient drawing", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Inside", "Fill outside and invert to find inside", kAnyModifier, 'i');
	InstallWindowHandler(MyWindowHandler);
	InstallCommandLineHandler(MyCLHandler, "-convert", "-convert <source> <dest>", "Convert a map from the DE format to the repo format");
	InstallCommandLineHandler(MyCLHandler, "-map", "-map <source>", "Use the provided map");
	InstallCommandLineHandler(MyCLHandler, "-capture", "-capture", "Capture animation");
	InstallCommandLineHandler(MyCLHandler, "-hcost", "-hcost x1 y1 z1 x2 y2 z2", "Compute heuristic between points");
	InstallCommandLineHandler(MyCLHandler, "-benchmark", "-benchmark <map>", "Build benchmark problems for map");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve <map> <benchmark>", "Build benchmark problems for map");

	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg", "capture model as svg (filename(s) will come from -map argument)");
	InstallCommandLineHandler(MyCLHandler, "-bmp", "-bmp", "capture model as bmp (filename(s) will come from -map argument)");
	InstallCommandLineHandler(MyCLHandler, "-extractComponents", "-extractComponents <map>", "extract connected components in map");
//
//	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		
		mouseTracking = false;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
		SetNumPorts(windowID, 1);
		//SetLighting(0.65, 1.0, 0.2);
	}
}

double distance = 12.5;

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
	case '|': break; //resetCamera(); break;
		case 'r': recording = !recording; break;
		case ' ':
		{
			velocity *= 0;
			break;
		}
		case 'a':
		{
			break;
		}
		case 'd':
		{
			break;
		}
		case 'w':
		{
			break;
		}			
		case 's':
		{
			break;
		}
		case 'e':
		{
			//vg->efficient = !vg->efficient;
			break;
		}
		case 'i':
		{
			printf("Inverting world\n");
			vg->Fill({0,0,0});
			vg->Invert();
			break;
		}
		case 'p':
			while (true)
			{
				// TODO: Make sure we don't generate the same child twice
				Timer t;
				voxelGridState s = vg->GetRandomState();
				voxelGridState g = vg->GetRandomState();
//(233, 391, 362) to (702, 170, 62)
				//				s = {400, 395, 150};
//				g = {291, 278, 238};
				std::cout << "-->Searching from " << s << " to " << g << ". Base h = " << vg->HCost(s, g) << " \n";
				static TemplateAStar<voxelGridState, voxelGridAction, VoxelGrid> astar;
				t.StartTimer();
				astar.GetPath(vg, s, g, path);
				t.EndTimer();
				printf("%llu nodes expanded\n", astar.GetNodesExpanded());
				printf("%llu nodes generated\n", astar.GetNodesTouched());
				printf("%1.2fms elapsed\n", t.GetElapsedTime()*1000.0);
				printf("Path length: %1.2f. Heuristic: %1.2f\n", vg->GetPathLength(path), vg->HCost(s, g));
				printf("---Done!---\n");
				if (!fequal(vg->GetPathLength(path), vg->HCost(s, g)))
				break;
			}
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-convert") == 0 && maxNumArgs >= 3)
	{
		Voxels v(argument[1]);
		v.Export(argument[2]);
		exit(0);
	}
	if (strcmp(argument[0], "-capture") == 0)
	{
		capture = true;
		if (maxNumArgs > 1)
		{
			if (argument[1][0] != '-')
			{
				frames = atoi(argument[1]);
				return 2;
			}
		}
		return 1;
	}
	if (strcmp(argument[0], "-map") == 0 && maxNumArgs >= 2)
	{
		map = argument[1];
		return 2;
	}
	if (strcmp(argument[0], "-hcost") == 0 && maxNumArgs >= 7)
	{
		VoxelGrid v(100, 100, 100);
		voxelGridState v1(atoi(argument[1]), atoi(argument[2]), atoi(argument[3]));
		voxelGridState v2(atoi(argument[4]), atoi(argument[5]), atoi(argument[6]));
		double c = v.HCost(v1, v2);
		std::cout << "h(" << v1 << ", " << v2 << ") = ";
		printf("%1.8f\n", c);
		exit(0);
	}
	if (strcmp(argument[0], "-benchmark") == 0 && maxNumArgs >= 2)
	{
		BuildBenchmarks(argument[1]);
		return 2;
	}
	if (strcmp(argument[0], "-solve") == 0 && maxNumArgs >= 3)
	{
		SolveBenchmarks(argument[1], argument[2]);
		return 3;
	}
	if (strcmp(argument[0], "-extractComponents") == 0 && maxNumArgs >= 2)
	{
		ExtractComponents(argument[1]);
		return 2;
	}
	if (strcmp(argument[0], "-svg") == 0 && maxNumArgs >= 1)
	{
		printf("Loading '%s'\n", map.c_str());
		VoxelGrid v(map.c_str());
		printf("Exporting to graphics\n");
		Graphics::Display d;
		v.Draw(d);
		std::string output = map;
		output+=".svg";
		printf("Exporting to %s\n", output.c_str());
		int x, y, z;
		v.GetLimits(x, y, z);
		MakeSVG(d, output.c_str(), std::max(x, y), std::max(x, y));
		exit(0);
		return 1;
	}
	if (strcmp(argument[0], "-bmp") == 0 && maxNumArgs >= 1)
	{
		printf("Loading '%s'\n", map.c_str());
		VoxelGrid v(map.c_str());
		BitMapPic *bmp = v.GetImage(0);
		std::string output = map;
		output+=".bmp";
		bmp->Save(output.c_str());
		exit(0);
		return 1;
	}

	return 0;
}
Timer frameRate;
std::string te;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (capture)
	{
		static int currentFrame = 0;
		//glRotatef(currentFrame*360.0/frames, 0, 1, 0);
		//vg->OpenGLDraw();

		char fname[255];
		sprintf(fname, "%s-%d%d%d%d",
				map.c_str(), (currentFrame/1000)%10, (currentFrame/100)%10, (currentFrame/10)%10, currentFrame%10);
		//SaveScreenshot(windowID, fname);
		currentFrame++;
		if (currentFrame%frames == 0)
			exit(0);

		return;
	}
	frameRate.EndTimer();
	te = std::to_string(1.0/frameRate.GetElapsedTime());
	submitTextToBuffer(te.c_str());
	frameRate.StartTimer();

	
	if (viewport == 0)
	{
	  //cameraOffset(velocity.x, velocity.y, velocity.z);
		
	  //		v->OpenGLDraw();
	}
//	if (viewport == 1)
	{
	  //	vg->OpenGLDraw();

		if (path.size() > 0)
		{
		  //vg->SetColor(1.0, 1.0, 1.0);
		  //	glColor3f(1, 1, 1);
		  //	glLineWidth(8.0);
		  //	for (int x = 0; x < path.size()-1; x++)
		  //		vg->GLDrawLine(path[x], path[x+1]);
		  //	glLineWidth(1.0);
		}
	}
	//LoadData();
	//Draw(theWorld);

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		//SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
}


bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
}

void SolveBenchmarks(const char *map, const char *bench)
{
	printf("Solving benchmarks for '%s'\n", map);
	VoxelGrid v(map);
	FILE *f = fopen(bench, "r");
	if (f == 0)
	{
		printf("Aborting: cannot open file '%s'\n", bench);
		exit(0);
	}
	TemplateAStar<voxelGridState, voxelGridAction, VoxelGrid> astar;
	NBS<voxelGridState, voxelGridAction, VoxelGrid> nbs;
	voxelGridState s, g;
	char mapname[255];
	fscanf(f, "version 1\n%s\n", mapname);
	Timer t;
//	t.StartTimer();
	int cnt = 0;
	while (f != 0)
	{
		cnt++;
		if (cnt == 1000) break;
		double t1, t2;
		int cnt = fscanf(f, "%hd %hd %hd %hd %hd %hd %lf %lf\n", &s.x, &s.y, &s.z, &g.x, &g.y, &g.z, &t1, &t2);
		if (cnt != 8) exit(0);
		t.StartTimer();
		astar.GetPath(&v, s, g, path);
		t.EndTimer();
		printf("Forward A* Path length %1.8lf [vs %1.8lf] %1.2fs %llu exp %llu gen\n", v.GetPathLength(path), t1, t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched());
		t.StartTimer();
		astar.GetPath(&v, g, s, path);
		t.EndTimer();
		printf("Backward A* Path length %1.8lf [vs %1.8lf] %1.2fs %llu exp %llu gen\n", v.GetPathLength(path), t1, t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched());
		t.StartTimer();
		nbs.GetPath(&v, s, g, &v, &v, path);
		t.EndTimer();
//		for (auto i : path)
//			std::cout << i << " ";
		printf("NBS Path length %1.8lf [vs %1.8lf] %1.2fs %llu exp %llu gen\n\n", v.GetPathLength(path), t1, t.GetElapsedTime(), nbs.GetNodesExpanded(), nbs.GetNodesTouched());
	}
//	t.EndTimer();
	//printf("NBS:%1.2f ", t.GetElapsedTime());
	//printf("A:%1.2f ", t.GetElapsedTime());
	fclose(f);
	exit(0);
}

void BuildBenchmarks(const char *map)
{
	srandom(time(0));
	std::unordered_map<voxelGridState, bool> locs;
	VoxelGrid v(map);
	// 1. Get unique locations nearby set voxels
	int xmax, ymax, zmax;
	int range = 5;
	v.GetLimits(xmax, ymax, zmax);
	
	for (uint16_t x = 0; x < xmax; x++)
	{
		for (uint16_t y = 0; y < ymax; y++)
		{
			for (uint16_t z = 0; z < zmax; z++)
			{
				if (v.IsBlocked(x, y, z))
				{
					for (uint16_t x1 = x-range; x1 <= x+range; x1++)
					{
						for (uint16_t y1 = y-range; y1 <= y+range; y1++)
						{
							for (uint16_t z1 = z-range; z1 <= z+range; z1++)
							{
								if (!v.IsBlocked(x1, y1, z1))
									locs[{x1, y1, z1}] = true;
							}
						}
					}
				}
			}
		}
	}
	// 2. put all locations into a vector
	std::vector<voxelGridState> values;
	for (auto i = locs.begin(); i != locs.end(); i++)
	{
		values.push_back(i->first);
	}

	TemplateAStar<voxelGridState, voxelGridAction, VoxelGrid> astar;
	NBS<voxelGridState, voxelGridAction, VoxelGrid> nbs;
	voxelGridState s, g;
	// 3. generate problems
	for (int x = 0; x < 10000; x++)
	{
		do {
			s = values[random()%values.size()];
			g = values[random()%values.size()];
			nbs.GetPath(&v, s, g, &v, &v, path);
//			astar.GetPath(&v, s, g, path);
		} while (path.size() == 0 || fequal(v.GetPathLength(path), v.HCost(s, g)));
		
		std::cout << s << "\t" << g << "\t";
		printf("%1.8f\t%1.3f\n", v.GetPathLength(path), v.GetPathLength(path)/v.HCost(s, g));
	}
	
	exit(0);
}

void BFS(VoxelGrid &v, const voxelGridState &s, std::unordered_map<voxelGridState, bool> &l)
{
	std::deque<voxelGridState> q;
	q.push_back(s);
	while (q.size() > 0)
	{
		voxelGridState next = q.front();
		q.pop_front();
		
		if (!v.Legal(next) || !v.IsBlocked(next))
			continue;
		
		v.SetBlocked(next, false);
		l[next] = true;
		
		if (v.Legal(voxelGridState(next.x+1, next.y, next.z)))
			q.push_back(voxelGridState(next.x+1, next.y, next.z));
		if (v.Legal(voxelGridState(next.x-1, next.y, next.z)))
			q.push_back(voxelGridState(next.x-1, next.y, next.z));

		if (v.Legal(voxelGridState(next.x, next.y+1, next.z)))
			q.push_back(voxelGridState(next.x, next.y+1, next.z));
		if (v.Legal(voxelGridState(next.x, next.y-1, next.z)))
			q.push_back(voxelGridState(next.x, next.y-1, next.z));

		if (v.Legal(voxelGridState(next.x, next.y, next.z+1)))
			q.push_back(voxelGridState(next.x, next.y, next.z+1));
		if (v.Legal(voxelGridState(next.x, next.y, next.z-1)))
			q.push_back(voxelGridState(next.x, next.y, next.z-1));
	}
}

void ExtractComponents(const char *map)
{
	std::unordered_map<voxelGridState, bool> locs;
	VoxelGrid v(map);
	// 1. Get unique locations nearby set voxels
	int xmax, ymax, zmax;
	int range = 4;
	v.GetLimits(xmax, ymax, zmax);

	int count = 0;
	
	for (uint16_t x = 0; x < xmax; x++)
	{
		for (uint16_t y = 0; y < ymax; y++)
		{
			for (uint16_t z = 0; z < zmax; z++)
			{
				if (v.IsBlocked(x, y, z))
				{
					BFS(v, voxelGridState(x, y, z), locs);
					printf("Component %d has %d items\n", count, locs.size());
					VoxelGrid tmp(xmax, ymax, zmax);
					VoxelGrid tmp2(xmax, ymax, zmax);
					for (auto i = locs.begin(); i != locs.end(); i++)
					{
						tmp.SetBlocked(i->first, true);
						tmp2.SetBlocked(i->first, true);
					}
					tmp2.Fill(voxelGridState(0,0,0));
					tmp2.Invert();
					tmp.Or(&tmp2);
					std::string name = map;
					name.substr(0, name.find_last_of("."));
					name += "-component"+std::to_string(count);
					name += ".3dmap";
					tmp.SaveInMinBB(name.c_str());
					count++;
					locs.clear();
				}
			}
		}
	}
}



#include "Map.h"
#include "MapGenerators.h"
#include "Graph.h"
#include "Prim.h"
#include "Map2DEnvironment.h"
#include "SVGUtil.h"
#include <time.h>

bool MakeMazeMST(int d);
void MazeMazeTree(int d);
void ExportMazes(Map *m1, Map *m2, Map *m3, int mapSize);

void CustomMaze()
{
	Map *m1 = new Map("/Users/nathanst/m01.map");
	Map *m2 = new Map("/Users/nathanst/m02.map");
	Map *m3 = new Map("/Users/nathanst/m03.map");
	ExportMazes(m1, m2, m3, m1->GetMapWidth());
	exit(0);
}

void Maze3D(int d)
{
	srandom(time(0));
	int cnt = 0;
	while (true)
	{
		printf("Attept %d\n", ++cnt);
		bool result = MakeMazeMST(d);
		if (result)
			break;
	}
	MazeMazeTree(d);
	exit(0);
}

edge *GetEdge(Graph *g, int x1, int y1, int z1, int x2, int y2, int z2);

bool MakeMazeMST(int mazeSize)
{
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	std::vector<xyLoc> path;
	Graph *g;
	g = new Graph();
	int mapSize = mazeSize*2+1;
//	int s = d/2;
//	d = s*2+1;
	for (int z = 0; z < 2; z++)
	{
		for (int x = 0; x < mazeSize; x++)
		{
			for (int y = 0; y < mazeSize; y++)
			{
				// skip top area for logo (no nodes at all)
				if (z == 0 && (x > mazeSize/4 && x < 3*mazeSize/4) && (y > mazeSize/4 && y < 3*mazeSize/4))
				{
					continue;
				}
				node *n;
				n = new node("");
				n->SetLabelL(0, x);
				n->SetLabelL(1, y);
				n->SetLabelL(2, z);
				g->AddNode(n);
			}
		}
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n1 = g->GetNode(x);
		for (int y = 0; y < g->GetNumNodes(); y++)
		{
			node *n2 = g->GetNode(y);
			// Node 1 id must be < node 2 id
			if (x == y || n1->GetNum() >= n2->GetNum())
				continue;

//			printf("Considering edge (%ld, %ld, %ld) to (%ld, %ld, %ld)\n",
//				   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//				   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));

			int sum = 0;
			sum += std::abs(n1->GetLabelL(0) - n2->GetLabelL(0));
			sum += std::abs(n1->GetLabelL(1) - n2->GetLabelL(1));
			sum += std::abs(n1->GetLabelL(2) - n2->GetLabelL(2));

			// only 1 coordinate changed
			if (sum == 1)
			{
//				printf("Adding edge (%ld, %ld, %ld) to (%ld, %ld, %ld)\n",
//					   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//					   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));
				edge *e;
				if (n1->GetLabelL(2) + n2->GetLabelL(2) == 1) // through bottom
					e = new edge(n1->GetNum(), n2->GetNum(), 100000);
				else
					e = new edge(n1->GetNum(), n2->GetNum(), 100+random()%10);
				g->AddEdge(e);
			}
		}
	}

	
	// Reweight edges we don't want in the MST
	
	// New idea (to implement here)
			// 0. Board is split into
			//       Top    saaxxx  Bottom  bbbbbb
			//              ah  hx          bh  hb
			//              ha  xh          hc  ch
			//              aaaxxg          cccccc
			// 1. Limited holes from top to bottom. Fixed at:
			//    Corners (start/goal) [should goal be top right?]
			//    [[OTHER TBA]]
			// 2. Find MST as normal
			//
	{
		// Start gets no horizontal connections on bottom
		GetEdge(g,
				0, 0, 1,
				1, 0, 1)->setWeight(100000);
		GetEdge(g,
				0, 0, 1,
				0, 1, 1)->setWeight(100000);
		// Goal gets no horizontal connections on bottom
		GetEdge(g,
				mazeSize-1, mazeSize-1, 1,
				mazeSize-2, mazeSize-1, 1)->setWeight(100000);
		GetEdge(g,
				mazeSize-1, mazeSize-1, 1,
				mazeSize-1, mazeSize-2, 1)->setWeight(100000);

		// No down move from start
		GetEdge(g,
				0, 0, 0,
				0, 1, 0)->setWeight(100000);
		// Start is connected to top
		GetEdge(g,
				0, 0, 1,
				0, 0, 0)->setWeight(0);
		// Goal is connected to top
		GetEdge(g,
				0, 0, 1,
				0, 0, 0)->setWeight(0);
		GetEdge(g,
				mazeSize-1, mazeSize-1, 1,
				mazeSize-1, mazeSize-1, 0)->setWeight(0);
		// No left move from goal
		GetEdge(g,
				mazeSize-1, mazeSize-1, 0,
				mazeSize-2, mazeSize-1, 0)->setWeight(100000);
	}

	// Set pass-through points
	{
		// top middle-right
		GetEdge(g,
				3*mazeSize/4-1, 0, 0,
				3*mazeSize/4-1, 0, 1)->setWeight(0);
		// control edges
		GetEdge(g,
				3*mazeSize/4-1, 0, 1,
				3*mazeSize/4, 0, 1)->setWeight(100000);
		GetEdge(g,
				3*mazeSize/4-1, 0, 1,
				3*mazeSize/4-1, 1, 1)->setWeight(100000);
		// left middle-bottom
		GetEdge(g,
				0, 3*mazeSize/4-1, 0,
				0, 3*mazeSize/4-1, 1)->setWeight(0);
		// control edges
		GetEdge(g,
				0, 3*mazeSize/4-1, 1,
				0, 3*mazeSize/4, 1)->setWeight(100000);
		GetEdge(g,
				0, 3*mazeSize/4-1, 1,
				1, 3*mazeSize/4-1, 1)->setWeight(100000);

		// corners
		GetEdge(g,
				mazeSize-1, 0, 0,
				mazeSize-1, 0, 1)->setWeight(0);
		GetEdge(g,
				0, mazeSize-1, 0,
				0, mazeSize-1, 1)->setWeight(0);

		// bottom middle-right
		GetEdge(g,
				3*mazeSize/4-1, mazeSize-1, 0,
				3*mazeSize/4-1, mazeSize-1, 1)->setWeight(0);
		GetEdge(g,
				3*mazeSize/4-1, mazeSize-2, 1,
				3*mazeSize/4-1, mazeSize-1, 1)->setWeight(100000);
		GetEdge(g,
				3*mazeSize/4,   mazeSize-1, 1,
				3*mazeSize/4-1, mazeSize-1, 1)->setWeight(100000);
		// rightmiddle-bottom
		GetEdge(g,
				mazeSize-1, 3*mazeSize/4-1, 0,
				mazeSize-1, 3*mazeSize/4-1, 1)->setWeight(0);
		GetEdge(g,
				mazeSize-1, 3*mazeSize/4-1, 1,
				mazeSize-2, 3*mazeSize/4-1, 1)->setWeight(100000);
		GetEdge(g,
				mazeSize-1, 3*mazeSize/4-1, 1,
				mazeSize-1, 3*mazeSize/4, 1)->setWeight(100000);


		// final edge
		GetEdge(g,
				3*mazeSize/4, 3*mazeSize/4, 0,
				3*mazeSize/4, 3*mazeSize/4, 1)->setWeight(0);
		GetEdge(g,
				3*mazeSize/4-1, 3*mazeSize/4, 1,
				3*mazeSize/4, 3*mazeSize/4, 1)->setWeight(100000);
		GetEdge(g,
				3*mazeSize/4, 3*mazeSize/4+1, 1,
				3*mazeSize/4, 3*mazeSize/4, 1)->setWeight(100000);
		GetEdge(g,
				3*mazeSize/4, 3*mazeSize/4-1, 1,
				3*mazeSize/4, 3*mazeSize/4, 1)->setWeight(100000);

	}
	
	// Remove edges across middle
	if (1)
	{
		// split from middle across to right
		for (int x = 0; x < mazeSize/2; x++)
		{
			edge *e = GetEdge(g,
							  x, 3*mazeSize/4-1, 0,
							  x, 3*mazeSize/4, 0);
			if (e) e->setWeight(100000);
		}
		
		// split from middle across to right
		for (int y = 0; y < mazeSize/2; y++)
		{
			edge *e = GetEdge(g,
							  3*mazeSize/4, y, 0,
							  3*mazeSize/4-1, y, 0);
			if (e) e->setWeight(100000);
		}

		if (0)
		for (int t = mazeSize/2; t < mazeSize; t++)
		{
			edge *e;
			e = GetEdge(g,
						mazeSize/2, t, 1,
						mazeSize/2+1, t, 1);
			if (e) e->setWeight(100000);
//			else { printf("No edge (%d, %d, %d) (%d, %d, %d)\n", s/2, t, 1, s/2+1, t, 1); }
			e = GetEdge(g,
						t, mazeSize/2, 1,
						t, mazeSize/2+1, 1);
			if (e) e->setWeight(100000);
//			else { printf("No edge (%d %d %d) (%d %d %d)\n", t, s/2, 1, t, s/2+1, 1); }
			
		}
	}
	
	Map *m1, *m2, *m3;
	m1 = new Map(mapSize, mapSize);
	m2 = new Map(mapSize, mapSize);
	m3 = new Map(mapSize, mapSize);
	for (int x = 0; x < mapSize; x++)
		for (int y = 0; y < mapSize; y++)
		{
			m1->SetTerrainType(x, y, kOutOfBounds);
			m2->SetTerrainType(x, y, kOutOfBounds);
			m3->SetTerrainType(x, y, kOutOfBounds);
		}

	Prim(g);
	const int edgeWidth = 2;
	const int edgeStart = 1;
	const int edgeEnd = 1;
	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		if (e->getMarked() == true)
		{
			node *n1 = g->GetNode(e->getFrom());
			node *n2 = g->GetNode(e->getTo());
			long x1, x2, y1, y2, z1, z2;
			x1 = n1->GetLabelL(0);
			x2 = n2->GetLabelL(0);
			y1 = n1->GetLabelL(1);
			y2 = n2->GetLabelL(1);
			z1 = n1->GetLabelL(2);
			z2 = n2->GetLabelL(2);
			if (z1 == z2 && z1 == 0)
			{
				m1->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x2*edgeWidth+edgeEnd, y2*edgeWidth+edgeEnd, 0, kGround);
			}
			else if (z1 == z2 && z1 == 1)
			{
				m3->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x2*edgeWidth+edgeEnd, y2*edgeWidth+edgeEnd, 0, kGround);
			}
			else if (z1 != z2)
			{
				m2->SetRectHeight(x1*edgeWidth+edgeStart, y1*edgeWidth+edgeStart, x1*edgeWidth+edgeEnd, y1*edgeWidth+edgeEnd, 0, kGround);
			}
		}
	}
	
	xyLoc start, goal;

	if (0)
	{
		MapEnvironment me1(m1);
		MapEnvironment me2(m2);
		MapEnvironment me3(m3);

		// 1. Can get from start to bottom entrances
		start = {1, 1};
		goal = {1, 15};
		astar.GetPath(&me1, start, goal, path);
		if (path.size() == 0)
			return false;
		start = {1, 1};
		goal = {15, 1};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
		astar.GetPath(&me1, start, goal, path);
		if (path.size() == 0)
			return false;
		
		// 2. On bottom can get to secondary points
		start = {15, 1};
		goal = {21, 1};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
		if (m3->GetTerrainType(21, 1) != kGround)
			return false;
		astar.GetPath(&me3, start, goal, path);
		if (path.size() == 0)
			return false;
		start = {1, 15};
		goal = {1, 21};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
		if (m3->GetTerrainType(1, 21) != kGround)
			return false;
		astar.GetPath(&me3, start, goal, path);
		if (path.size() == 0)
			return false;
		
		// 3. On top can get to goal
		start = {21, 1};
		goal = {21, 15};
		if (m1->GetTerrainType(21, 15) != kGround)
			return false;
		astar.GetPath(&me1, start, goal, path);
		if (path.size() == 0)
			return false;
		start = {1, 21};
		goal = {15, 21};//{static_cast<uint16_t>(1), static_cast<uint16_t>((3*s/4+1)*2+1)};
		if (m1->GetTerrainType(15, 21) != kGround)
			return false;
		astar.GetPath(&me1, start, goal, path);
		if (path.size() == 0)
			return false;
		
		if (m3->GetTerrainType(15, 21) != kGround)
			return false;
		if (m3->GetTerrainType(21, 15) != kGround)
			return false;
	}
	
	m1->Print();
	printf("\n\n");
	m2->Print();
	printf("\n\n");
	m3->Print();

	ExportMazes(m1, m2, m3, mapSize);

	return true;
}


void ExportMazes(Map *m1, Map *m2, Map *m3, int mapSize)
{
	MapEnvironment me1(m1);
	MapEnvironment me2(m2);
	MapEnvironment me3(m3);

	// Flip m3 so it burns on the same side
	for (int y = 0; y < m3->GetMapHeight(); y++)
		for (int x = 0; x < m3->GetMapWidth()/2; x++)
		{
			tTerrain tmp = (tTerrain)m3->GetTerrainType(x, y);
			m3->SetTerrainType(x, y, (tTerrain)m3->GetTerrainType(m3->GetMapWidth()-1-x, y));
			m3->SetTerrainType(m3->GetMapWidth()-1-x, y, tmp);
		}
	
	Graphics::Display disp;
	disp.StartFrame();
	me1.Draw(disp);
	disp.DrawText("SoCS", {0, -0.22}, Colors::lightgray, 0.35, "Impact");
	disp.DrawText("2018", {0, 0.22}, Colors::lightgray, 0.4, "Impact");
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze01.svg", 375, 375);
	disp.StartFrame();
	me2.Draw(disp);
//	{
//		xyLoc ss(1,mapSize-2);
//		xyLoc gg(mapSize-2,1);
//		me2.SetColor(Colors::lightgray);
//		me2.DrawStateLabel(disp, ss, "s");
//		me2.DrawStateLabel(disp, gg, "g");
//	}
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze02.svg", 375, 375);
	disp.StartFrame();
	me3.Draw(disp);
	disp.DrawText("g", {-0.865, 0.87}, Colors::lightgray, 0.15);
	disp.DrawText("s", {0.865, -0.86}, Colors::lightgray, 0.15);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze03.svg", 375, 375);
	
//	m1->Save("/Users/nathanst/maze01.map");
//	m2->Save("/Users/nathanst/maze02.map");
//	m3->Save("/Users/nathanst/maze03.map");

	m3->SetRectHeight(0, 0, mapSize-1, mapSize-1, 0, kOutOfBounds);
	disp.StartFrame();
	me3.Draw(disp);
	disp.EndFrame();
	MakeSVG(disp, "/Users/nathanst/maze04.svg", 375, 375);

	
	{
		double px1, py1, t1, rad1;
		m1->GetCoord(0, 0, px1, py1, t1, rad1);
		printf("Test: (0, 0) is (%f, %f)\n", px1-rad1, py1-rad1);
		m1->GetCoord((int)m1->GetMapWidth()-1, (int)m1->GetMapHeight()-1, px1, py1, t1, rad1);
		printf("Test: (%d, %d) is (%f, %f)\n",
		       m1->GetMapWidth()-1,
		       m1->GetMapHeight()-1,
		       px1+rad1, py1+rad1);
		
	}
	
}

// In practice we should use the ranking (constant time) instead of this
// n^2 algorithm. But, another time.
edge *GetEdge(Graph *g, int x1, int y1, int z1, int x2, int y2, int z2)
{
	for (int t = 0; t < g->GetNumEdges(); t++)
	{
		edge *e = g->GetEdge(t);
		node *n1 = g->GetNode(e->getFrom());
		node *n2 = g->GetNode(e->getTo());
//		printf("--Trying (%ld, %ld, %ld) (%ld, %ld, %ld)\n",
//			   n1->GetLabelL(0), n1->GetLabelL(1), n1->GetLabelL(2),
//			   n2->GetLabelL(0), n2->GetLabelL(1), n2->GetLabelL(2));
		if (n1->GetLabelL(0) == x1 &&
			n1->GetLabelL(1) == y1 &&
			n1->GetLabelL(2) == z1 &&
			n2->GetLabelL(0) == x2 &&
			n2->GetLabelL(1) == y2 &&
			n2->GetLabelL(2) == z2)
		{
//			printf("-Success\n");
			return e;
		}
		if (n2->GetLabelL(0) == x1 &&
			n2->GetLabelL(1) == y1 &&
			n2->GetLabelL(2) == z1 &&
			n1->GetLabelL(0) == x2 &&
			n1->GetLabelL(1) == y2 &&
			n1->GetLabelL(2) == z2)
		{
//			printf("-Success\n");
			return e;
		}
//		printf("---Failure\n");
	}
	return 0;
}

void MazeMazeTree(int d)
{
	return;
	
	Map *m1 = new Map(d, d);
	Map *tmp = new Map(d/2, d/2);

	MakeMaze(tmp, 1, 1, 1);
	tmp->Print();
	printf("\n--\n");

	MakeMaze(tmp, 1, d/2-2, 1);
	tmp->Print();
	printf("\n--\n");

	MakeMaze(tmp, 1, d/2-2, d/2-2);
	tmp->Print();
	printf("\n--\n");
}
