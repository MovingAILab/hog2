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
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "MM.h"
#include "BidirectionalDijkstra.h"
//#include "GLUtil.h"
#include "RoadMap.h"
#include "WeightedVertexGraph.h"
#include "NBS.h"

bool screenShot = false;
bool recording = false;
bool running = false;
bool showSearch = false;

bool runningBidirectional = false;
bool showSearchBidirectional = false;

bool drawLine = false;
std::vector<graphState> thePath;
std::vector<graphState> thePath2;

point3d lineStart, lineEnd;

std::string graphFile, coordinatesFile;

void LoadGraph();

int search = 0;

RoadMap *ge;

TemplateAStar<graphState, graphMove, RoadMap> astar;
MM<graphState, graphMove, RoadMap> mm;
BIdijkstra<graphState, graphMove, RoadMap> bidijkstra;

uint32_t gStepsPerFrame = 512;//1;
double distance(graphState n1, graphState n2);

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return distance(a, b);
	}
};

GraphDistHeuristic h;
ZeroHeuristic<graphState> z;


class myHeuristic: public Heuristic<unsigned long>
{
	Graph* g;
	double maxSpeed;
	double scale;
	
public:
	myHeuristic(Graph* g,double maxSpeed, double scale){
		this->g = g;
		this->maxSpeed = maxSpeed;
		this->scale = scale;
	}
	
	double HCost(const unsigned long &from, const unsigned long &to) const{
		
		node* a = g->GetNode(from);
		node* b = g->GetNode(to);
		
		double aX=a->GetLabelF(GraphSearchConstants::kXCoordinate);
		double aY=a->GetLabelF(GraphSearchConstants::kYCoordinate);
		double bX=b->GetLabelF(GraphSearchConstants::kXCoordinate);
		double bY=b->GetLabelF(GraphSearchConstants::kYCoordinate);
		
		return scale*sqrt((aX-bX)*(aX-bX)+(aY-bY)*(aY-bY))/(2*maxSpeed);
		
	}
	
};


myHeuristic* h1=NULL;
string resultDataFileName;
string pathToInputFiles = "/home/snesawla/thesisData/inputFiles/";
string pathToResultFiles = "/home/snesawla/thesisData/results/";
int fileExtensionLength = 3;


int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
}

/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	LoadGraph();
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record the screen.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reset Rotations", "Reset the current rotation/translation of the map.", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Print", "Save map as svg.", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Simulate search faster.", kNoModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Simulate search slower.", kNoModifier, ']');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "", "", kNoModifier, 'd');

	
	
	InstallCommandLineHandler(MyCLHandler, "-graph", "-graph <filename>", "Specifies file name for graph. Both graph and coordinates must be supplied.");
	InstallCommandLineHandler(MyCLHandler, "-coord", "-coord <filename>", "Specifies file name for coordinates. Both graph and coordinates must be supplied.");

	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
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
		LoadGraph();
		SetNumPorts(windowID, 1);
	}

}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (ge)
	{

		if (running)
		{
			//std::cout << "Expanding next: " << astar.CheckNextNode() << "\n";
			for (int x = 0; x < gStepsPerFrame && running; x++)
			{
				running = !astar.DoSingleSearchStep(thePath);
			}
			if (!running)
			{
//				for (int x = 1; x < thePath.size(); x++)
//				{
//					ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(true);
//				}
//				for (int x = 0; x < astar.GetNumItems(); x++)
//				{
//					node *n = ge->GetGraph()->GetNode(astar.GetItem(x).data);
//					for (int y = 0; y < n->GetNumEdges(); y++)
//					{
//						n->getEdge(y)->setMarked(true);
//					}
//				}
				std::cout << astar.GetNodesExpanded() << " nodes expanded\n";

//				for (int x = 0; x < bidijkstra.GetNumForwardItems(); x++)
//				{
//					node *n = ge->GetGraph()->GetNode(bidijkstra.GetForwardItem(x).data);
//					for (int y = 0; y < n->GetNumEdges(); y++)
//					{
//						n->getEdge(y)->setMarked(true);
//					}
//				}
//				for (int x = 0; x < bidijkstra.GetNumBackwardItems(); x++)
//				{
//					node *n = ge->GetGraph()->GetNode(bidijkstra.GetBackwardItem(x).data);
//					for (int y = 0; y < n->GetNumEdges(); y++)
//					{
//						n->getEdge(y)->setMarked(true);
//					}
//				}
//				std::cout << bidijkstra.GetNodesExpanded() << " nodes expanded\n";
			}
		}
		if (showSearch)
		{
		  //astar.OpenGLDraw();
		}

		if (runningBidirectional)
		{
			//std::cout << "Expanding next: " << astar.CheckNextNode() << "\n";
			for (int x = 0; x < gStepsPerFrame && runningBidirectional; x++)
			{
				//runningBidirectional = !bidijkstra.DoSingleSearchStep(thePath2);
			}
			if (!runningBidirectional)
			{
				std::cout << bidijkstra.GetNodesExpanded() << " nodes expanded\n";
				//				for (int x = 1; x < thePath.size(); x++)
//				{
//					ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath2[x])->setMarked(true);
//				}
			}
		}
		if (showSearchBidirectional)
		{
		  //bidijkstra.OpenGLDraw();
		}

		if (drawLine)
		{
		  /*
			glLineWidth(5);
			glColor4f(1, 1, 1, 0.9);
			glBegin(GL_LINES);
			glVertex3f(lineStart.x, lineStart.y, -0.01);
			glVertex3f(lineEnd.x, lineEnd.y, -0.01);
			glEnd();
			glLineWidth(1);
		  */
		}
		
		ge->SetColor(1.0, 1.0, 1.0);
		//ge->OpenGLDraw();
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/graph-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		//SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
		if (!running && !runningBidirectional)
			exit(0);
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-graph" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		graphFile = argument[1];
		return 2;
	}
	if (strcmp( argument[0], "-coord" ) == 0 )
	{
		if (maxNumArgs <= 1)
			return 0;
		coordinatesFile = argument[1];
		return 2;
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
	case '|': break;//resetCamera(); break;
		case 'r': recording = !recording; break;
		case '[': if (gStepsPerFrame > 1) gStepsPerFrame /= 2; break;
		case ']': gStepsPerFrame *= 2; break;
		case 'p':
		{
			for (int x = 0; x < 100; x++)
			{
				node *start = ge->GetGraph()->GetRandomNode();
				node *goal = ge->GetGraph()->GetRandomNode();
				BidirectionalProblemAnalyzer<graphState, graphMove, RoadMap>::GetWeightedVertexGraph(start->GetNum(), goal->GetNum(), ge, ge, ge);
			}

//			std::fstream svgFile;
//			ge->SetColor(Colors::darkgray.r, Colors::darkgray.g, Colors::darkgray.b);
//			svgFile.open("/Users/nathanst/graph.svg", std::fstream::out | std::fstream::trunc);
//			svgFile << ge->SVGHeader();
//			svgFile << ge->SVGDraw();
////			svgFile << ge->SVGDraw(bidijkstra.GetMeetingPoint());
//			svgFile << ge->SVGDraw(84366);
//			svgFile << ge->SVGDraw(192587);
//			svgFile << ge->SVGLabelState(192587, "S");
//			svgFile << ge->SVGLabelState(84366, "G");
//
//			svgFile << "</svg>";
//			svgFile.close();
//			exit(0);
		}
		default:
			break;
	}
}

void PrintGDistribution(TemplateAStar<graphState, graphMove, RoadMap> &astar)
{
	std::vector<int> counts;
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
		if (i.where == kClosedList)
		{
			int val = i.g;
			if (val >= counts.size())
				counts.resize(val+1);
			counts[val]++;
		}
	}
	for (int x = 0; x < counts.size(); x++)
	{
		if (x>0)
			counts[x] += counts[x-1];
		if (0 == x%1000)
			printf("%d : %d\n", x, counts[x]);
	}
}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
//	printf("Starting Search\n");
//	running = true;
//	showSearch = true;
//	for (int x = 1; x < thePath.size(); x++)
//	{
//		ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(false);
//	}
//	node *n1 = ge->GetGraph()->GetRandomNode();
//	node *n2 = ge->GetGraph()->GetRandomNode();
//	astar.InitializeSearch(ge, n1->GetNum(), n2->GetNum(), thePath);
	NBS<graphState, graphMove, RoadMap, NBSQueue<graphState, 0>> nbs;
	
	const int count = 200;
	srandom(20170825);
	uint64_t fa=0, ba=0, ma=0, na = 0, nza = 0;
	ZeroHeuristic<graphState> z;
	for (int x = 0; x < count; x++)
	{
		node *start = ge->GetGraph()->GetRandomNode();
		node *goal = ge->GetGraph()->GetRandomNode();
		std::string s = "/Users/nathanst/bidir/roads/COL"+std::to_string(x)+".svg";
		BidirectionalProblemAnalyzer<graphState, graphMove, RoadMap>::GetWeightedVertexGraph(start->GetNum(), goal->GetNum(), ge, ge, ge);//, s.c_str());
		uint64_t f, b, m, n, nz;
		astar.GetPath(ge, start->GetNum(), goal->GetNum(), thePath);
		f = astar.GetNecessaryExpansions();
		if (x == 5)
		{
			printf("Problem %d Forward distribution\n", x);
			PrintGDistribution(astar);
		}
		astar.GetPath(ge, goal->GetNum(), start->GetNum(), thePath);
		if (x == 5)
		{
			printf("Problem %d Backward distribution\n", x);
			PrintGDistribution(astar);
		}
		b = astar.GetNecessaryExpansions();
		m = std::min(f, b);
		printf("%1.2f ", ge->GetPathLength(thePath));
		nbs.GetPath(ge, start->GetNum(), goal->GetNum(), ge, ge, thePath);
		n = nbs.GetNecessaryExpansions();
		printf("%1.2f ", ge->GetPathLength(thePath));
		nbs.GetPath(ge, start->GetNum(), goal->GetNum(), &z, &z, thePath);
		nz = nbs.GetNecessaryExpansions();
		printf("%1.2f\n", ge->GetPathLength(thePath));
		
		printf("%llu %llu %llu %llu %llu\n", f, b, m, n, nz);
		fa += f;
		ba += b;
		ma += m;
		na += n;
		nza += nz;
	}
	printf("--------------------------\n");
	printf("forward backward min nbs nbs0\n");
	printf("--------------------------\n");
	printf("%llu %llu %llu %llu %llu\n", fa/count, ba/count, ma/count, na/count, nza/count);
}

double distance(graphState n1, graphState n2)
{
	Graph *g = ge->GetGraph();
	double dx1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double dx2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return sqrt((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2));
}

#include <regex>

void LoadGraph()
{
	bool timeGraph = true;
	if (strstr(graphFile.c_str(), "road-t") == NULL)
		timeGraph = false;

	ge = new RoadMap(graphFile.c_str(), coordinatesFile.c_str(), timeGraph);
}

//void nothing()
//{
//	double xMin = DBL_MAX;
//	double xMax = -DBL_MAX;
//	double yMin = DBL_MAX;
//	double yMax = -DBL_MAX;
//
//	double scale = 0;
//
//	bool distanceGraph = false;
//	bool timeGraph = false;
//	
//	std::cout<<"file name "<<coordinatesFile.c_str();
//	
////	if(std::regex_match(graphFile.c_str(), std::regex(pathToInputFiles+"USA-road-d.*")))
////		distanceGraph = true;
////	else if(std::regex_match(graphFile.c_str(), std::regex(pathToInputFiles+"USA-road-t.*")))
////		timeGraph = true;
//	
//	
//	std::vector<double> speed;
//	Graph *g = new Graph();
//	node *n = new node("");
//	g->AddNode(n);
//	FILE *f = fopen(coordinatesFile.c_str(), "r");
//	//FILE *f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.co", "r");
//	std::vector<double> xloc, yloc;
//	double minx = DBL_MAX, maxx=-DBL_MAX, miny=DBL_MAX, maxy=-DBL_MAX;
//	int count=0;
//	while (!feof(f))
//	{
//		char line[255];
//		fgets(line, 255, f);
//		if (line[0] == 'v')
//		{
//			double x1, y1;
//			int id;
//			if (3 != sscanf(line, "v %d %lf %lf", &id, &x1, &y1))
//				continue;
//			if(count==0)
//			{
//				std::cout<<"x1 and y1 after scanning\n"<<x1<<" "<<y1;
//				count++;
//			}
//			//assert(id == xloc.size()+1);
//			xloc.push_back(x1);
//			yloc.push_back(y1);
//			//printf("%d: (%f, %f) [%f, %f]\n", xloc.size(), x1, y1, minx, maxx);
//			if (x1 > maxx) maxx = x1;
//			if (x1 < minx) minx = x1;
//			if (y1 > maxy) maxy = y1;
//			if (y1 < miny) miny = y1;
//			//if (maxx > -1)
//		}
//	}
//	std::cout<<"Xloc and yloc before scaling\n"<<xloc[1]<<" "<<yloc[1];
//	fclose(f);
//	printf("x between (%f, %f), y between (%f, %f)\n",
//		   minx, maxx, miny, maxy);
//	scale = std::max(maxx-minx,maxy-miny);
//	printf("Scale is %f\n", scale);
//	double xoff = (maxx-minx)-scale;
//	double yoff = (maxy-miny)-scale;
//	
//	for (unsigned int x = 0; x < xloc.size(); x++)
//	{
//		//printf("(%f, %f) -> ", xloc[x], yloc[x]);
//		xloc[x] -= (minx);
//		xloc[x] /= scale;
//		xloc[x] = xloc[x]*2-1+xoff/scale;
//		
//		yloc[x] -= (miny);
//		yloc[x] /= scale;
//		yloc[x] = yloc[x]*2-1;
//		yloc[x] = -yloc[x]+yoff/scale;
//		
//		if(xloc[x] < xMin)
//			xMin = xloc[x];
//		else if(xloc[x] > xMax)
//			xMax = xloc[x];
//		
//		if(yloc[x] < yMin)
//			yMin = yloc[x];
//		else if(yloc[x] > yMax)
//			yMax = yloc[x];
//		
//		
//		node *n = new node("");
//		int nodeNum = g->AddNode(n);
//		n->SetLabelF(GraphSearchConstants::kXCoordinate, xloc[x]);
//		n->SetLabelF(GraphSearchConstants::kYCoordinate, yloc[x]);
//		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
//		
//		//std::cout<<" grid y size is "<<grid.size();
//		//std::cout<<" grid x size is "<<grid[0].size();
//		
//	}
//	//a 1 2 1988
//	std::cout<<"xMin is "<<xMin<<" yMin is "<<yMin<< " xMax is "<<xMax<<" yMax is "<<yMax<<std::endl;
//	std::cout<<" min x is "<<minx/scale + xoff;
//	std::cout<<" max x is "<<maxx/scale + xoff;
//	std::cout<<" min y is "<<miny/scale + yoff;
//	std::cout<<" max y is "<<maxy/scale + yoff;
//	
//	
//	
//	f = fopen(graphFile.c_str(), "r");
//	//f = fopen("/Users/nathanst/Downloads/USA-road-d.COL.gr", "r");
//	int dups = 0;
//	long double e=0;
//	double minE=DBL_MAX;
//	while (!feof(f))
//	{
////		std::cout<<"reading file\n";
//		char line[255];
//		fgets(line, 255, f);
//		if (line[0] == 'a')
//		{
//			int x1, y1;
//			sscanf(line, "a %d %d %Lf", &x1, &y1,&e);
//			if (g->findDirectedEdge(x1, y1) == 0)
//			{
//				
//				
//				node* n1=g->GetNode(x1);
//				node* n2=g->GetNode(y1);
//				long double xCoord1 = n1->GetLabelF(GraphSearchConstants::kXCoordinate);
//				long double yCoord1 =n1->GetLabelF(GraphSearchConstants::kYCoordinate);
//				
//				long double xCoord2 = n2->GetLabelF(GraphSearchConstants::kXCoordinate);
//				long double yCoord2 =n2->GetLabelF(GraphSearchConstants::kYCoordinate);
//				
//				long double dist = scale*sqrtf((xCoord1-xCoord2)*(xCoord1-xCoord2) + (yCoord1-yCoord2)*(yCoord1-yCoord2))/2;
//				
//				
//				if(e<minE)
//					minE=e;
//				if (fequal(e, 0))
//					printf("**FOUND ZERO-COST EDGE\n");
//				g->AddEdge(new edge(x1, y1, e));
//				
//				speed.push_back(dist/e);
//			}
//			
//			else if(g->findDirectedEdge(x1,y1)->GetWeight() < e)
//				g->findDirectedEdge(x1,y1)->setWeight(e);
//			else{
//				dups++;
//				//printf("Not adding duplicate directed edge between %d and %d\n", x1, y1);
//			}
//		}
//	}
//	
//	std::cout<<"minE "<<minE;
//	bidijkstra.SetEpsilon(minE);
//	//printf("%d dups ignored\n", dups);
//	fclose(f);
//	std::cout<<"closed the file\n";
//	ge = new GraphEnvironment(g);
//	ge->SetDirected(true);
//	std::cout<<"created graph environment\n";
//	
//	double maxSpeed = 0;
//	
//	if(distanceGraph)
//		maxSpeed = *(std::max_element(speed.begin(),speed.end()));
//	else if(timeGraph)
//		maxSpeed = *(std::max_element(speed.begin(),speed.end()));
//	else
//	{
//		std::cout<<"\n Invalid graph type";
//		exit(1);
//	}
//	
//	printf(" max element %lf \n",maxSpeed);
//	
//	// create heuristic & pass max speed
//	
//	h1 = new myHeuristic(g,maxSpeed, scale);
//	astar.SetHeuristic(h1);
//	
//	std::cout<<" Set heuristic for A* \n";
// 
//	//PathFindingKeyHandler13(0,0);
//	//DensityAnalysisKeyHandler(0,0);
//	
//}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			lineStart = loc;
			lineEnd = loc;
			drawLine = true;
			running = false;
			showSearch = false;
			for (int x = 1; x < thePath.size(); x++)
			{
				ge->GetGraph()->findDirectedEdge(thePath[x-1], thePath[x])->setMarked(false);
			}

			return true;
		}
		case kMouseDrag:
		{
			lineEnd = loc;
			return true;
		}
		case kMouseUp:
		{
			printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			lineEnd = loc;

			printf("Starting Search\n");
			if (search == 0 || search == 1)
			{
				running = true;
				showSearch = true;
			}
			else if (search == 2)
			{
				showSearchBidirectional = true;
				runningBidirectional = true;
			}
			// find closest point to start/goal loc and run from there.

			node *start = ge->GetGraph()->GetRandomNode(), *goal = ge->GetGraph()->GetRandomNode();
			double startDist = 20, goalDist = 20; // maximum actual distance is 4^2 = 16
			node_iterator ni = ge->GetGraph()->getNodeIter();
			for (node *next = ge->GetGraph()->nodeIterNext(ni); next;
				 next = ge->GetGraph()->nodeIterNext(ni))
			{
				double xdist = next->GetLabelF(GraphSearchConstants::kXCoordinate)-lineStart.x;
				double ydist = next->GetLabelF(GraphSearchConstants::kYCoordinate)-lineStart.y;
				if (xdist*xdist+ydist*ydist < startDist)
				{
					startDist = xdist*xdist+ydist*ydist;
					start = next;
				}
				xdist = next->GetLabelF(GraphSearchConstants::kXCoordinate)-lineEnd.x;
				ydist = next->GetLabelF(GraphSearchConstants::kYCoordinate)-lineEnd.y;
				if (xdist*xdist+ydist*ydist < goalDist)
				{
					goalDist = xdist*xdist+ydist*ydist;
					goal = next;
				}
			}
			drawLine = false;
//			start = ge->GetGraph()->GetNode(22636);
//			goal = ge->GetGraph()->GetNode(119223);
//			recording = true;
//			start = ge->GetGraph()->GetNode(192587);
//			goal = ge->GetGraph()->GetNode(84366);
			if (start == goal)
			{
				printf("Same start and goal; no search\n");
				running = false;
				showSearch = false;
			}
			else {
				printf("Searching from (%1.2f, %1.2f) to (%1.2f, %1.2f) [%d to %d]\n",
					   start->GetLabelF(GraphSearchConstants::kXCoordinate),
					   start->GetLabelF(GraphSearchConstants::kYCoordinate),
					   goal->GetLabelF(GraphSearchConstants::kXCoordinate),
					   goal->GetLabelF(GraphSearchConstants::kYCoordinate),
					   start->GetNum(), goal->GetNum());
				
				astar.InitializeSearch(ge, start->GetNum(), goal->GetNum(), thePath);
//				astar.SetHeuristic(&h);
				bidijkstra.SetVersion(1);
				bidijkstra.InitializeSearch(ge, start->GetNum(), goal->GetNum(), thePath);
				if (search == 0)
					astar.SetWeight(0);
				else if (search == 1)
					astar.SetWeight(1);
				else if (search == 2)
					bidijkstra.InitializeSearch(ge, start->GetNum(), goal->GetNum(), thePath2);
				else if (search == 3)
					bidijkstra.InitializeSearch(ge, start->GetNum(), goal->GetNum(), thePath2);
				search++;
				search = search%4;
			}
			return true;
		}
	}
	return false;
}
