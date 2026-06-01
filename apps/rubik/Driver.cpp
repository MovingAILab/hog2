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

#include <cassert>
#include <stdlib.h>
#include <limits.h>
#include <unordered_set>
#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "TemplateAStar.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "DiskBitFile.h"
#include "RubiksCube7Edges.h"
#include "RubiksCubeCorners.h"
#include "BFS.h"
#include "BloomFilter.h"
#include <string>
#include "BitVector.h"
#include "MinBloom.h"
#include <deque>
#include <mutex>
#include "ParallelAStarIDAStar.h"

RubiksCube c;
RubiksAction a;
RubiksState s;
RubikPDB *pdb1 = 0;

Rubik7Edge e7;
Rubik7EdgeState e7s;
Rubik7EdgeAction e7a;
void Test(int which);
void TestMinBloom();
void GetInstanceFromStdin(RubiksState &start);

bool readFromStdin = false;
bool recording = false;

std::deque<RubiksAction> animateActions;

void TestCornerRanking();
void TestEdgeRanking();
void GetKorfInstance(RubiksState &start, int which);
void KorfTest(int which, bool use1997heuristic = true);
void PDBFaceTest();
void KorfAll();

const char *pdbLocation = "/Users/nathanst/Desktop/pdb";

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
//	Test(0);
	//PDBFaceTest();
	//KorfAll();
	//TestCornerRanking();
	//TestEdgeRanking();
	InstallHandlers();
	RunHOGGUI(argc, argv, 600);
	
	return 0;
}


void GetSuperFlip(RubiksState &start);

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, 1);

	InstallCommandLineHandler(MyCLHandler, "-test", "-test entries", "Test using 'entries' billion entries from edge pdb");
	InstallCommandLineHandler(MyCLHandler, "-animate", "-animate <apply|reverse>", "Read a sequence of actions from stdin and output animated sequence");

	InstallCommandLineHandler(MyCLHandler, "-buildBloom", "-buildBloom <size> <#GB> <#hash> <dataloc>", "Build a bloom filter using a size/hash combo.");
	InstallCommandLineHandler(MyCLHandler, "-buildMinBloom", "-buildMinBloom <#GB> <#hash> <maxDepth> <dataloc>", "Build a bloom filter using a size/hash combo.");
	InstallCommandLineHandler(MyCLHandler, "-showStats", "-showStats <size> <#hash> <prefix>", "Print out bloom filter stats.");
	
	InstallCommandLineHandler(MyCLHandler, "-bloomSample", "-bloomSample <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-bloomSearch", "-bloomSearch <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-bloomSearchStdin", "-bloomSearchStdin <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations. Problems from stdin.");

	InstallCommandLineHandler(MyCLHandler, "-korf", "-korf <instance>", "Run the nth Korf instance from his 1997 RC paper using the same PDBs.");
	InstallCommandLineHandler(MyCLHandler, "-region", "-region", "Analyze regions (preset problem)");

	InstallCommandLineHandler(MyCLHandler, "-minBloomSearch", "-minBloomSearch <corner-prefix> <other-prefix> <8size> <8hash> <9size> <9hash>", "Use bloom filter + corner pdb. Pass data locations");
	InstallCommandLineHandler(MyCLHandler, "-measure", "-measure interleave", "Measure loss from interleaving versus min");
	InstallCommandLineHandler(MyCLHandler, "-extract", "-extract <file>", "Extract levels from <file>");
	InstallCommandLineHandler(MyCLHandler, "-testBloom", "-testBloom <entires> <accuracy>", "Test bloom filter with <entries> total and given <accuracy>");
	InstallCommandLineHandler(MyCLHandler, "-testCompression", "-testCompression <factor> <type> <edgepdb> <cornerpdb>", "");
	InstallCommandLineHandler(MyCLHandler, "-compress", "-compress <type [corner,n-edge,edge]> <input> <factor> <output>", "Compress provided pdb by a factor of <factor>");
	InstallCommandLineHandler(MyCLHandler, "-pdb", "-pdb <edge> <corner>", "Run tests using edge and corner pdbs");
	
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
//		s.corner.Reset();
//		std::cout << "Base hash: " << c.c.GetStateHash(s.corner) << "\n";
//		c.c.GetStateFromHash(c.c.GetStateHash(s.corner), s.corner);
//		s.edge.Reset();
		s.Reset();
//		s.edge.SetCubeInLoc(1, 5);
//		s.edge.SetCubeInLoc(5, 1);
//		s.edge.SetCubeOrientation(5, false);
//		s.edge.SetCubeOrientation(1, false);

//		c.ApplyAction(s, 2);
//		c.ApplyAction(s, 8);
//		c.ApplyAction(s, 1);
//		c.ApplyAction(s, 3);
//		c.ApplyAction(s, 17);
//		c.ApplyAction(s, 0);
//		c.ApplyAction(s, 4);
//		c.ApplyAction(s, 8);
		

		//glClearColor(0.8, 0.8, 0.8, 1.0);

		if (0)
		{
			//std::vector<int> edges = {0,1,2,3,4,5};
			std::vector<int> edges = {0, 1, 2, 3, 4, 5};
			std::vector<int> corners = {0, 1, 2, 3};
			s.Reset();
			pdb1 = new RubikPDB(&c, s, edges, corners);
			//recording = true;
			SetNumPorts(windowID, 1);
			//SetZoom(windowID, 6);
		}
		else if (animateActions.size() > 0)
		{
			SetNumPorts(windowID, 1);
			//SetZoom(windowID, 6);
		}
		else {
		  //SetNumPorts(windowID, 4);
		}
		//GetSuperFlip(s);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	auto &display = GetContext(windowID)->display;
	c.Draw(display, s);
	return;
//	bool endRecording = false;
//	if (viewport == 3)
//	{
//		c.OpenGLDrawEdgeDual(s);
//	}
//	else if (viewport == 2)
//	{
//		RubiksState tmp = s;
//		tmp.edge = RotateRubikEdgeClockwise(s.edge, 1);
//		c.OpenGLDrawEdges(tmp);
//	}
//	else if (viewport == 1)
//	{
//		c.OpenGLDrawCorners(s);
//	}
//	else {
//		static double tmp = 0;
//		glRotatef(25, 1, 0, 0);
//		glRotatef(-45+tmp, 0, 1, 0);
//		tmp += 1;
//		if (tmp > 360)
//			endRecording = true;
//		
//		if (pdb1 != 0)
//		{
//			pdb1->OpenGLDraw();
//		}
//		else if (animateActions.size() == 0)
//		{
//			c.OpenGLDraw(s);
//			
//		}
//		else {
//			static float t = 0.0;
//			RubiksState s2 = s;
//			c.ApplyAction(s2, animateActions.front());
//			c.OpenGLDraw(s, s2, t);
//			t+=0.02;
//			if (t >= 1)
//			{
//				s = s2;
//				animateActions.pop_front();
//				t = 0;
//			}
//			if (animateActions.size() == 0)
//				endRecording = true;
//		}
//	}
//	if (recording && viewport == GetNumPorts(windowID)-1)
//	{
//		static int cnt = 0;
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/Rubik-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
//		//SaveScreenshot(windowID, fname);
//		printf("Saved %s\n", fname);
//		cnt++;
//	}
//	if (endRecording)
//		recording = false;
}

void RunTest(int billionEntriesToLoad);
void MeasurePDBCompression(int itemsCompressed);
void Compress(const char *pdbType, const char *theFile, const char *compresType, int ratio, const char *outFile);
//void RunCompressionTest(int factor, const char *compType, const char *edgePDB, const char *cornerPDB);
void RunCompressionTest(int factor, const char *compType, const char *edgePDBmin, const char *edgePDBint, const char *cornerPDB);
void RunSimpleTest(const char *edgePDB, const char *cornerPDB);
void TestBloom(int entries, double accuracy);
void TestBloom2(int entries, double accuracy);
void ExtractStatesAtDepth(const char *theFile);
void SampleBloomHeuristics(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9);
void RunBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9);
void RunMinBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float space, int numHash);
void ManyCompression();
void BuildDepthBloomFilter(int size, float space, int numHash, const char *dataLoc);
void GetBloomStats(uint64_t size, int hash, const char *prefix);
void BuildMinBloomFilter(float space, int numHash, int finalDepth, const char *dataLoc);
void GetActionsFromStdin(std::vector<RubiksAction> &acts);
void RegionAnalysis();

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs < 1)
		return 0;

	if (strcmp(argument[0], "-test") == 0)
	{
		RunTest(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-extract") == 0)
	{
		ExtractStatesAtDepth(argument[1]);
		exit(0);
	}
	else if (strcmp(argument[0], "-region") == 0)
	{
		RegionAnalysis();
		exit(0);
	}
	else if (strcmp(argument[0], "-korf") == 0)
	{
		KorfTest(atoi(argument[1]));
		exit(0);
	}
	else if (strcmp(argument[0], "-animate") == 0)
	{		
		animateActions.resize(0);
		recording = true;
		std::vector<RubiksAction> acts;
		GetActionsFromStdin(acts);
		
		if (strcmp(argument[1], "apply") == 0)
		{
			for (size_t x = 0; x < acts.size(); x++)
			{
				animateActions.push_back(acts[x]);
			}
		}
		if (strcmp(argument[1], "reverse") == 0)
		{
			s.Reset();
			for (int x = acts.size()-1; x >= 0; x--)
			{
				RubiksAction a = acts[x];
				c.InvertAction(a);
				c.ApplyAction(s, a);
				animateActions.push_front(acts[x]);
			}
		}
		return 2;
	}
	else if (strcmp(argument[0], "-measure") == 0)
	{
		MeasurePDBCompression(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-testBloom") == 0)
	{
		TestBloom(atoi(argument[1]), atof(argument[2]));
		TestBloom2(atoi(argument[1]), atof(argument[2]));
		exit(0);
	}
	else if (strcmp(argument[0], "-buildBloom") == 0)
	{
		BuildDepthBloomFilter(atoi(argument[1]), atof(argument[2]), atoi(argument[3]), argument[4]);
		exit(0);
	}
	else if (strcmp(argument[0], "-buildMinBloom") == 0)
	{
		BuildMinBloomFilter(atof(argument[1]), atoi(argument[2]), atoi(argument[3]), argument[4]);
		exit(0);
	}
	else if (strcmp(argument[0], "-bloomSearch") == 0)
	{
		RunBloomFilterTest(argument[1], argument[2], atof(argument[3]), atoi(argument[4]), atof(argument[5]), atoi(argument[6]));
		exit(0);
	}
	else if (strcmp(argument[0], "-bloomSearchStdin") == 0)
	{
		readFromStdin = true;
		RunBloomFilterTest(argument[1], argument[2], atof(argument[3]), atoi(argument[4]), atof(argument[5]), atoi(argument[6]));
		exit(0);
	}
	else if (strcmp(argument[0], "-bloomSample") == 0)
	{
		SampleBloomHeuristics(argument[1], argument[2], atof(argument[3]), atoi(argument[4]), atof(argument[5]), atoi(argument[6]));
		exit(0);
	}
	else if (strcmp(argument[0], "-minBloomSearch") == 0)
	{
		RunMinBloomFilterTest(argument[1], argument[2], atof(argument[3]), atoi(argument[4]));
		exit(0);
	}
	else if (strcmp(argument[0], "-pdb") == 0)
	{
		RunSimpleTest(argument[1], argument[2]);
		exit(0);
	}
	else if (strcmp(argument[0], "-compress") == 0)
	{
		//		ManyCompression(); exit(0);
		if (maxNumArgs < 5)
		{
			printf("Insufficient number of arguments\n");
			exit(0);
		}
		//InstallCommandLineHandler(MyCLHandler, "-compress", "-compress <e> <file> <factor>", "Compress provided pdb by a factor of <factor>");
		printf("Calling compress (%d args available)\n", maxNumArgs); fflush(stdout);
		Compress(argument[1], argument[2], argument[3], atoi(argument[4]), argument[5]);
		exit(0);
	}
	else if (strcmp(argument[0], "-testCompression") == 0)
	{
		RunCompressionTest(atoi(argument[1]), argument[2], argument[3], argument[4], argument[5]);
		exit(0);
	}
	else if (strcmp(argument[0], "-showStats") == 0)
	{
		GetBloomStats(strtoull(argument[1], 0, 10), strtol(argument[2], 0, 10), argument[3]);
		exit(0);
	}
	//	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
//		case '6':
//		case '7':
//		case '8':
			//e7.GetStateFromHash(e7.GetStateHash(e7s), e7s);
			//printf("Old hash is %llu. ", c.GetStateHash(s));
			c.ApplyAction(s, (key-'0')*3);
			e7.ApplyAction(e7s, (key-'0')*3);
			//printf("New is %llu\n", c.GetStateHash(s));
			printf("rank is %llu\n", e7.GetStateHash(e7s));
			break;
		case '9':
		{
			static int a = 0;
			e7.GetStateFromHash(a++, e7s);
//			int a = random()%18;
//			c.ApplyAction(s, a);
//			e7.ApplyAction(e7s, a);
			printf("rank is %llu\n", e7.GetStateHash(e7s));
		}
			break;
		case '\t':
			break;
		case 'p':
		{
			SetNumPorts(windowID, 1);
			//SetZoom(windowID, 6);
			s.Reset();
			animateActions.push_back(0);
			animateActions.push_back(3);
			animateActions.push_back(7);
			animateActions.push_back(9);
			animateActions.push_back(12);
			animateActions.push_back(16);
			animateActions.push_back(0);
			animateActions.push_back(3);
			printf("Resetting state\n");
		}
		{
			RubiksCube cube;
			std::vector<int> blank;
			RubiksState goal;
			goal.Reset();
			
			const char *hprefix = "/Users/nathanst/Desktop/pdb";
			std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
			std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			std::vector<int> edges3 = {0, 2, 4};
			std::vector<int> corners2 = {0, 1};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, edges3, corners2);
			
			//	assert(!"File names are getting corrupted here. Perhaps by the abstraction of the goal state");
			//	assert(!"Need to abstract the goal state immediately when creating the pdb instead of only when I build the pdb");
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
				pdb1->PrintHistogram();
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
				pdb2->PrintHistogram();
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
				pdb3->PrintHistogram();
			}
			if (!pdb4->Load(hprefix))
			{
				pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb4->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
				pdb4->PrintHistogram();
			}
			
		}
			break;
		case 'o':
//		{
//			Rubik7EdgeState es7, dual7;
//			Rubik7Edge e7;
//
//			RubikEdgeState es, dual;
//			RubikEdge e;
//
//			static std::vector<RubikEdgeAction> acts;
//			for (int t = 0; t < 5; t++)
//				acts.push_back(random()%18);
//			es7.Reset();
//			es.Reset();
//			e.GetStateFromHash(12345, es);
//			es7.state = es.state;
//			std::cout << "Strt: " << es7 << std::endl;
//			std::cout << "Strt: " << es << std::endl;
//			std::cout << "Acts: ";
//			for (unsigned int x = 0; x < acts.size(); x++)
//			{
//				if (x > 0)
//					std::cout << ", ";
//				e7.ApplyAction(es7, acts[x]);
//				e.ApplyAction(es, acts[x]);
//				std::cout << acts[x];
//			}
//			std::cout << std::endl;;
//			std::cout << "Init: " << es7 << std::endl;
//			std::cout << "Init: " << es << std::endl;
//			es7.GetDual(dual7);
//			es.GetDual(dual);
//			std::cout << "Dual: " << dual7 << std::endl;
//			std::cout << "Dual: " << dual << std::endl;
//			es7.Reset();
//			es.Reset();
//			e.GetStateFromHash(12345, es);
//			es7.state = es.state;
//			for (unsigned int x = 0; x < acts.size(); x++)
//			{
//				e7.UndoAction(es7, acts[acts.size()-x-1]);
//				e.UndoAction(es, acts[acts.size()-x-1]);
//			}
//			std::cout << "Undo: " << es7 << std::endl;
//			std::cout << "Undo: " << es << std::endl;
//			//std::cout << dual << std::endl << std::endl;
//			assert(dual7 == es7);
//			assert(dual == es);
//			for (unsigned int x = 0; x < acts.size(); x++)
//			{
//				e7.ApplyAction(es7, acts[x]);
//				e.ApplyAction(es, acts[x]);
//			}
//			std::cout << es7 << std::endl << std::endl;
//			std::cout << es << std::endl << std::endl;
//		}
			break;
		case ']':
			s.Reset();
			for (int x = 0; x < 5; x++)
				c.ApplyAction(s, x);
			break;
		case '[':
			s.Reset();
			for (int x = 4; x >= 0; x--)
				c.ApplyAction(s, x);
			break;
		default:
			//if (unitSim)
			//	unitSim->GetEnvironment()->GetMapAbstraction()->ToggleDrawAbstraction(((mod == kControlDown)?10:0)+(key-'0'));
			break;
	}
}

void Compress(const char *pdbType, const char *theFile, const char *compressType, int ratio, const char *outFile)
{
	printf("Inside compress\n"); fflush(stdout);
	bool minCompression = true;

	if (strcmp(compressType, "interleave") == 0)
	{
		minCompression = false;
	}
	else if (strcmp(compressType, "min") == 0)
	{
		minCompression = true;
	}
	else {
		printf("Unknown compression '%s'\n", compressType);
		exit(0);
	}
	printf("Done setting compression type\n"); fflush(stdout);
	if (strcmp(pdbType, "corner") == 0)
	{
		printf("Done test a\n"); return;
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;
		
		uint64_t maxBuckSize = GetMaxBucketSize<RubiksCorner, RubiksCornerState>(true);
		InitTwoPieceData<RubiksCorner, RubiksCornerState>(data, maxBuckSize);
		InitBucketSize<RubiksCorner, RubiksCornerState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);

		uint64_t avgReg = 0;
		uint64_t avgComp = 0;

		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
				}
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					minValue = std::min(minValue, val);
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						avgComp += minValue;
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
	}
	if (strcmp(pdbType, "n-edge") == 0)
	{
		printf("Done test b\n"); return;
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;
		
		uint64_t maxBuckSize = GetMaxBucketSize<Rubik7Edge, Rubik7EdgeState>(true);
		InitTwoPieceData<Rubik7Edge, Rubik7EdgeState>(data, maxBuckSize);
		InitBucketSize<Rubik7Edge, Rubik7EdgeState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);
		
		uint64_t avgReg = 0;
		uint64_t avgComp = 0;
		
		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
				}
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n"); fflush(stdout);
			int64_t index = 0;
			int64_t realIndex = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					minValue = std::min(minValue, val);
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						avgComp += minValue;
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
	}		
	if (strcmp(pdbType, "edge") == 0)
	{
		FourBitArray b;
		
		std::vector<bucketInfo> data;
		std::vector<bucketData> buckets;

		uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);

		InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
		InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
		int64_t totalSize = 0;
		for (unsigned int x = 0; x < buckets.size(); x++)
		{
			totalSize += buckets[x].theSize;
		}
		totalSize = (totalSize+ratio-1)/ratio;
		b.Resize(totalSize);
		
		DiskBitFile f(theFile);

		uint64_t avgReg = 0;
		uint64_t avgComp = 0;
		
		float currPerc = 1;
		if (!minCompression)
		{ // just write first  value
			printf("Performing interleave compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					//mem[index++] = val;
					assert(realIndex < b.Size());
					if (0 == index%ratio)
					{
						b.Set(realIndex++, val);
						avgComp += val;
					}
					index++;
					if (float(realIndex)/float(totalSize) > currPerc)
					{
						printf("%1.0f%% ", currPerc);
						currPerc++;
					}
				}
			}
			printf("\n");
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/realIndex);
		}
		else { // write the min of the values
			printf("Performing min compression\n");
			int64_t index = 0;
			int64_t realIndex = 0;
			int64_t written = 0;
			int minValue = 0xFF;
			for (unsigned int x = 0; x < data.size(); x++)
			{
				for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
				{
					int val = f.ReadFileDepth(data[x].bucketID, y);
					avgReg += val;
					assert(realIndex < b.Size());
					index++;
					if (minValue >= 8)
					{
						minValue = std::min(minValue, val);
					}
					if (0 == index%ratio)
					{
						b.Set(realIndex++, minValue);
						if (minValue != 0xFF)
						{
							avgComp += minValue;
							written++;
						}
						minValue = 0xFF;
					}
				}
			}
			if (minValue != 0xFF)
			{
				b.Set(realIndex++, minValue);
				if (minValue != 0xFF)
				{
					avgComp += minValue;
					written++;
				}
			}
			b.Write(outFile);
			printf("%llu entries compressed into %llu\n", index, realIndex);
			printf("Original average: %f; Compressed average: %f\n", (float)avgReg/index, (float)avgComp/written);
			printf("%llu entries didn't compress to a value\n", realIndex-written);
		}
	}
}

void ManyCompression()
{
	const int toCompress = 3;
	FourBitArray b[toCompress];
	int factor[toCompress] = {15, 20, 25};//{ 30, 35, 40, 45, 50 }; //10, 15, 20, 25,
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;
	for (unsigned int x = 0; x < buckets.size(); x++)
	{
		totalSize += buckets[x].theSize;
	}
	//totalSize = (totalSize+ratio-1)/ratio;
	for (int x = 0; x < toCompress; x++)
	{
		b[x].Resize((totalSize+factor[x]-1)/factor[x]);
	}
	//b.Resize(totalSize);
	
	DiskBitFile f("/data/cc/rubik/res/RC");

	
	printf("Performing min compression\n"); fflush(stdout);
	uint64_t avgReg = 0;
	uint64_t avgComp[toCompress];//(0);// = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int64_t realIndex[toCompress];// = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int64_t index = 0;
	int minValue[toCompress];// = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	for (int x = 0; x < toCompress; x++)
	{
		avgComp[x] = 0; realIndex[x] = 0;
		minValue[x] = 0xFF;
	}
	for (unsigned int x = 0; x < data.size(); x++)
	{
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);
			avgReg += val;
			index++;
			for (int r = 0; r < toCompress; r++)
			{
				minValue[r] = std::min(minValue[r], val);
				if (0 == index%factor[r])
				{
					b[r].Set(realIndex[r]++, minValue[r]);
					avgComp[r] += minValue[r];
					minValue[r] = 0xFF;
				}
			}
		}
	}
	for (int r = 0; r < toCompress; r++)
	{
		if (minValue[r] != 0xFF)
		{
			b[r].Set(realIndex[r]++, minValue[r]);
		}
		char name[255];
		sprintf(name, "edge-min-%dx.pdb", factor[r]);
		b[r].Write(name);
		printf("%dx compression:\n", factor[r]);
		printf("%llu entries compressed into %llu\n", index, realIndex[r]);
		printf("Original average: %f; Compressed average: %f\n\n", (float)avgReg/index, (float)avgComp[r]/realIndex[r]);
	}

}

void LoadCornerPDB()
{
//	if (c.GetCornerPDB().Size() != 0)
//		return;
//	FourBitArray &b = c.GetCornerPDB();
//	
//	//uint8_t *mem;
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<RubiksCorner, RubiksCornerState>(true);
//	InitTwoPieceData<RubiksCorner, RubiksCornerState>(data, maxBuckSize);
//	InitBucketSize<RubiksCorner, RubiksCornerState>(buckets, maxBuckSize);
//	int64_t totalSize = 0;
//	for (unsigned int x = 0; x < buckets.size(); x++)
//		totalSize += buckets[x].theSize;
//	b.Resize(totalSize);
//	//mem = new uint8_t[totalSize];
//	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC");
//	
//	int64_t index = 0;
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//			int val = f.ReadFileDepth(data[x].bucketID, y);
//			//mem[index++] = val;
//			b.Set(index++, val);
//		}
//	}
//	//c.SetCornerPDB(mem);
}

void LoadEdgePDB(uint64_t sizeLimit)
{
//	if (c.GetEdgePDB().Size() != 0)
//		return;
//	FourBitArray &b = c.GetEdgePDB();
//
//	//uint8_t *mem;
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
//	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
//	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
//	int64_t totalSize = 0;
//	for (unsigned int x = 0; x < buckets.size(); x++)
//		totalSize += buckets[x].theSize;
//
//	b.Resize((totalSize+sizeLimit-1)/sizeLimit);
//	//DiskBitFile f("/data/cc/rubik/final/RC");
//	DiskBitFile f("/store/rubik/RC");
//	int64_t index = 0;
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//		  if (0 == y%sizeLimit)
//		    {
//		      int val = f.ReadFileDepth(data[x].bucketID, y);
//		      b.Set(index++, val);
//		    }
//		}
//	}
////	c.SetEdgePDB(mem, totalSize);
}

void LoadEdgePDBOLD(uint64_t sizeLimit)
{
//	//
//	if (c.GetEdgePDB().Size() != 0)
//		return;
//	FourBitArray &b = c.GetEdgePDB();
//
//	//uint8_t *mem;
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
//	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
//	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
//	int64_t totalSize = 0;
////	const int64_t sizeLimit = 1000000000;
//	for (unsigned int x = 0; x < buckets.size(); x++)
//		totalSize += buckets[x].theSize;
//	if (totalSize > sizeLimit)
//		totalSize = sizeLimit;
//	//mem = new uint8_t[totalSize];
//	b.Resize(totalSize);
//	DiskBitFile f("/data/cc/rubik/final/RC");
//	//DiskBitFile f("/store/rubik/RC");
//	int64_t index = 0;
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//			int val = f.ReadFileDepth(data[x].bucketID, y);
//			b.Set(index++, val);
//			//mem[index++] = val;
//			if (index >= totalSize)
//				break;
//		}
//		if (index >= totalSize)
//			break;
//	}
////	c.SetEdgePDB(mem, totalSize);
}

void LoadEdge7PDB(uint64_t sizeLimit = 0x7FFFFFFFFFFFFFFFull);

void LoadEdge7PDB(uint64_t sizeLimit)
{
//	if (c.GetEdge7PDB().Size() != 0)
//		return;
//	FourBitArray &b = c.GetEdge7PDB();
//	
//	//uint8_t *mem;
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<Rubik7Edge, Rubik7EdgeState>(true);
//	InitTwoPieceData<Rubik7Edge, Rubik7EdgeState>(data, maxBuckSize);
//	InitBucketSize<Rubik7Edge, Rubik7EdgeState>(buckets, maxBuckSize);
//	int64_t totalSize = 0;
//	//	const int64_t sizeLimit = 1000000000;
//	for (unsigned int x = 0; x < buckets.size(); x++)
//	  totalSize += buckets[x].theSize;
//	//mem = new uint8_t[totalSize];
//	b.Resize(totalSize);
//	DiskBitFile f("/home/sturtevant/sturtevant/code/cc/rubik/RC-7edge");//RC-9edge");
//	int64_t index = 0;
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//			int val = f.ReadFileDepth(data[x].bucketID, y);
//			b.Set(index++, val);
//			//mem[index++] = val;
//			if (index >= totalSize)
//				break;
//		}
//		if (index >= totalSize)
//			break;
//	}
//	//	c.SetEdgePDB(mem, totalSize);
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	while (true)
	{
		c.GetActions(s, acts);
		for (unsigned int x = 0; x < acts.size(); x++)
			printf("%d) %d\n", x, acts[x]);
		printf("Choose action to apply: ");
		int which;
		std::cin >> which;
		c.ApplyAction(s, acts[which]);
	}
}

void GetInstance(RubiksState &start, int which);
void SolveOneProblem(int instance, const char *txt)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	c.SetPruneSuccessors(true); // clears history

	if (readFromStdin)
	{
		GetInstanceFromStdin(start);
	}
	else {
		GetInstance(start, instance);
	}
	
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
//	c.percentage = percentage;
	s = start;
	IDAStar<RubiksState, RubiksAction> ida;
	c.SetPruneSuccessors(true);
	Timer t;
	t.StartTimer();
	ida.SetUseBDPathMax(true);
	ida.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("[%s] Problem %d - %llu expanded; %1.2f elapsed\n", txt, instance+1, ida.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		printf("%d ", acts[x]);
		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
	fflush(stdout);
}

void SolveOneProblemAStar(int instance)
{
	RubiksState start, goal;
	goal.Reset();
	start.Reset();
	c.SetPruneSuccessors(false); // clears history

	if (readFromStdin)
	{
		GetInstanceFromStdin(start);
	}
	else {
		GetInstance(start, instance);
	}
	
	std::vector<RubiksState> acts;
	c.SetPruneSuccessors(true);
	//	c.percentage = percentage;
	s = start;
	TemplateAStar<RubiksState, RubiksAction, RubiksCube> astar;
	Timer t;
	t.StartTimer();
	astar.SetUseBPMX(1);
	astar.GetPath(&c, start, goal, acts);
	t.EndTimer();
	printf("[AStar] Problem %d - %llu expanded; %1.2f elapsed\n", instance+1, astar.GetNodesExpanded(), t.GetElapsedTime());
	for (unsigned int x = 0; x < acts.size()-1; x++)
	{
		printf("%d ", c.GetAction(acts[x], acts[x+1]));
//		c.ApplyAction(s, acts[x]);
	}
	printf("\n");
	fflush(stdout);
}


void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
	readFromStdin = true;
	s.Reset();
	//GetInstanceFromStdin(s);
	SolveOneProblem(0, "online");
	//TestMinBloom();
	//	LoadCornerPDB();
//	//LoadEdge7PDB();
//	LoadEdgePDB(10);
//	//LoadEdgePDBOLD(2000000000);
//
//	for (int x = 3; x < 4; x++)
//	{
//		srandom(9283+x*23);
//		SolveOneProblem(x, "online");
//	}
//	
//	LoadCornerPDB();
	//	static int t = 0;
//	c.GetStateFromHash(t, s);
//	t++;
//	return;
}

void RunSimpleTest(const char *edgePDB, const char *cornerPDB)
{
//	FourBitArray &corner = c.GetCornerPDB();
//	corner.Read(cornerPDB);
//	FourBitArray &edge = c.GetEdgePDB();
//	edge.Read(edgePDB);
//	c.compressionFactor = 2;
//	for (int x = 0; x < 100; x++)
//	{
//		srandom(9283+x*23);
//		SolveOneProblem(x, "2x");
//		//		SolveOneProblemAStar(x);
//	}
}

int countBits(int64_t val)
{
	int count = 0;
	while (val)
	{
		if (val&1)
			count++;
		val >>= 1;
	}
	return count;
}

void BuildDepthBloomFilter(int size, float space, int numHash, const char *dataLoc)
{
	printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
	space = space*8*1024*1024*1024;
	uint64_t depth9states = 11588911021ull;
	
	BloomFilter *bf = new BloomFilter(space, numHash, true, true);
	printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n", size, bf->GetStorage(),
		   bf->GetStorage()/8.0/1024.0/1024.0,
		   bf->GetStorage()/8.0/1024.0/1024.0/1024.0);
	printf("%d hashes being used\n", bf->GetNumHash());
	
	printf("Building hash table/bloom filter\n"); fflush(stdout);
	RubikEdge e;
	RubikEdgeState es;
	
	int x = size;
	
	char name[255];
	sprintf(name, "%s12edge-depth-%d.dat", dataLoc, x);
	FILE *f = fopen(name, "r");
	if (f == 0)
	{
		printf("Error opening %s; aborting!\n", name);
		exit(0);
	}
	printf("Reading from '%s'\n", name);
	fflush(stdout);
	
	uint64_t nextItem;
	uint64_t count = 0;
	while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
	{
		if (0 != countBits(nextItem&0xFFF)%2)
			nextItem ^= 1;
		//es.state = nextItem;
		//nextItem = e.GetStateHash(es);
		
		count++;
		bf->Insert(nextItem);
		
		if (0 == count%100000000ull)
		{
			printf("%llu added to table\n", count);
			fflush(stdout);
		}
	}
	printf("%llu items read at depth %d\n", count, x);fflush(stdout);
	fclose(f);
	bf->Analyze();
	delete bf;
}

void BuildMinBloomFilter(float space, int numHash, int finalDepth, const char *dataLoc)
{
	printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
	space = space*2*1024*1024*1024;
				
	MinBloomFilter *bf = new MinBloomFilter(space, numHash, true, true);
	printf("Approximate storage: %llu bits (%1.2f MB / %1.2f GB)\n", bf->GetStorage(),
		   bf->GetStorage()*4.0/8.0/1024.0/1024.0,
		   bf->GetStorage()*4.0/8.0/1024.0/1024.0/1024.0);
	printf("%d hashes being used\n", bf->GetNumHash());

	printf("Building hash table/bloom filter\n"); fflush(stdout);
	RubikEdge e;
	RubikEdgeState es;
	
	for (int x = 0; x <= finalDepth; x++)
	{
		char name[255];
		sprintf(name, "%s12edge-depth-%d.dat", dataLoc, x);
		FILE *f = fopen(name, "r");
		if (f == 0)
		{
			printf("Error opening %s; aborting!\n", name);
			exit(0);
		}
		printf("Reading from '%s'\n", name);
		fflush(stdout);
		
		uint64_t nextItem;
		uint64_t count = 0;
		while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
		{
			if (0 != countBits(nextItem&0xFFF)%2)
				nextItem ^= 1;
			//es.state = nextItem;
			//nextItem = e.GetStateHash(es);
			
			count++;
			bf->Insert(nextItem, x);
			
			if (0 == count%100000000ull)
			{
				printf("%llu added to table\n", count);
				fflush(stdout);
			}
		}
		printf("%llu items read at depth %d\n", count, x);fflush(stdout);
		fclose(f);
	}
	bf->Analyze();
	delete bf;
}

void SampleBloomHeuristics(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9)
{
//	// setup corner pdb
//	{
//		FourBitArray &corner = c.GetCornerPDB();
//		printf("Loading corner pdb: %s\n", cornerPDB);
//		corner.Read(cornerPDB);
//		printf("Done.\n");
//	}
//	
//	bool zero = false;
//	
//	// setup bloom filters
//	{
//		printf("Loading bloom filters.\n");fflush(stdout);
//		//uint64_t depth8states = 1050559626ull;
//		//uint64_t depth9states = 11588911021ull;
//		
//		uint64_t size8filter = size8*1024*1024*1024*8ull;
//		uint64_t size9filter = size9*1024ull*1024ull*1024ull*8ull;
//		
//		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", (int)size8, size8filter, hash8);
//		c.depth8 = new BloomFilter(size8filter, hash8, depthPrefix);
//		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
//			   8, c.depth8->GetStorage(),
//			   c.depth8->GetStorage()/8.0/1024.0/1024.0,
//			   c.depth8->GetStorage()/8.0/1024.0/1024.0/1024.0);
//		printf("%d hashes being used\n", c.depth8->GetNumHash());
//		
//		printf("Loading filter with %d GB of entries (%llu) and %d hashes\n", (int)size9, size9filter, hash9);
//		c.depth9 = new BloomFilter(size9filter, hash9, depthPrefix);
//		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
//			   9, c.depth9->GetStorage(),
//			   c.depth9->GetStorage()/8.0/1024.0/1024.0,
//			   c.depth9->GetStorage()/8.0/1024.0/1024.0/1024.0);
//		printf("%d hashes being used\n", c.depth9->GetNumHash());
//		
//		c.bloomFilter = true;
//		printf("Done.\n");fflush(stdout);
//	}
//	
//	// load hash table / bloom filter data
//	if (1)
//	{
//		printf("Building hash table/bloom filter\n"); fflush(stdout);
//		RubikEdge e;
//		RubikEdgeState es;
//		for (int x = 0; x < 8; x++)
//		{
//			char name[255];
//			sprintf(name, "%s12edge-depth-%d.dat", depthPrefix, x);
//			FILE *f = fopen(name, "r");
//			if (f == 0)
//			{
//				printf("Error opening %s; aborting!\n", name);
//				exit(0);
//			}
//			printf("Reading from '%s'\n", name);
//			fflush(stdout);
//			
//			uint64_t nextItem;
//			uint64_t count = 0;
//			while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
//			{
//				if (0 != countBits(nextItem&0xFFF)%2)
//					nextItem ^= 1;
//				if (x < 8)
//				{
//					count++;
//					c.depthTable[nextItem] = x;
//				}
//				else {
//					printf("Error; hit depth %d\n", x);
//				}
//			}
//			printf("%llu items read at depth %d\n", count, x);fflush(stdout);
//			fclose(f);
//		}
//		
//	}
//	
//	RubiksState start;
//	start.Reset();
//	RubikEdge edge;
//	uint64_t max = edge.getMaxSinglePlayerRank();
//	double val = 0;
//	int cnt = 100000;
//	for (int x = 0; x < cnt; x++)
//	{
//		uint64_t r1 = random(), r2 = random();
//		r1 = (r1<<32)|(r2);
//		r1 = r1%max;
//		edge.GetStateFromHash(r1, start.edge);
//		val += c.HCost(start, start);
////		srandom(9283+x*23);
////		SolveOneProblem(x, "hash7bloom89");
////		//		SolveOneProblemAStar(x);
//	}
//	printf("Edge heuristic distribution\n");
//	for (int x = 0; x < c.edgeDist.size(); x++)
//	{
//		printf("%d : %llu\n", x, c.edgeDist[x]);
//	}
//	printf("Average: %1.4f\n", val/cnt);
//	//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
//	//	bloom_filter *depth8, *depth9;
//	
}

void RunBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float size8, int hash8, float size9, int hash9)
{
//	// setup corner pdb
//	{
//		FourBitArray &corner = c.GetCornerPDB();
//		printf("Loading corner pdb: %s\n", cornerPDB);
//		corner.Read(cornerPDB);
//		printf("Done.\n");
//	}
//
//	bool zero = false;
//
//	// setup bloom filters
//	{
//		printf("Loading bloom filters.\n");fflush(stdout);
//		//uint64_t depth8states = 1050559626ull;
//		//uint64_t depth9states = 11588911021ull;
//		
//		uint64_t size8filter = size8*1024*1024*1024*8ull;
//		uint64_t size9filter = size9*1024ull*1024ull*1024ull*8ull;
//
//		//size8filter = 11054728626ull;
//		//size9filter = 121946687506ull;
//		//hash8 = hash9 = 4;
//
//		printf("Loading filter with %1.1f GB of entries (%llu) and %d hashes\n", size8, size8filter, hash8);
//		c.depth8 = new BloomFilter(size8filter, hash8, depthPrefix);
//		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
//			   8, c.depth8->GetStorage(),
//			   c.depth8->GetStorage()/8.0/1024.0/1024.0,
//			   c.depth8->GetStorage()/8.0/1024.0/1024.0/1024.0);
//		printf("%d hashes being used\n", c.depth8->GetNumHash());
//
//		printf("Loading filter with %1.1f GB of entries (%llu) and %d hashes\n", size9, size9filter, hash9);
//		c.depth9 = new BloomFilter(size9filter, hash9, depthPrefix);
//		printf("Approximate storage (%d): %llu bits (%1.2f MB / %1.2f GB)\n",
//			   9, c.depth9->GetStorage(),
//			   c.depth9->GetStorage()/8.0/1024.0/1024.0,
//			   c.depth9->GetStorage()/8.0/1024.0/1024.0/1024.0);
//		printf("%d hashes being used\n", c.depth9->GetNumHash());
//
//		c.bloomFilter = true;
//		printf("Done.\n");fflush(stdout);
//	}
//	
//	// load hash table / bloom filter data
//	if (1)
//	{
//		printf("Building hash table/bloom filter\n"); fflush(stdout);
//		RubikEdge e;
//		RubikEdgeState es;
//		for (int x = 0; x < 8; x++)
//		{
//			char name[255];
//			sprintf(name, "%s12edge-depth-%d.dat", depthPrefix, x);
//			FILE *f = fopen(name, "r");
//			if (f == 0)
//			{
//				printf("Error opening %s; aborting!\n", name);
//				exit(0);
//			}
//			printf("Reading from '%s'\n", name);
//			fflush(stdout);
//
//			uint64_t nextItem;
//			uint64_t count = 0;
//			while (fread(&nextItem, sizeof(uint64_t), 1, f) == 1)
//			{
//				if (0 != countBits(nextItem&0xFFF)%2)
//					nextItem ^= 1;
//				if (x < 8)
//				{
//					count++;
//					c.depthTable[nextItem] = x;
//				}
//				else {
//					printf("Error; hit depth %d\n", x);
//				}
//			}
//			printf("%llu items read at depth %d\n", count, x);fflush(stdout);
//			fclose(f);
//		}
//
//	}
//	
//	for (int x = 0; x < 100; x++)
//	{
//		srandom(9283+x*23);
//		SolveOneProblem(x, "hash7bloom89");
//		//		SolveOneProblemAStar(x);
//	}
//	printf("Edge heuristic distribution\n");
//	for (int x = 0; x < c.edgeDist.size(); x++)
//	{
//		printf("%d : %llu\n", x, c.edgeDist[x]);
//	}
//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
//	bloom_filter *depth8, *depth9;

}

void RunMinBloomFilterTest(const char *cornerPDB, const char *depthPrefix, float space, int numHash)
{
//	// setup corner pdb
//	{
//		FourBitArray &corner = c.GetCornerPDB();
//		printf("Loading corner pdb: %s\n", cornerPDB);
//		corner.Read(cornerPDB);
//		printf("Done.\n");
//	}
//	
//	bool zero = false;
//	
//	// setup bloom filters
//	{
//		printf("Loading min bloom filters.\n");fflush(stdout);
//		
//		printf("Creating bloom filter using %2.1f GB of mem.\n", space);fflush(stdout);
//		space = space*2*1024*1024*1024;
//		
//		MinBloomFilter *bf = new MinBloomFilter(space, numHash, depthPrefix);
//		printf("Approximate storage: %llu bits (%1.2f MB / %1.2f GB)\n", bf->GetStorage(),
//			   bf->GetStorage()*4.0/8.0/1024.0/1024.0,
//			   bf->GetStorage()*4.0/8.0/1024.0/1024.0/1024.0);
//		printf("%d hashes being used\n", bf->GetNumHash());
//		c.minBloom = bf;
//		c.minBloomFilter = true;
//	}
//	
//	for (int x = 0; x < 100; x++)
//	{
//		srandom(9283+x*23);
//		SolveOneProblem(x, "minBloom0-9");
//		//		SolveOneProblemAStar(x);
//	}
//	printf("Edge heuristic distribution\n");
//	for (int x = 0; x < c.edgeDist.size(); x++)
//	{
//		printf("%d : %llu\n", x, c.edgeDist[x]);
//	}
//	//	::std::tr1::unordered_map<uint64_t, uint8_t> hash;
//	//	bloom_filter *depth8, *depth9;
//	
}


void RunCompressionTest(int factor, const char *compType, const char *edgePDBmin, const char *edgePDBint, const char *cornerPDB)
{
//	FourBitArray &corner = c.GetCornerPDB();
//	printf("Loading corner pdb: %s\n", cornerPDB);
//	corner.Read(cornerPDB);
//	FourBitArray &edgemin = c.GetEdgePDB();
//	printf("Loading edge min pdb: %s\n", edgePDBmin);
//	edgemin.Read(edgePDBmin);
//	//FourBitArray &edgeint = c.GetEdge7PDB(false);
//	//printf("Loading edge interleave pdb: %s\n", edgePDBint);
//	//edgeint.Read(edgePDBint);
//
//	c.compressionFactor = factor;
//	if (strcmp(compType, "min") == 0)
//	{
//		printf("Using min compression (%dx)\n", factor);
//		c.minCompression = true;
//	}
//	else {
//		printf("Using interleave compression (%dx)\n", factor);
//		c.minCompression = false;
//	}
//
//	for (int x = 0; x < 100; x++)
//	{
//		for (int t = 0; t <= 0; t++)
//		{
//			if (t == 0)
//			{
//				printf("Solving with min compression\n");
//				c.minCompression = true;
//			}
//			else {
//				printf("Solving with interleave compression\n");
//				c.minCompression = false;
//			}
//			{
//				srandom(9283+x*23);
//				SolveOneProblem(x, (t==0)?"min":"int");
//				//SolveOneProblemAStar(x);
//			}
//		}
//	}
////	for (int x = 0; x < c.edgeDist.size(); x++)
////	{
////		printf("edgeDist[%3d] = %llu\n", x, c.edgeDist[x]);
////	}
////	for (int x = 0; x < c.cornDist.size(); x++)
////	{
////		printf("cornDist[%3d] = %llu\n", x, c.cornDist[x]);
////	}
}

#pragma mark -

void TestCornerRanking()
{
	RubiksCorner corner;
	RubiksCornerState cornerState, correct;
	//std::vector<int> distinctCorners = {3, 1, 6, 4};
	std::vector<int> distinctCorners = {0, 1, 2, 3, 4, 5, 6, 7};
	RubikCornerPDB pdb(&corner, cornerState, distinctCorners);
	//	correct.Reset();
	//	std::cout << correct << "!\n";
	//	corner.GetStateFromHash(40319, correct);
	//	std::cout << correct << "!!\n";
	for (uint64_t x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, cornerState);
		uint64_t hash = pdb.GetPDBHash(cornerState);
		assert(hash == x);
		if (0 == x%100000)
		{
			std::cout << x << " " << pdb.GetPDBHash(cornerState) << " : " << cornerState << "\n";
		}
		for (int y = 0; y < 18; y++)
		{
			//corner.ApplyAction(correct, y);
			corner.ApplyAction(cornerState, y);
			corner.InvertAction(y);
			assert(pdb.GetPDBHash(cornerState) < pdb.GetPDBSize());
			//std::cout << x << " " << pdb.GetPDBHash(cornerState) << " : " << cornerState << "\n";
			//corner.ApplyAction(correct, y);
			corner.ApplyAction(cornerState, y);
			corner.InvertAction(y);
			//std::cout << x << " " << pdb.GetPDBHash(cornerState) << " : " << cornerState << "\n";
			assert(pdb.GetPDBHash(cornerState) == x);
		}
	}
	cornerState.Reset();
	pdb.BuildPDB(cornerState, std::thread::hardware_concurrency());
	exit(0);
}

void TestEdgeRanking()
{
	RubikEdge edge;
	RubikEdgeState edgeState, correct;
	//std::vector<int> distinctCorners = {3, 1, 6, 4};
	//std::vector<int> distinctEdges = {0, 2, 4, 5, 6, 7};
	std::vector<int> distinctEdges = {1, 3, 8, 9, 10, 11};
	RubikEdgePDB pdb(&edge, edgeState, distinctEdges);
	//	correct.Reset();
	//	std::cout << correct << "!\n";
	//	corner.GetStateFromHash(40319, correct);
	//	std::cout << correct << "!!\n";
	for (uint64_t x = 0; x < pdb.GetPDBSize(); x+=100)
	{
		pdb.GetStateFromPDBHash(x, edgeState);
		uint64_t hash = pdb.GetPDBHash(edgeState);
		assert(hash == x);
		if (0 == x%100000)
		{
			std::cout << x << " " << pdb.GetPDBHash(edgeState) << " : " << edgeState << "\n";
		}
		for (int y = 0; y < 18; y++)
		{
			//edge.ApplyAction(correct, y);
			edge.ApplyAction(edgeState, y);
			edge.InvertAction(y);
			assert(pdb.GetPDBHash(edgeState) < pdb.GetPDBSize());
			//std::cout << x << " " << pdb.GetPDBHash(edgeState) << " : " << edgeState << "\n";
			//edge.ApplyAction(correct, y);
			edge.ApplyAction(edgeState, y);
			edge.InvertAction(y);
			//std::cout << x << " " << pdb.GetPDBHash(edgeState) << " : " << edgeState << "\n";
			assert(pdb.GetPDBHash(edgeState) == x);
		}
	}
	edgeState.Reset();
	pdb.BuildPDB(edgeState, std::thread::hardware_concurrency());
	exit(0);
}

void PDBFaceTest()
{
	RubiksCube cube;
	RubiksState goal;
	RubiksState start;
	GetKorfInstance(start, 0);
	std::vector<int> edges = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> corners = {};//{0, 1, 2};
	RubikPDB pdb1(&cube, goal, edges, corners);
	goal.Reset();
	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//	Heuristic<RubiksState> h;
//	h.lookups.push_back({kLeafNode, 0, 0});
//	h.lookups.push_back({kLeafNode, 1, 0});
//	h.lookups.push_back({kLeafNode, 2, 0});
//	h.heuristics.push_back(&pdb1);
//	h.heuristics.push_back(&pdb2);
//	h.heuristics.push_back(&pdb3);
//	
//	cube.SetPruneSuccessors(true);
//	Timer t;
//	t.StartTimer();
//	std::vector<RubiksAction> path;
//	IDAStar<RubiksState, RubiksAction> ida;
//	ida.SetHeuristic(&h);
//	ida.GetPath(&cube, start, goal, path);
//	t.EndTimer();
//	printf("%1.5fs elapsed\n", t.GetElapsedTime());
//	printf("%llu nodes expanded %llu touched (%1.3f nodes/sec)\n", ida.GetNodesExpanded(), ida.GetNodesTouched(),
//		   ida.GetNodesExpanded()/t.GetElapsedTime());
}

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

std::mutex resultLock;

void ParallelExpand(int myID, int totalThreads,
					const std::unordered_set<RubiksState> &toExpand,
					const std::unordered_set<RubiksState> &duplicates,
					std::unordered_set<RubiksState> &results)
{
	RubiksCube cube;
	std::vector<RubiksAction> acts;
	std::vector<RubiksState> cache;

	uint64_t counter = 0;
	for (auto state : toExpand)
	{
		counter++;
		if (myID != counter%totalThreads)
			continue;
		cube.GetActions(state, acts);
		for (auto act : acts)
		{
			cube.ApplyAction(state, act);
			if ((duplicates.find(state) == duplicates.end()) && (toExpand.find(state) == toExpand.end()))
			{
				cache.push_back(state);
			}
			cube.UndoAction(state, act);
		}
		if (cache.size() > 4096)
		{
			resultLock.lock();
			for (auto &s : cache)
				results.insert(s);
			resultLock.unlock();
			cache.clear();
		}
	}
	resultLock.lock();
	for (auto &s : cache)
		results.insert(s);
	resultLock.unlock();
	cache.clear();
}

void RegionAnalysis()
{
	uint32_t bfsSize[] = {1, 18, 243, 3240, 43239, 574908, 7618438, 100803036, 1332343288};
	if (!fileExists("rubik1997.bin"))
		KorfTest(0, true);
	if (!fileExists("rubik888.bin"))
		KorfTest(0, false);

	RubiksState s;

	
	const int bfsDepth = 7;
//	FILE *f = fopen("/Users/nathanst/rubik1997.bin", "w+");
//	FILE *f = fopen("/Users/nathanst/rubik888.bin", "w+");
//	FILE *f = fopen("/Users/nathanst/rubik1997-f0.bin", "w+");
	std::vector<std::unordered_set<RubiksState> > forward;
	std::vector<std::unordered_set<RubiksState> > reverse;
	Timer t;
	std::vector<RubiksAction> acts;
	RubiksCube cube;
	RubiksState goal;
	RubiksState start;
	GetInstance(start, 0);
	
	reverse.resize(bfsDepth+1);
	forward.resize(bfsDepth+1);
	for (int x = 0; x <= bfsDepth; x++)
	{
		reverse[x].reserve(bfsSize[x]);
		forward[x].reserve(bfsSize[x]);
	}
	
	reverse[0].insert(goal);
	forward[0].insert(start);
	int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread*> threads(numThreads);
	printf("Using %d threads\n", numThreads);
	for (int x = 1; x <= bfsDepth; x++)
	{
		printf("Starting on depth %d\n", x);
		t.StartTimer();
		for (int t = 0; t < numThreads; t++)
		{
			if (x == 1)
			{
				threads[t] = new std::thread(ParallelExpand, t, numThreads,
											 std::ref(reverse[x-1]), std::ref(reverse[x-1]), std::ref(reverse[x]));
			}
			else {
				threads[t] = new std::thread(ParallelExpand, t, numThreads,
											 std::ref(reverse[x-1]), std::ref(reverse[x-2]), std::ref(reverse[x]));
			}
		}
		for (int t = 0; t < numThreads; t++)
		{
			threads[t]->join();
		}
		printf("%lu total states (%1.2fs seconds)\n", reverse[x].size(), t.EndTimer());
	}
	for (int x = 1; x <= bfsDepth; x++)
	{
		t.StartTimer();
		printf("Starting on depth %d\n", x);
		for (int t = 0; t < numThreads; t++)
		{
			if (x == 1)
			{
				threads[t] = new std::thread(ParallelExpand, t, numThreads,
											 std::ref(forward[x-1]), std::ref(forward[x-1]), std::ref(forward[x]));
			}
			else {
				threads[t] = new std::thread(ParallelExpand, t, numThreads,
											 std::ref(forward[x-1]), std::ref(forward[x-2]), std::ref(forward[x]));
			}
		}
		for (int t = 0; t < numThreads; t++)
		{
			threads[t]->join();
		}
		printf("%lu total states (%1.2fs seconds)\n", forward[x].size(), t.EndTimer());
	}

	for (int x = 0; x < 2; x++)
	{
		uint64_t NG = 0;
		uint64_t NS = 0;
		uint64_t NN = 0;
		uint64_t O = 0;
		
		FILE *f;
		if (x == 0)
			f = fopen("rubik888.bin", "r");
		if (x == 1)
			f = fopen("rubik1997.bin", "r");
		RubiksState s;
		while (!feof(f))
		{
			if (fread(&s, sizeof(s), 1, f) != 1)
				break;
			bool ns = false, ng = false;
			for (const auto &set : reverse)
			{
				if (set.find(s) != set.end())
				{
					ng = true;
				}
			}
			for (const auto &set : forward)
			{
				if (set.find(s) != set.end())
				{
					ns = true;
				}
			}
			if (ng && ns)
				NN++;
			else if (ng)
				NG++;
			else if (ns)
				NS++;
			else
				O++;
		}
		printf("NN: %llu NG: %llu NS: %llu O: %llu\n", NN, NG, NS, O);
		fclose(f);
	}
}

void KorfTest(int which, bool use1997heuristic)
{
	RubiksCube cube;
	RubiksState goal;
	RubiksState start;

	std::vector<int> edges1;
	std::vector<int> edges2;
	std::vector<int> corners;
	
	if (use1997heuristic)
	{
		edges1 = {1, 3, 8, 9, 10, 11};
		edges2 = {0, 2, 4, 5, 6, 7};
		corners = {0, 1, 2, 3, 4, 5, 6, 7};
	}
	else {
		edges1 = {0, 1, 2, 3, 4, 5, 6, 7};
		edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
		corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
	}

	std::vector<int> blank;
	RubikPDB pdb1(&cube, goal, edges1, blank);
	RubikPDB pdb2(&cube, goal, edges2, blank);
	RubikPDB pdb3(&cube, goal, blank, corners);
	if (!pdb1.Load(pdbLocation))
	{
		goal.Reset();
		pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1.Save(pdbLocation);
	}
	if (!pdb2.Load(pdbLocation))
	{
		goal.Reset();
		pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2.Save(pdbLocation);
	}
	if (!pdb3.Load(pdbLocation))
	{
		goal.Reset();
		pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3.Save(pdbLocation);
	}
	Heuristic<RubiksState> h;
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 0});
	h.lookups.push_back({kLeafNode, 2, 0});
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&pdb3);
	
	FILE *f;
#ifdef DO_LOGGING
	if (use1997heuristic)
		f = fopen("rubik1997.bin", "w+");
	else
		f = fopen("rubik888.bin", "w+");
#endif
	
	cube.SetPruneSuccessors(true);
	Timer t;
	t.StartTimer();
	//GetInstance(start, which);
	GetKorfInstance(start, which);
	goal.Reset();
	std::vector<RubiksAction> path;
	IDAStar<RubiksState, RubiksAction> ida;
	ida.SetHeuristic(&h);
#ifdef DO_LOGGING
	ida.func = [cube, f](RubiksState nextState, int depth)
	{
		fwrite(&nextState, sizeof(nextState), 1, f);
	};
#endif
	
	ida.GetPath(&cube, start, goal, path);
	t.EndTimer();
#ifdef DO_LOGGING
	fclose(f);
#endif
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
		   ida.GetNodesExpanded()/t.GetElapsedTime());
	printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
		   ida.GetNodesTouched()/t.GetElapsedTime());


	t.StartTimer();
	
	//GetInstance(start, which);
	GetKorfInstance(start, which);
	goal.Reset();
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida;
	pida.SetHeuristic(&h);
	pida.GetPath(&cube, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida.GetNodesExpanded(),
		   pida.GetNodesExpanded()/t.GetElapsedTime());
	printf("%llu nodes generated (%1.3f nodes/sec)\n", pida.GetNodesTouched(),
		   pida.GetNodesTouched()/t.GetElapsedTime());
}

void KorfAll()
{
	RubiksCube cube;
	RubiksState goal;
	RubiksState start;
	std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7};//{1, 3, 8, 9, 10, 11};
	std::vector<int> edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
	std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> blank;
	RubikPDB pdb1(&cube, goal, edges1, blank);
	RubikPDB pdb2(&cube, goal, edges2, blank);
	RubikPDB pdb3(&cube, goal, blank, corners);
	goal.Reset();
	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
	goal.Reset();
	pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
	goal.Reset();
	pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
	Heuristic<RubiksState> h;
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 0});
	h.lookups.push_back({kLeafNode, 2, 0});
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&pdb3);
	
	for (int which = 9; which < 10; which++)
	{
		printf("Korf instance %d\n", which+1);
		cube.SetPruneSuccessors(true);
		Timer t;
		t.StartTimer();
		//GetInstance(start, which);
		GetKorfInstance(start, which);
		goal.Reset();
		std::vector<RubiksAction> path;
//		IDAStar<RubiksState, RubiksAction> ida;
//		ida.SetHeuristic(&h);
//		//ida.GetPath(&cube, start, goal, path);
//		t.EndTimer();
//		printf("%1.5fs elapsed\n", t.GetElapsedTime());
//		printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
//			   ida.GetNodesExpanded()/t.GetElapsedTime());
//		printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
//			   ida.GetNodesTouched()/t.GetElapsedTime());
		
		
		t.StartTimer();
		
		//	GetInstance(start, which);
		//	cube.ApplyAction(start, 7);
		//	cube.ApplyAction(start, 2);
		//	printf("After applying actions 7, 2, heuristic is %1.2f\n", h.HCost(start, goal));
		
		//GetInstance(start, which);
		GetKorfInstance(start, which);
		goal.Reset();
		ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida;
		pida.SetHeuristic(&h);
		pida.GetPath(&cube, start, goal, path);
		t.EndTimer();
		printf("%1.5fs elapsed\n", t.GetElapsedTime());
		printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida.GetNodesExpanded(),
			   pida.GetNodesExpanded()/t.GetElapsedTime());
		printf("%llu nodes generated (%1.3f nodes/sec)\n", pida.GetNodesTouched(),
			   pida.GetNodesTouched()/t.GetElapsedTime());
	}
}

void RunTest(int billionEntriesToLoad)
{
//	uint64_t numEntries = 1000000000;
//	numEntries *= billionEntriesToLoad;
//	LoadCornerPDB();
//	//LoadEdgePDB(numEntries);
//	LoadEdge7PDB();
//	
//	for (int t = 100; t >= 80; t--)
//	{
//		for (int x = 0; x < 100; x++)
//		{
//			srandom(9283+x*23);
//			SolveOneProblem(x, t);
//		}
//	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

int GetNextMove(const char *input, int &base);
const char *GetStringFromMove(int move)
{
	const char *str[] = {"U", "D", "L", "R", "B", "F", "U-", "D-", "L-", "R-", "B-", "F-", "U2", "D2", "L2", "R2", "B2", "F2"};
	for (int x = 0; x < 18; x++)
	{
		int act;
		GetNextMove(str[x], act);
		if (act == move)
			return str[x];
	}
	return "?";
}

int GetNextMove(const char *input, int &base)
{
	int used = 0;
	if (isdigit(input[0])) // this is our move notation - numeric
	{
		int curr = 0;
		base = input[curr]-'0';
		curr++;
		if (isdigit(input[curr]))
		{
			base = base*10+input[curr]-'0';
			curr++;
		}
		while (!isdigit(input[curr]) && input[curr] != '\n' && input[curr] != 0)
		{
			curr++;
		}
		return curr;
	}
	switch (input[0])
	{
		case 'F': used = 1; base = 2*3; break;
		case 'B': used = 1; base = 3*3; break;
		case 'L': used = 1; base = 4*3; break;
		case 'R': used = 1; base = 5*3; break;
		case 'U': used = 1; base = 0*3; break;
		case 'D': used = 1; base = 1*3; break;
		default: break;
	}
	if (used == 0)
		return 0;
	if (input[0] != 'U')
	{
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 2; used++; break;
				case '-': base += 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	else {
		base++;
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 1; used++; break;
				case '-': base -= 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	return used;
}

void GetActionsFromStdin(std::vector<RubiksAction> &acts)
{
	const int maxStrLength = 1024;
	char string[maxStrLength];
	char *result = fgets(string, maxStrLength, stdin);
	if (result == 0)
	{
		printf("No entries found; exiting.\n");
		exit(0);
	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		acts.push_back(act);
	}
}

void GetKorfInstance(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	assert(which >= 0 && which < 10);
	RubiksCube c;
	char instances[10][maxStrLength] =
	{
		"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2",
		"B- R2 B  D  B- L  B  L2 F2 R F2 D- L2 U2 L- U  L- U2 B- L- R- U  D  L- B2 D  R- U  F  D2 F  B  U  B2 L2 D2 R- B2 L- R2 U2 D2 F2 D  R2 D2 B- U- D  F- R  B2 D  R2 F  L- B  L2 R- U2 L  F2 B- D- F- B- L2 D  B2 U- D  F2 U  L2 D  L- D- R2 D- B2 U- L2 U  B- L- U- F- L- R- B- U- R  B2 U2 B  R- B- R2 F  R-",
		"L- R  F- L  R2 F2 D- L2 D  B2 R2 D- F- L- F  R  F2 U  L- B2 D- R- U- R  D  F  R  D  B2 U- F- L2 F- B  U- R  F- U  F  D- L2 R- F- B  L2 B2 D- R- B  L B  D- R  U- R2 D2 F  R  U2 B2 D2 B  R- F- L2 D2 L2 R  D  L- B2 U  F2 R  F  L  U  D  L- B2 L2 B2 D- L  D2 B- U- B- U2 B L  D  B- L- U2 L- R  D- R  B2",
		"L- B2 U2 R- D  F  U  F2 D- F U  D- R- B  R  U- R2 B  R  F D  R2 F- R- B2 R- D- R2 U- F- R  D  F- R  U- F  B  U- D2 B- L  D2 L2 B- U2 L2 F2 L  D2 B D- L  R2 B2 U2 F2 B- U2 F  D2 L2 U2 F2 L  R- U- R- D  F  L2 F2 L- R  U- L2 U  R  D  F  R- F- D- L- R2 U2 F  R- B2 D  B2 D2 B2 R  F- L- D  B- U  L2 B-",
		"B2 L- F- U2 R2 D  R2 B  U2 R B- L  R  F  R- D- R- D2 F2 R2 B  L2 D- B2 D2 L  F- R  B- R2 B2 D  B- U  R- D- L2 B2 L2 R2 B- U2 D- R2 B  U- B- R- D  L B- L  R2 D- B  L2 D2 F2 B- U2 B  D- F- B  L2 U  F- U  F- L U  R  U2 D- B- U2 D  F- L  F2 B2 L  U  B- D2 B2 L- D- L2 B- D2 F  U2 R  D2 L2 D  B- L2 R-",
		"B- R2 B  R  U- D- R2 D  B  L2 B2 U- F- B  D2 R- U  F  L  F U2 B  L  D- L- R2 B  D  R- F- B- D  L- B- L2 R- U2 D  B  R- D2 B- U2 B- L- R2 F  L  U  L- R  B- R2 D2 R2 B- U2 F- U  L D- F  B- R2 D2 L  B  L  U2 B2 R  D- B- R2 B2 U  F2 R2 U- B- R2 F2 U2 F2 B- D  F  U- F2 B R  D  R- U- L- R  U2 D- R2 F",
		"L- R- B  L- D- L  U2 B2 D- F2 D  B2 R- F- R  U2 B2 D2 L- B- D  R2 D2 R  D- R2 F- R  U  B- U2 B2 D- R- D2 F  U  D- F- U2 L2 U  F2 R- B- U- L  B- R- U- L- U2 B2 D  B- R  B  D  B2 R- U2 D- R- F  L- F- D- B  L2 R- B  R- B- D  R  U- R2 B- D2 F- R  F  L- U  L2 B2 R2 F- U- F D  B- L- F2 B  L2 U2 D- L2 B2",
		"U  F  B2 L  F2 L- D2 B2 L2 U- R  D2 L2 U- D  F  U2 L2 B- U- B- R2 B2 R  U  R2 D- B2 R  B2 L2 U- R  D  L- R2 U2 R- D  B L2 R- B2 U2 L  U2 R  F2 U  B2 L2 R2 D- F  R2 D- L2 U- R  B2 D2 R- U2 L- B- L- F- L  R  F L- B- D  L- R2 F2 U- F  L- U2 B  U  R2 F  U  R- B  D  B  R- B- L- B2 R- F  L- B2 L- F2 D2",
		"L  F2 L2 B2 D  B  R  D- R- F2 U2 D2 F2 B- U- L- R2 B  U2 R F- D2 B2 U- R- F  L- U2 R- B L2 R2 B2 L2 R- B- D  F2 L  U2 D  L- B2 L  D- R2 D  B2 U  R- F2 U- R  F2 R2 B  U2 D2 R- U2 L2 R- D2 F2 R2 B- U  B  U  F D- F  D2 R- B- U2 B2 L  D  B- L- B2 D  B2 D- B  D- F- B- L- U  B2 D2 B- D- F  D- L  F  R",
		"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2"
	};
	
	char *string = instances[which];
	start.Reset();
	
	//	if (result == 0)
	//	{
	//		printf("No more entries found; exiting.\n");
	//		exit(0);
	//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}


void GetInstanceFromStdin(RubiksState &start)
{
	const int maxStrLength = 1024;
	char string[maxStrLength];
	char *result = fgets(string, maxStrLength, stdin);

	start.Reset();
	
	if (result == 0)
	{
		printf("No more entries found; exiting.\n");
		exit(0);
	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetInstance(RubiksState &start, int which)
{
	int instances[100][14] = {
	{4, 10, 5, 10, 17, 6, 1, 11, 0, 10, 16, 9, 14, 4},
	{15, 0, 12, 16, 1, 6, 17, 8, 17, 8, 2, 11, 4, 15},
	{6, 12, 2, 16, 9, 13, 8, 2, 15, 11, 4, 8, 4, 16},
	{9, 13, 16, 1, 14, 0, 11, 13, 4, 10, 12, 6, 5, 15},
	{1, 9, 4, 6, 0, 4, 15, 5, 12, 0, 14, 9, 4, 16},
	{5, 8, 4, 8, 5, 9, 5, 11, 0, 7, 11, 12, 8, 17},
	{14, 11, 4, 14, 5, 6, 4, 10, 1, 4, 15, 9, 13, 7},
	{17, 1, 4, 10, 4, 16, 4, 16, 8, 13, 15, 2, 4, 17},
	{7, 12, 11, 4, 15, 1, 17, 2, 5, 14, 9, 5, 8, 10},
	{10, 13, 15, 1, 16, 3, 7, 9, 17, 4, 9, 17, 6, 15},
	{0, 5, 10, 16, 8, 14, 7, 2, 6, 1, 4, 12, 8, 13},
	{4, 6, 5, 17, 7, 4, 17, 9, 17, 3, 9, 3, 17, 3},
	{15, 7, 13, 17, 11, 3, 10, 4, 12, 10, 0, 16, 2, 9},
	{6, 10, 16, 2, 4, 7, 0, 14, 16, 9, 2, 17, 2, 9},
	{9, 16, 0, 5, 16, 7, 1, 6, 0, 3, 10, 2, 12, 11},
	{1, 7, 3, 10, 2, 10, 2, 8, 4, 11, 13, 17, 5, 7},
	{5, 11, 16, 4, 9, 13, 4, 15, 8, 15, 0, 11, 12, 7},
	{14, 17, 11, 4, 8, 3, 10, 4, 10, 0, 5, 7, 13, 3},
	{17, 8, 16, 11, 14, 5, 13, 2, 16, 10, 3, 12, 0, 5},
	{8, 9, 2, 3, 11, 15, 0, 17, 9, 2, 13, 10, 3, 17},
	{10, 17, 2, 17, 10, 2, 11, 16, 8, 16, 10, 5, 16, 1},
	{0, 6, 17, 1, 15, 9, 4, 15, 3, 14, 1, 11, 1, 3},
	{4, 7, 17, 9, 17, 9, 0, 9, 17, 3, 13, 5, 15, 11},
	{13, 15, 1, 5, 16, 6, 15, 6, 1, 10, 2, 14, 3, 7},
	{6, 16, 1, 16, 3, 9, 1, 13, 8, 12, 7, 4, 9, 14},
	{9, 13, 2, 5, 16, 5, 9, 17, 7, 4, 6, 9, 14, 3},
	{1, 13, 11, 5, 15, 3, 7, 12, 5, 7, 12, 5, 14, 0},
	{5, 17, 10, 4, 9, 4, 13, 1, 5, 6, 0, 6, 3, 17},
	{14, 8, 10, 13, 11, 12, 6, 11, 0, 5, 12, 0, 14, 1},
	{17, 10, 0, 11, 16, 9, 5, 6, 17, 8, 3, 11, 13, 5},
	{8, 17, 10, 15, 4, 8, 0, 15, 1, 16, 10, 13, 15, 6},
	{10, 14, 15, 6, 13, 5, 13, 0, 3, 11, 5, 7, 5, 6},
	{0, 12, 11, 17, 9, 1, 11, 16, 7, 13, 5, 14, 17, 5},
	{4, 15, 11, 3, 17, 7, 13, 6, 17, 7, 12, 0, 5, 6},
	{13, 16, 2, 3, 11, 14, 2, 11, 17, 7, 2, 13, 8, 1},
	{6, 12, 3, 14, 8, 5, 12, 5, 17, 4, 14, 1, 12, 17},
	{9, 13, 7, 9, 5, 7, 2, 3, 6, 2, 15, 2, 13, 1},
	{1, 17, 7, 9, 14, 0, 3, 11, 15, 10, 0, 4, 15, 5},
	{3, 9, 2, 17, 5, 10, 17, 10, 1, 12, 9, 3, 8, 4},
	{14, 4, 15, 11, 5, 15, 11, 0, 14, 10, 1, 10, 12, 11},
	{17, 4, 11, 13, 4, 14, 9, 13, 11, 17, 1, 6, 16, 0},
	{8, 5, 8, 15, 6, 4, 15, 11, 14, 7, 4, 7, 4, 9},
	{10, 14, 8, 3, 15, 6, 5, 17, 3, 12, 11, 16, 6, 2},
	{0, 15, 3, 17, 0, 9, 3, 14, 16, 2, 9, 14, 0, 4},
	{4, 16, 5, 7, 2, 9, 4, 12, 6, 17, 2, 13, 7, 14},
	{13, 6, 9, 3, 16, 8, 9, 13, 15, 4, 12, 0, 15, 3},
	{6, 13, 6, 5, 7, 4, 11, 15, 10, 13, 3, 8, 10, 2},
	{9, 16, 0, 6, 1, 9, 12, 7, 9, 5, 16, 5, 15, 4},
	{1, 4, 9, 3, 8, 4, 8, 4, 6, 14, 5, 14, 16, 3},
	{3, 14, 17, 11, 13, 4, 8, 15, 6, 15, 8, 4, 10, 2},
	{14, 17, 0, 13, 9, 3, 17, 10, 3, 8, 3, 17, 3, 16},
	{17, 5, 8, 15, 2, 9, 3, 17, 6, 17, 0, 4, 13, 4},
	{8, 14, 2, 17, 7, 10, 5, 13, 0, 9, 16, 7, 17, 0},
	{11, 15, 2, 11, 17, 11, 4, 17, 1, 11, 12, 3, 14, 1},
	{0, 3, 15, 0, 7, 10, 3, 12, 6, 17, 9, 0, 7, 1},
	{4, 12, 15, 7, 3, 7, 15, 7, 13, 3, 15, 11, 5, 10},
	{13, 1, 6, 11, 16, 3, 14, 5, 16, 1, 9, 0, 16, 0},
	{6, 3, 7, 2, 15, 6, 2, 15, 6, 11, 17, 7, 0, 16},
	{9, 4, 13, 9, 5, 13, 11, 17, 9, 15, 11, 3, 14, 9},
	{1, 14, 9, 14, 4, 9, 1, 16, 10, 16, 11, 13, 17, 1},
	{3, 11, 13, 15, 9, 16, 8, 13, 10, 15, 0, 11, 5, 15},
	{14, 5, 11, 0, 13, 6, 1, 11, 3, 9, 14, 8, 10, 12},
	{17, 2, 9, 14, 11, 12, 10, 4, 9, 14, 0, 5, 17, 6},
	{8, 2, 14, 10, 0, 17, 7, 5, 8, 12, 7, 12, 10, 3},
	{11, 3, 14, 7, 0, 6, 13, 0, 6, 15, 5, 7, 3, 12},
	{0, 12, 9, 3, 8, 9, 14, 15, 0, 4, 6, 5, 10, 17},
	{4, 9, 16, 8, 1, 4, 12, 4, 15, 2, 4, 6, 3, 15},
	{13, 3, 12, 1, 7, 0, 5, 8, 17, 10, 17, 5, 9, 17},
	{16, 4, 12, 9, 4, 13, 9, 1, 8, 2, 3, 12, 10, 0},
	{9, 1, 7, 4, 14, 2, 13, 8, 4, 13, 5, 11, 13, 10},
	{1, 7, 13, 16, 10, 16, 8, 5, 11, 0, 14, 8, 5, 7},
	{3, 15, 8, 13, 3, 10, 3, 12, 16, 11, 17, 0, 5, 12},
	{14, 10, 5, 10, 2, 13, 17, 7, 16, 7, 0, 3, 8, 0},
	{17, 2, 8, 12, 2, 16, 5, 10, 0, 17, 5, 16, 10, 3},
	{8, 5, 8, 0, 7, 3, 12, 10, 12, 6, 17, 3, 16, 7},
	{11, 17, 6, 11, 2, 6, 4, 14, 0, 11, 17, 0, 6, 13},
	{0, 3, 8, 2, 9, 17, 5, 7, 12, 8, 0, 14, 4, 16},
	{4, 14, 7, 5, 11, 13, 4, 6, 1, 15, 1, 13, 1, 7},
	{13, 9, 16, 7, 12, 9, 4, 12, 8, 15, 1, 4, 11, 17},
	{16, 1, 6, 14, 15, 8, 16, 4, 11, 13, 2, 7, 1, 3},
	{9, 0, 14, 5, 6, 1, 15, 2, 12, 4, 7, 17, 10, 16},
	{1, 7, 0, 13, 1, 14, 0, 14, 7, 13, 9, 0, 6, 1},
	{3, 10, 17, 11, 1, 15, 2, 8, 11, 16, 5, 14, 16, 7},
	{12, 2, 17, 8, 2, 6, 10, 0, 12, 6, 12, 0, 11, 12},
	{17, 7, 1, 6, 9, 15, 9, 5, 9, 14, 15, 7, 10, 3},
	{8, 11, 2, 17, 6, 1, 7, 11, 3, 6, 10, 16, 3, 14},
	{11, 1, 14, 8, 0, 6, 5, 8, 2, 4, 14, 9, 3, 17},
	{2, 8, 15, 7, 4, 16, 8, 5, 16, 0, 10, 13, 2, 6},
	{4, 9, 15, 5, 12, 1, 15, 1, 5, 10, 5, 13, 5, 9},
	{13, 1, 10, 16, 3, 13, 9, 15, 0, 14, 3, 6, 16, 8},
	{16, 6, 10, 13, 4, 12, 3, 14, 11, 15, 0, 9, 12, 17},
	{9, 13, 9, 1, 13, 4, 17, 6, 4, 16, 4, 14, 1, 10},
	{1, 10, 13, 4, 16, 8, 10, 17, 5, 13, 8, 0, 15, 9},
	{3, 16, 11, 2, 8, 16, 4, 9, 5, 16, 11, 16, 5, 12},
	{12, 7, 11, 15, 11, 17, 6, 0, 16, 0, 17, 9, 15, 4},
	{17, 11, 12, 10, 2, 13, 2, 7, 2, 12, 4, 16, 0, 7},
	{8, 17, 8, 5, 14, 11, 14, 17, 8, 11, 1, 12, 7, 4},
	{11, 14, 8, 4, 11, 4, 7, 5, 9, 12, 10, 1, 14, 16},
	{2, 9, 16, 2, 5, 14, 10, 16, 2, 11, 0, 5, 6, 2},
	{4, 17, 9, 1, 12, 5, 17, 8, 11, 13, 15, 6, 12, 4}
	};

	start.Reset();
	for (int x = 13; x >= 0; x--)
	{
		c.UndoAction(start, instances[which][x]);
		printf("%d ", instances[which][x]);
	}
	printf("\n");
}

void MeasurePDBCompression(int itemsCompressed)
{
	std::vector<bucketInfo> data;
	std::vector<bucketData> buckets;
	
	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
	int64_t totalSize = 0;

	//DiskBitFile f("/data/cc/rubik/final/RC");
	DiskBitFile f("/store/rubik/RC");
	int64_t index = 0;
	int64_t records[20];
	int64_t averageFirst[20];
	int64_t averageMin[20];
	int64_t averageMax[20];
	int64_t averageValue[20];

	int tempSum[20];
	int firstVal[20];
	int minVal[20];
	int maxVal[20];
	for (int x = 0; x < 20; x++)
	{
		records[x] = 0;
		averageFirst[x] = 0;
		averageMin[x] = 0;
		averageMax[x] = 0;
		averageValue[x] = 0;

		tempSum[x] = 0;
		firstVal[x] = 0;
		minVal[x] = 0;
		maxVal[x] = 0;
	}
	printf("Measuring PDB compression stats!\n");
	fflush(stdout);
	for (unsigned int x = 0; x < data.size(); x++)
    {
		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
		{
			int val = f.ReadFileDepth(data[x].bucketID, y);

			for (int i = 0; i < 20; i++)
			{
				// reset computation
				if (0 == index%(i+1))
				{
					// record results
					if (x != 0 || y != 0)
					{
						records[i]++;
						averageFirst[i] += firstVal[i];
						averageMin[i] += minVal[i];
						averageMax[i] += maxVal[i];
						averageValue[i] += tempSum[i];
					}
					
					tempSum[i] = val;
					firstVal[i] = val;
					minVal[i] = val;
					maxVal[i] = val;
				}
				else {
					tempSum[i] += val;
					if (val < minVal[i])
						minVal[i] = val;
					if (val > maxVal[i])
						maxVal[i] = val;
				}
			}

			index++;
			if (0 == index%100000000)
			{
				for (int i = 0; i < 20; i++)
				{
					printf("---- %d ---\n", i+1);
					printf("%lld processed (%1.2fGB); %lld records total:\n", index, index/1024.0/1024.0/1024.0/2.0, records[i]);
					printf(": Average first: %1.4f\n", (float)averageFirst[i]/records[i]);
					printf(": Average min: %1.4f\n", (float)averageMin[i]/records[i]);
					printf(": Average max: %1.4f\n", (float)averageMax[i]/records[i]);
					printf(": Average value: %1.4f\n", (float)averageValue[i]/records[i]/(i+1));
				}
				fflush(stdout);
			}
		}
    }

}

void ExtractStatesAtDepth(const char *theFile)
{
//	RubikEdge e;
//	RubikEdgeState es;
//
//	std::vector<std::vector<uint64_t> > vals;
//	std::vector<FILE *> files;
//	std::vector<long> counts;
//	vals.resize(10); // 0...9
//	counts.resize(10);
//	std::vector<bucketInfo> data;
//	std::vector<bucketData> buckets;
//	
//	uint64_t maxBuckSize = GetMaxBucketSize<RubikEdge, RubikEdgeState>(true);
//	InitTwoPieceData<RubikEdge, RubikEdgeState>(data, maxBuckSize);
//	InitBucketSize<RubikEdge, RubikEdgeState>(buckets, maxBuckSize);
//	
//	DiskBitFile f(theFile);
//	
//	uint64_t entry = 0;
//	for (int x = 0; x < 10; x++)
//	{
//		char name[255];
//		sprintf(name, "12edge-depth-%d.dat", x);
//		FILE *f = fopen(name, "w+");
//		if (f == 0)
//		{
//			printf("Error opening %s; aborting!\n", name);
//			exit(0);
//		}
//		files.push_back(f);
//	}
//	for (unsigned int x = 0; x < data.size(); x++)
//	{
//		for (int64_t y = data[x].bucketOffset; y < data[x].bucketOffset+data[x].numEntries; y++)
//		{
//			bool debug = false;
//			int val = f.ReadFileDepth(data[x].bucketID, y);
//			if (val < 10)
//			{
//				int64_t r1, r2;
//				e.unrankPlayer(entry, es, 0);
//				e.rankPlayer(es, 0, r1, r2);
//				if (r1 != x || r2 != y-data[x].bucketOffset)
//				{
//					printf("Error: entry is %llu, which should map to x: %llu y: %llu. Instead we got r1: %llu, r2: %llu\n", entry, x, y-data[x].bucketOffset, r1, r2);
//					exit(0);
//				}
//				vals[val].push_back(es.state);
//				counts[val]++;
//			}
//			entry++;
//			if (0 == entry%1000000000)
//			{
//				printf("Processing entry %llu\n", entry);
//				fflush(stdout);
//				printf("Flushing files:\n"); fflush(stdout);
//				for (int t = 0; t < 10; t++)
//				{
//					printf("File %d, %ld entries\n", t, vals[t].size()); fflush(stdout);
//					fwrite(&(vals[t][0]), sizeof(uint64_t), vals[t].size(), files[t]);
//					vals[t].resize(0);
//				}
//			}
//		}
//	}
//	printf("%llu entries read total\n", entry);
//	{
//		printf("Processing entry %llu\n", entry); fflush(stdout);
//		printf("Flushing files:\n"); fflush(stdout);
//		for (int x = 0; x < 10; x++)
//		{
//			printf("File %d, %ld entries\n", x, vals[x].size()); fflush(stdout);
//			fwrite(&(vals[x][0]), sizeof(uint64_t), vals[x].size(), files[x]);
//			vals[x].resize(0);
//		}
//	}
//	for (int x = 0; x < 10; x++)
//	{
//		printf("Total entries for file %d: %lu\n", x, counts[x]);
//		fclose(files[x]);
//	}
}

void TestBloom(int entries, double accuracy)
{
	bloom_parameters parameters;
	
	// How many elements roughly do we expect to insert?
	parameters.projected_element_count = entries;//1000;
	
	// Maximum tolerable false positive probability? (0,1)
	parameters.false_positive_probability = accuracy/100.0;//0.0001; // 1 in 10000
	
	// Simple randomizer (optional)
	parameters.random_seed = 0xA5A5A5A5;
	
	if (!parameters)
	{
		std::cout << "Error - Invalid set of bloom filter parameters!" << std::endl;
		return;
	}
	
	parameters.compute_optimal_parameters();
	
	//Instantiate Bloom Filter
	bloom_filter filter(parameters);
	
	printf("Approximate storage: %lld bits (%1.2f MB / %1.2f GB)\n", parameters.optimal_parameters.table_size,
		   parameters.optimal_parameters.table_size/8.0/1024.0/1024.0,
		   parameters.optimal_parameters.table_size/8.0/1024.0/1024.0/1024.0);

	Timer t;
	t.StartTimer();
	// Insert into Bloom Filter
	RubikEdge e;
	RubikEdgeState s;
	{
		// Insert some numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			filter.insert(i); // s.state
		}
	}
	t.EndTimer();
	printf("%1.6f spend adding entries (%lu bytes each)\n", t.GetElapsedTime(), sizeof(std::size_t));
	
	// Query Bloom Filter
	{
		t.StartTimer();
		int correct = 0;
		int incorrect = 0;
		// Query the existence of numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			if (filter.contains(i)) //s.state
			{
				//std::cout << "BF contains: " << i << std::endl;
				correct++;
			}
		}
		
		srandom(12345);
		// Query the existence of invalid numbers
		for (int i = entries; i < 10*entries; ++i)
		{
			uint64_t next;
			do {
				next = random()%e.getMaxSinglePlayerRank();
			} while (next < entries);
			//e.unrankPlayer(next, s, 0);
			if (filter.contains(next)) // s.state
			{
				//std::cout << "BF falsely contains: " << i << std::endl;
				incorrect++;
			}
		}
		t.EndTimer();
		printf("%1.6f spend querying %d entries\n", t.GetElapsedTime(), entries*11);
		printf("%d of %d items (%5.2f%%) correctly stored. %d of %d items (%5.2f%%) incorrectly detected\n",
			   correct, entries, (100.0*correct/entries), incorrect, entries*10, incorrect*100.0/(entries*10.0));
	}
	return;

}
#include "Bloom.h"

void TestBloom2(int entries, double accuracy)
{
	BloomFilter filter(entries, accuracy/100.0, false);
	
	printf("Approximate storage: %lld bits (%1.2f MB / %1.2f GB)\n", filter.GetStorage(),
		   filter.GetStorage()/8.0/1024.0/1024.0,
		   filter.GetStorage()/8.0/1024.0/1024.0/1024.0);
	
	Timer t;
	t.StartTimer();
	// Insert into Bloom Filter
	RubikEdge e;
	RubikEdgeState s;
	{
		// Insert some numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			filter.Insert(i);
		}
	}
	t.EndTimer();
	printf("%1.6f spend adding entries (%lu bytes each)\n", t.GetElapsedTime(), sizeof(std::size_t));
	
	// Query Bloom Filter
	{
		t.StartTimer();
		int correct = 0;
		int incorrect = 0;
		// Query the existence of numbers
		for (std::size_t i = 0; i < entries; ++i)
		{
			//e.unrankPlayer(i, s, 0);
			if (filter.Contains(i))
			{
				//std::cout << "BF contains: " << i << std::endl;
				correct++;
			}
		}
		
		srandom(12345);
		// Query the existence of invalid numbers
		for (int i = entries; i < 10*entries; ++i)
		{
			uint64_t next;
			do {
				next = random()%e.getMaxSinglePlayerRank();
			} while (next < entries);
			//e.unrankPlayer(next, s, 0);
			if (filter.Contains(next))
			{
				//std::cout << "BF falsely contains: " << i << std::endl;
				incorrect++;
			}
		}
		t.EndTimer();
		printf("%1.6f spend querying %d entries\n", t.GetElapsedTime(), entries*11);
		printf("%d of %d items (%5.2f%%) correctly stored. %d of %d items (%5.2f%%) incorrectly detected\n",
			   correct, entries, (100.0*correct/entries), incorrect, entries*10, incorrect*100.0/(entries*10.0));
	}
	return;
	
}

void GetBloomStats(uint64_t size, int hash, const char *prefix)
{
	BloomFilter bf(size, hash, prefix);
	bf.Analyze();
	exit(0);
}

void TestMinBloom()
{
	int entries = 10000;
	int totalSpace = entries*80/4;
	{
		MinBloomFilter f4(totalSpace, 6, true);
		
		for (int x = 0; x < entries; x++)
			f4.Insert(x, x%15);
		
		printf("Testing correctness\n");
		for (int x = 0; x < entries; x++)
		{
			if (f4.Contains(x) != (x%15))
			{
				printf("For an insertion of %d, we got %d out instead of %d\n",
					   x, f4.Contains(x), (x%15));
			}
		}
		printf("Done!\n");
	
		int hits = 0;
		int total = 0;
		for (int x = entries+1; x < entries*1000; x++)
		{
			total++;
			if (f4.Contains(x) != 0xF)
			{
//				printf("For a false lookup of %d, we got out %d\n",
//					   x, f4.Contains(x));
				hits++;
			}
		}
		printf("%d bytes for %d entries. (%1.1fbits/entry)\n%d of %d hit; fp rate = %1.6f%%\n",
			   totalSpace/2, entries, float(8*totalSpace/2)/float(entries),
			   hits, total, 100*float(hits)/float(total));
	}
	
	{
		MinBloomFilter t4(totalSpace, 6, "");
		printf("Testing correctness\n");
		for (int x = 0; x < entries; x++)
		{
			if (t4.Contains(x) != (x%15))
			{
				printf("For an insertion of %d, we got %d out instead of %d\n",
					   x, t4.Contains(x), (x%15));
			}
		}
		printf("Done!\n");
	}
	
}

const int maxStrLength = 4024;
void GetSuperFlip(RubiksState &start)
{
	char string[maxStrLength] = "U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	
	start.Reset();
	
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void EnvTest()
{
	RubiksCube c1;
	RubiksCube c2;
	RubiksState s1;
	RubiksState s2;
	std::vector<RubiksAction> history, a1, a2;
	
	c1.SetPruneSuccessors(true);
	
	for (int t = 0; t < 1000; t++)
	{
		printf("Starting walk %d\n", t);
		for (int x = 0; x < 10000; x++)
		{
			c1.GetActions(s1, a1);
			if (history.size() > 0)
				c2.GetActions(s2, a2, history.back());
			else
				c2.GetActions(s2, a2);
			assert(a1.size() == a2.size());
			
			history.push_back(a1[random()%a1.size()]);
			c1.ApplyAction(s1, history.back());
			c2.ApplyAction(s2, history.back());
		}
		while (history.size() > 0)
		{
			c1.UndoAction(s1, history.back());
			c2.UndoAction(s2, history.back());
			history.pop_back();
		}
	}
	printf("Completed\n");
	exit(0);
}

void Test(int which)
{
	//EnvTest();
	RubiksCube cubeAStar;
	RubiksCube cubeIDAStar;
	RubiksState goal;
	RubiksState start;
	
	std::vector<int> edges1;
	std::vector<int> edges2;
	std::vector<int> corners;
	
	if (/*use1997heuristic*/true)
	{
		edges1 = {1, 3, 8, 9, 10, 11};
		edges2 = {0, 2, 4, 5, 6, 7};
		corners = {0, 1, 2, 3, 4, 5, 6, 7};
	}
	else {
		edges1 = {0, 1, 2, 3, 4, 5, 6, 7};
		edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
		corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
	}
	
	std::vector<int> blank;
	RubikPDB pdb1(&cubeAStar, goal, edges1, blank);
	RubikPDB pdb2(&cubeAStar, goal, edges2, blank);
	RubikPDB pdb3(&cubeAStar, goal, blank, corners);
	if (!pdb1.Load(pdbLocation))
	{
		goal.Reset();
		pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//		pdb1.Save(pdbLocation);
	}
	if (!pdb2.Load(pdbLocation))
	{
		goal.Reset();
		pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
//		pdb2.Save(pdbLocation);
	}
	if (!pdb3.Load(pdbLocation))
	{
		goal.Reset();
		pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
//		pdb3.Save(pdbLocation);
	}
	Heuristic<RubiksState> h;
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 0});
	h.lookups.push_back({kLeafNode, 2, 0});
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&pdb3);
		
	cubeAStar.SetPruneSuccessors(false);
	cubeIDAStar.SetPruneSuccessors(true);
	Timer t;
	std::vector<RubiksAction> path;

//	// 1. Test A*+IDA* (parallel)
//	t.StartTimer();
//	GetKorfInstance(start, which);
//	goal.Reset();
//	// TODO: Need to verify that there aren't nodes on OPEN that have been generated suboptimally
//	// So that searching from them results in wasted work
//	ParallelAStarIDAStar<RubiksCube, RubiksState, RubiksAction> a_ida(1000, std::thread::hardware_concurrency());
//	a_ida.SetHeuristic(&h);
//	
//	//	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida;
//	//	pida.SetHeuristic(&h);
//	a_ida.GetPath(&cubeAStar, &cubeIDAStar, start, goal, path);
//	t.EndTimer();
//	printf("%1.5fs elapsed\n", t.GetElapsedTime());
//	printf("%llu nodes expanded (%1.3f nodes/sec)\n", a_ida.GetNodesExpanded(),
//		   a_ida.GetNodesExpanded()/t.GetElapsedTime());
////	printf("%llu nodes generated (%1.3f nodes/sec)\n", a_ida.GetNodesTouched(),
////		   pida.GetNodesTouched()/t.GetElapsedTime());


	// 3. Test Parallel IDA*
	GetKorfInstance(start, which);
	goal.Reset();
	
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida1(1);
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida2(2);
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida4(4);
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> pida8(8);
	pida1.SetFinishAfterSolution(true);
	pida1.SetHeuristic(&h);
	pida2.SetFinishAfterSolution(true);
	pida2.SetHeuristic(&h);
	pida4.SetFinishAfterSolution(true);
	pida4.SetHeuristic(&h);
	pida8.SetFinishAfterSolution(true);
	pida8.SetHeuristic(&h);

	t.StartTimer();
	pida1.GetPath(&cubeIDAStar, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida1.GetNodesExpanded(),
		   pida1.GetNodesExpanded()/t.GetElapsedTime());
	t.StartTimer();
	pida2.GetPath(&cubeIDAStar, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida2.GetNodesExpanded(),
		   pida2.GetNodesExpanded()/t.GetElapsedTime());
	t.StartTimer();
	pida4.GetPath(&cubeIDAStar, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida4.GetNodesExpanded(),
		   pida4.GetNodesExpanded()/t.GetElapsedTime());
	t.StartTimer();
	pida8.GetPath(&cubeIDAStar, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", pida8.GetNodesExpanded(),
		   pida8.GetNodesExpanded()/t.GetElapsedTime());
//	printf("%llu nodes generated (%1.3f nodes/sec)\n", a_ida.GetNodesTouched(),
//		   pida.GetNodesTouched()/t.GetElapsedTime());

	
//	// 2. Test IDA*
//	t.StartTimer();
//	//GetInstance(start, which);
//	GetKorfInstance(start, which);
//	goal.Reset();
//	IDAStar<RubiksState, RubiksAction> ida;
//	ida.SetHeuristic(&h);
//	
//	ida.GetPath(&cubeIDAStar, start, goal, path);
//	t.EndTimer();
//	printf("%1.5fs elapsed\n", t.GetElapsedTime());
//	printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
//		   ida.GetNodesExpanded()/t.GetElapsedTime());
//	printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
//		   ida.GetNodesTouched()/t.GetElapsedTime());


	exit(0);
}
//{
//	printf("Testing\n");
//	exit(0);
//}
