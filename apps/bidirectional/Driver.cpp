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

#include <algorithm>
#include <random>
#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <vector>
#include "GUICode.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "MMRubik.h"
#include "MM0Rubik.h"
#include "ParallelIDAStar.h"
#include "BidirSTP.h"
#include "BidirPancake.h"
#include "BidirTOH.h"
#include "BidirTS.h"
#include "BidirRubik.h"
#include "MapGenerators.h"
#include "MNPuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "WeightedVertexGraph.h"
#include "PancakePuzzle.h"
#include "RubiksCubeCorners.h"


void Test100Easy();
void Animate();
void Test();

struct hash128
{
	uint32_t parent;
	uint32_t cornerHash;
	uint64_t edgeHash;
};

bool operator< (const hash128 &left, const hash128 &right)
{
	if (left.cornerHash != right.cornerHash)
		return (left.cornerHash < right.cornerHash);
	return left.edgeHash < right.edgeHash;
}

bool operator==(const hash128 &left, const hash128 &right)
{
	return (left.cornerHash == right.cornerHash && left.edgeHash == right.edgeHash);
}


namespace std {
	template <> struct hash<hash128>
	{
		size_t operator()(const hash128 & x) const
		{
			return x.edgeHash^(uint64_t(x.cornerHash)<<40);
		}
	};
}

void PDBTest();
void GetInstanceFromStdin(RubiksState &start);
void WriteStatesToDisk(std::vector<hash128> &states, int depth);
void ExpandLayer(int depth);
bool DuplicateDetectLayer(int depthToRemove);
void ClearFiles();
void TestPruning(int depth, int bucket);
const int kNumBuckets = 512;
void BFS();
void GetKorfRubikInstance(RubiksState &start, int which);
void GetSuperFlip(RubiksState &start);
void GetDepth20(RubiksState &start, int which);
void GetRandom15(RubiksState &start, int which);
void DualTest();
void ArbitraryGoalTest();
int MyCLHandler(char *argument[], int maxNumArgs);
void TestDataStructure();
const char *GetStringFromMove(int move);
void PathTest();

int main(int argc, char* argv[])
{
	PathTest();
	//TestDataStructure();
	setvbuf(stdout, NULL, _IONBF, 0);

	// technically we could/should install a command-line handler and handle these there
	InstallHandlers();
	InstallCommandLineHandler(MyCLHandler, "-mm", "-mm <tmpdir1 <tmpdir2>", "Run MM");
	InstallCommandLineHandler(MyCLHandler, "-pida", "-pida", "Run MM");
	InstallCommandLineHandler(MyCLHandler, "-grid", "-grid <map> <scenario> <hweight>", "MM/A* region analysis");
	InstallCommandLineHandler(MyCLHandler, "-nbs", "-nbs <map> <scenario> <hweight>", "NBS test");
	InstallCommandLineHandler(MyCLHandler, "-stp", "-stp <alg>", "A*/BS*/MM/NBS/MM0 test on 15 puzzle 100 korf instances");
	InstallCommandLineHandler(MyCLHandler, "-rubik", "-rubik <alg>", "A*/BS*/MM/NBS/MM0 test on rubik's cube");
	InstallCommandLineHandler(MyCLHandler, "-pancake", "-pancake", "NBS test on pancake");
	InstallCommandLineHandler(MyCLHandler, "-ts", "-ts", "NBS test on TopSpin");
	InstallCommandLineHandler(MyCLHandler, "-toh", "-toh", "NBS test on TOH");
	InstallCommandLineHandler(MyCLHandler, "-animate", "-animate", "Build animation");
	//const char *map, const char *scenario, double weight
	InstallCommandLineHandler(MyCLHandler, "-heuristic", "-heuristic <dir> <1997/888/8210/none>", "Load the given heuristic");
	InstallCommandLineHandler(MyCLHandler, "-problem", "-problem which", "Load the given problem");

	//2 8 1 3 17 0 4 8
	std::cout << GetStringFromMove(2) << " ";
	std::cout << GetStringFromMove(8) << " ";
	std::cout << GetStringFromMove(1) << " ";
	std::cout << GetStringFromMove(3) << " ";
	std::cout << GetStringFromMove(17) << " ";
	std::cout << GetStringFromMove(0) << " ";
	std::cout << GetStringFromMove(4) << " ";
	std::cout << GetStringFromMove(8) << "\n";

	RunHOGGUI(argc, argv);
}

#include "FixedSizeSet.h"

void TestDataStructure()
{
	// This tests the states
	RubikEdgeStateArray ea, ea2;
	RubikEdgeStateBits eb;
	RubikEdge edge;
	uint64_t count = 0;
	while (true)
	{
		count++;
		if (0 == count%1000000)
			std::cout << count << "\n";
		for (int x = 0; x < 12; x++)
		{
			if (ea.GetCubeInLoc(x) != eb.GetCubeInLoc(x) || ea.GetCubeOrientation(x) != eb.GetCubeOrientation(x))
			{
				std::cout << ea << "\n";
				std::cout << eb << "\n";
				exit(0);
			}
		}
		ea2 = ea;
		int x = random()%18;
		edge.UndoAction(ea2, x);
		edge.UndoAction(eb, x);
		ea = ea2;
	}
}
// This tests the hash table
//{
//	Timer t;
//	const int testSize = 10000000;
//	std::vector<int> elements;
//	for (int x = 0; x < testSize; x++)
//		elements.push_back(random());
//	//std::sort(elements.begin(), elements.end());
//
//	FixedSizeSet<int> test(testSize);
//	std::unordered_set<int> test2;
//
//	t.StartTimer();
//	for (int x = 0; x < testSize; x++)
//		test.insert(elements[x]);
//	t.EndTimer();
//	printf("%1.9fs inserting %d elements (mine)\n", t.GetElapsedTime(), testSize);
//	
//	t.StartTimer();
//	for (int x = 0; x < testSize; x++)
//		test2.insert(elements[x]);
//	t.EndTimer();
//	printf("%1.9fs inserting %d elements (std)\n", t.GetElapsedTime(), testSize);
//
//	// Reset data
////	{
////		elements.resize(0);
////		for (int x = 0; x < testSize; x++)
////		{
////			elements.push_back(random());
////		}
////	}
//	srandom(43);
//	t.StartTimer();
//	for (int x = 0; x < testSize*10; x++)
//	{
//		auto i = test.find(random());
//		if (i != test.end())
//			test.erase(i);
//	}
//	size_t cnt = 0, sum = 0;
//	for (auto &i : test)
//	{
//		if (i.valid)
//		{
//			cnt++;
//			sum+=i.item;
//		}
//	}
//	t.EndTimer();
//	printf("%1.9fs finding %d elements (mine)\n", t.GetElapsedTime(), testSize);
//	printf("%lu / %lu\n", cnt, sum);
//	
//	srandom(43);
//	t.StartTimer();
//	for (int x = 0; x < testSize*10; x++)
//	{
//		auto i = test2.find(random());
//		if (i != test2.end())
//			test2.erase(i);
//	}
//	cnt = 0; sum = 0;
//	for (auto &i : test2)
//	{
//		cnt++;
//		sum+=i;
//	}
//	t.EndTimer();
//	printf("%1.9fs finding %d elements (std)\n", t.GetElapsedTime(), testSize);
//	printf("%lu / %lu\n", cnt, sum);
//	
//	
//	
//	exit(0);
//}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	static RubiksState start, goal;
	static char *heuristicDir = 0;
	static bool heuristic = false;
	static bool problem = false;
	static MM::heuristicType h = MM::kNone;
	//	ArbitraryGoalTest();
	//	DualTest();

	if (strcmp(argument[0], "-heuristic") == 0) // not hooked up
	{
		if (maxNumArgs <= 2)
		{
			return 0;
		}
		heuristic = true;
		heuristicDir = argument[1];
		if (strcmp(argument[2], "1997") == 0)
			h = MM::k1997;
		else if (strcmp(argument[2], "888") == 0)
			h = MM::k888;
		else if (strcmp(argument[2], "none") == 0)
			h = MM::kNone;
		else if (strcmp(argument[2], "8210") == 0)
			h = MM::k8210;
		else if (strcmp(argument[2], "839") == 0)
			h = MM::k839;

		return 3;
	}
	else if (strcmp(argument[0], "-problem") == 0) // not hooked up
	{
		if (maxNumArgs <= 1)
		{
			return 0;
		}
		problem = true;
		RubiksCube c;
		goal.Reset();

		int which = 0;
		which = atoi(argument[1]);
		if (which < 10)
			GetKorfRubikInstance(start, which);
		else if (which == 19)
		{
			GetSuperFlip(start);
			// Any action will reduce this to 19 moves to solve
			c.ApplyAction(start, 0);
		}
		else if (which == 20)
		{
			GetSuperFlip(start);
		}
		else if (which == 30)
		{
			start.Reset();
			start.edge.SetCubeInLoc(1, 5);
			start.edge.SetCubeInLoc(5, 1);
			start.edge.SetCubeOrientation(5, false);
			start.edge.SetCubeOrientation(1, false);
			start.edge.SetCubeInLoc(0, 6);
			start.edge.SetCubeInLoc(6, 0);
			start.edge.SetCubeOrientation(6, false);
			start.edge.SetCubeOrientation(0, false);
		}
		else if (which > 20)
		{
			GetDepth20(start, which-21);
		}
		return 2;
	}
	else if (strcmp(argument[0], "-bfs") == 0) // not hooked up
	{
		BFS();
		return 1;
	}
	else if (strcmp(argument[0], "-pancake") == 0)
	{
		TestPancake();
		return 1;
	}
	else if (strcmp(argument[0], "-ts") == 0)
	{
		TestTopspin();
		return 1;
	}
	else if (strcmp(argument[0], "-animate") == 0)
	{
		Animate();
		exit(0);
		return 1;
	}
	else if (strcmp(argument[0], "-toh") == 0)
	{
		TOHTest();
		return 1;
	}
	else if (strcmp(argument[0], "-pida") == 0)
	{
		if (!heuristic || !problem)
		{
			printf("Problem and heuristic not defined\n");
			return 0;
		}
		MM::CompareIDA(start, goal, h, heuristicDir);
		exit(0);
		return 1;
	}
	else if (strcmp(argument[0], "-mm") == 0)
	{
		if (maxNumArgs <= 2)
		{
			return 0;
		}
		if (!heuristic || !problem)
		{
			printf("Problem and heuristic not defined\n");
			return 0;
		}
		MM::MM(start, goal, argument[1], argument[2], h, heuristicDir);
		exit(0);
		return 3;
	}
	/*
	 * These aren't currently hooked up!
	 */
	else if (strcmp(argument[0], "-mm0") == 0) // not hooked up
	{
		if (maxNumArgs <= 3)
		{
			printf("Usage:\nbidirectional -mm0 <problem> <tmpdir1 <tmpdir2>\n");
			exit(0);
		}
		MM0::MM0(start, goal, argument[2], argument[3]);
	}
	else if (maxNumArgs > 2 && strcmp(argument[0], "-grid") == 0)
	{
		double weight = 1.0;
		if (maxNumArgs > 3)
			weight = atof(argument[3]);
		AnalyzeMap(argument[1], argument[2], weight);
		return 3;
	}
	else if (strcmp(argument[0], "-stp") == 0 && maxNumArgs > 1)
	{
		TestSTP(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-rubik") == 0 && maxNumArgs > 1)
	{
		TestRubik(atoi(argument[1]));
	}
	else if (maxNumArgs > 2 && strcmp(argument[0], "-nbs") == 0)
	{
		double weight = 1.0;
		if (maxNumArgs > 3)
			weight = atof(argument[3]);
		AnalyzeNBS(argument[1], argument[2], weight);
		return 3;
	}
	else if (maxNumArgs > 2 && strcmp(argument[0], "-testPruning") == 0)
	{
		TestPruning(atoi(argument[1]), atoi(argument[2]));
		return 3;
	}
	else if (strcmp(argument[0], "-easy") == 0)
	{
		Test100Easy();
		return 1;
	}
}


void BFS()
{
	Timer t;
	t.StartTimer();
	ClearFiles();
	RubiksCube c;
	RubiksState s;
	//uint64_t start1 = strtoll(argv[0], 0, 10);
	//GetInstanceFromStdin(s);
	GetKorfRubikInstance(s, 0);
	//GetSuperFlip(s);
	hash128 start;
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	std::vector<hash128> states;
	states.push_back(start);
	WriteStatesToDisk(states, 0);

	// write goal to disk
	s.Reset();
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	states.clear();
	states.push_back(start);
	WriteStatesToDisk(states, 1);
	
	int depth = 2;
	bool done = false;
	while (!done)
	{
		Timer t2, t3;
		t2.StartTimer();
		t3.StartTimer();
		printf("***Starting layer: %d\n", depth); fflush(stdout);
		ExpandLayer(depth-2);
		printf("%1.2fs expanding\n", t2.EndTimer());  fflush(stdout);
		t2.StartTimer();
		done = DuplicateDetectLayer(depth);
		printf("%1.2fs dd\n", t2.EndTimer());
		printf("%1.2fs total elapsed at depth %d\n", t3.EndTimer(), depth);  fflush(stdout);
		depth++;
	}
	printf("%1.2fs elapsed; found solution at depth %d (cost %d)\n", t.EndTimer(), depth-1, depth-2);  fflush(stdout);
}

const char *GetFileName(int depth, int bucket)
{
	static char fname[255];
	sprintf(fname, "bfs-d%d-b%d.dat", depth, bucket);
	return fname;
}

void ClearFiles()
{
	for (int depth = 0; depth < 20; depth++)
	{
		for (int bucket = 0; bucket < kNumBuckets; bucket++)
			remove(GetFileName(depth, bucket));
	}
}

void ConvertToBucketHash(const hash128 &s, uint64_t &bucket, uint64_t &hash)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	bucket = s.cornerHash%kNumBuckets;
	hash   = (((s.cornerHash/kNumBuckets)*maxEdgeRank + s.edgeHash)<<5) + s.parent;
}

void ConvertBucketHashToState(uint64_t bucket, uint64_t hash, hash128 &s)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	
	s.parent = hash&0x1F;
	hash >>= 5;
	s.edgeHash = hash%maxEdgeRank;
	s.cornerHash = uint32_t(hash/maxEdgeRank);
	s.cornerHash = s.cornerHash*kNumBuckets + uint32_t(bucket);

}

void WriteStatesToDisk(std::vector<hash128> &states, int depth)
{
	RubiksCorner c;
	RubikEdge e;

	std::vector<FILE *> files;
	files.resize(kNumBuckets);
	std::vector<uint64_t> counts;
	counts.resize(kNumBuckets);
	
	// 40 bits for edges
	// 27 bits for corners
	// 67 bits
	// 512 buckets (9 bits)
	// 58 bits total
	// 6 bits for extra information (parent)
	for (auto &s : states)
	{
		uint64_t bucket, hash;
		ConvertToBucketHash(s, bucket, hash);
//		uint64_t bucket = s.cornerHash%kNumBuckets;
//		uint64_t hash   = (s.cornerHash/kNumBuckets)*e.getMaxSinglePlayerRank() + s.edgeHash;
		if (files[bucket] == 0)
		{
			files[bucket] = fopen(GetFileName(depth, (int)bucket), "a");
			if (files[bucket] == 0)
			{
				printf("Can't open file %s\n", GetFileName(depth, (int)bucket));
			}
		}
//		if (depth < 2)
//			printf("%2d): Writing %llu %llu\n", depth, bucket, hash);
		fwrite(&hash, sizeof(uint64_t), 1, files[bucket]);
		counts[bucket]++;
	}
	for (int x = 0; x < files.size(); x++)
	{
		FILE *f = files[x];
		if (f)
		{
			fclose(f);
			//printf("%llu states written to depth %d bucket %d [%s]\n", counts[x], depth, x, GetFileName(depth, (int)x));
		}
	}
}

void ExpandLayer(int depth)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<RubiksAction> acts;
	std::vector<hash128> nextLevel;
	hash128 parent, child;
	uint64_t total = 0;
	for (int x = 0; x < kNumBuckets; x++)
	{
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;
		//printf("-Expanding depth %d bucket %d [%s]\n", depth, x, GetFileName(depth, x));
		uint64_t next;
		uint64_t count = 0;
		while (fread(&next, sizeof(uint64_t), 1, f) == 1)
		{
			total++;
			count++;
			ConvertBucketHashToState(x, next, parent);
//			uint64_t cornerRank;
//			uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
//			cornerRank = next/e.getMaxSinglePlayerRank();
//			cornerRank = cornerRank*kNumBuckets + x;
			//printf("%2d): Expanding %llu %llu\n", depth, x, next);
			c.GetStateFromHash(parent.cornerHash, parent.edgeHash, s);
			if (parent.parent < 20)
				c.GetPrunedActions(s, parent.parent, acts);
			else
				c.GetActions(s, acts);
			for (int a = 0; a < acts.size(); a++)
			{
				c.ApplyAction(s, acts[a]);
//				int toParent = acts[a];
//				c.InvertAction(toParent);
				child.parent = acts[a];
				child.edgeHash = c.GetEdgeHash(s);
				child.cornerHash = c.GetCornerHash(s);
				c.UndoAction(s, acts[a]);
				nextLevel.push_back(child);
			}
		}
		fclose(f);
		//printf("-Read %llu states from depth %d bucket %d\n", count, depth, x);
		WriteStatesToDisk(nextLevel, depth+2);
		nextLevel.clear();
	}
	printf("%llu states at depth %d\n", total, depth);
}

bool DuplicateDetectLayer(int depth)
{
	const int bufferSize = 128;
	
	double m0Dups = 0;
	double m2Dups = 0;
	double m4Dups = 0;
	double fDups = 0;
	double writeTime = 0;
	std::vector<uint64_t> values;
	values.resize(bufferSize);
	Timer t;
	uint64_t count = 0;
	bool dups = false;
	std::unordered_map<uint64_t, uint8_t> map;
	uint64_t removed0 = 0, removed2 = 0;
	for (int x = 0; x < kNumBuckets; x++)
	{
		t.StartTimer();
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;

		map.clear();
		count = 0;
		uint64_t numRead;
		while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
		{
			for (int x = 0; x < numRead; x++)
				map[values[x]>>5] = values[x]&0x1F;
			count++;
		}
		fclose(f);
		m0Dups += t.EndTimer();
		//printf("Read %llu states from depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		
		t.StartTimer();
		f = fopen(GetFileName(depth-2, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					//printf("Looking for duplicate %d %llu\n", x, next);
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Removing duplicate %d %llu\n", x, loc->first);
						removed0++;
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m2Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-4, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						removed2++;
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m4Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-1, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Found duplicate!\n");
						dups = true;
					}
				}
			}
			fclose(f);
		}
		fDups += t.EndTimer();
		
		t.StartTimer();
		count = 0;
		f = fopen(GetFileName(depth, x), "w");
		if (f == 0)
		{
			printf("Error with %s\n", GetFileName(depth, x));
			exit(0);
		}
		values.resize(0);
		for (auto val : map)
		{
			//if (val.second)
			{
				values.push_back((val.first<<5)|(val.second));
				count++;
				//printf("%2d): Writing %llu %llu\n", depth, x, val.first);
				
				if (values.size() >= bufferSize)
				{
					fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
					values.resize(0);
				}
			}
		}
		if (values.size() > 0)
			fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
		//printf("Wrote %llu states to depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		fclose(f);
		writeTime += t.EndTimer();
	}
	printf("%1.2f dup vs 0, %1.2f dup vs -2, %1.2f dup vs -4, %1.2f dup vs other frontier, %1.2f write; %llu dups at -2, %llu at -4\n",
		   m0Dups, m2Dups, m4Dups, fDups, writeTime, removed0, removed2);
	return dups;
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

void GetDepth20(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	char string[10][maxStrLength] = //"U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	{
		"B2 L B2 R- F- U- B- L D- F- L U L2 B2 L- D2 B2 D2 R2 B2",
		"R U2 R D2 R2 B2 L- D- B- F U B- R- U2 L- D R2 F- U2 L2",
		"D2 R2 F2 D2 F2 D2 R- F2 D- L2 R B L- F U R- B F2 R2 F-",
		"D- F- U B2 R2 F R- U2 B- L D F R D2 R2 L2 D- R2 F2 D-",
		"U2 R2 F2 D- U F2 U2 B U B- R U- F L B R- F L2 D- B",
		"D B2 D- B2 R2 D- R2 U L R- D B- D R F- D2 R2 U- F- R",
		"B D- L- F- L F B U- D2 F- R2 B- U F2 R- L U2 R2 F2 B2",
		"U2 L- U2 F2 L- R D2 L2 B- D2 L F- R- U- L U2 F- D- R B",
		"F- L B2 R U- B- L U2 D3 F L- R2 U2 D2 B2 R2 D R2 L2 F2",
		"U2 R2 D2 B U2 B- F D- B- R- D U2 B2 F2 R- D- B U- F- R2"
	};

	RubiksCube c;

	start.Reset();
	
	int index = 0;
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[which][index], act);
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


void GetSuperFlip(RubiksState &start)
{
	RubiksCube c;
	const int maxStrLength = 1024;
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

void GetKorfRubikInstance(RubiksState &start, int which)
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
	RubiksCube c;
	
	const int maxStrLength = 1024;
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B R- F- B R-";
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2";
//	char string[maxStrLength] = // length 18
//	"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2";
	char string[maxStrLength] = // length 16
	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2";
	// 	const char *result = fgets(string, maxStrLength, stdin);

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


void TestPruning(int depth, int bucket)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<uint64_t> depths(17);
	int count = 0;
	FILE *f = fopen(GetFileName(depth, bucket), "r");
	if (f == 0)
	{
		printf("Unable to open '%s'; aborting!\n", GetFileName(depth, bucket));
		return;
	}
	uint64_t next;
	while (fread(&next, sizeof(uint64_t), 1, f) == 1)
	{
		if (++count > 1000)
		{
			break;
		}
		printf("%d\r", count); fflush(stdout);
		uint64_t cornerRank;
		uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
		cornerRank = next/e.getMaxSinglePlayerRank();
		cornerRank = cornerRank*kNumBuckets + bucket;
		//printf("%2d): Expanding %llu %llu\n", depth, x, next);
		c.GetStateFromHash(cornerRank, edgeRank, s);
		
		depths[c.Edge12PDBDist(s)]++;
	}
	fclose(f);
	printf("\n\n");
	for (int x = 0; x < depths.size(); x++)
	{
		printf("%d\t%llu\n", x, depths[x]);
	}
}

void GetRandom15(RubiksState &start, int which)
{
	RubiksCube c;
	srandom(which);
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	for (int x = 0; x < 15; x++)
	{
		c.GetActions(start, acts);
		c.ApplyAction(start, acts[random()%acts.size()]);
	}
}

void PDBTest()
{
	RubiksState goal;
	RubiksCube cube;
	std::vector<int> blank;
	std::vector<int> test = {0, 1, 2};//, 4, 5, 6, 7};
//	std::vector<int> corners = {0, 1, 2, 3, 4};//, 5, 6, 7};
//	std::vector<int> edges = {0};
	std::vector<int> corners = {1, 3, 6};//, 5, 6, 7};
	std::vector<int> edges = {1, 3, 6};
	

	if (1)
	{
		const int N = 13;
		const int k = 13;
		MR1KPermutation p;
		uint64_t count = 1;
		for (int x = N; x > (N-k); x--)
			count *= x;
		int items[N], dual[N];
		for (uint64_t x = 0; x < 0/*count*/; x++)
		{
			if (0 == x%(count/1000)) // Print every 0.1% complete
				printf("%llu of %llu\n", x, count);
			
			p.Unrank(x, items, dual, k, N);

//			for (int y = 0; y < N; y++)
//				printf("%d ", items[y]);
//			printf(" : ");
//			for (int y = 0; y < N; y++)
//				printf("%d ", dual[y]);
//			printf("\n");
			
			uint64_t rank = p.Rank(items, dual, k, N);
			if (x != rank)
			{
				printf("Unranked %llu and got %llu back\n", x, rank);
				assert(x == rank);
			}
		}
		printf("Success!\n");
std::random_device rd;
std::mt19937 g(rd());

		for (int t = 0; t < 10000; t++)
		{
			for (int x = 0; x < N; x++)
				items[x] = x;
			std::shuffle(&items[0], &items[N], g);
			for (int x = 0; x < N; x++)
			{
				printf("%d ", items[x]);
				dual[items[x]] = x;
			}
			printf(": ");
			for (int x = 0; x < N; x++)
			{
				printf("%d ", dual[x]);
			}
			printf("\n");
			uint64_t hash = p.Rank(items, dual, N, N);
			p.Unrank(hash, items, dual, N, N);
			for (int x = 0; x < N; x++)
			{
				printf("%d ", items[x]);
			}
			printf("\n");

			assert(p.Rank(items, dual, N, N) == hash);
		}
		//			for (int x = N-1; x >= 0; x++)
		//			{
		//				int n = random()%(x+1);
		//				int temp = items[n];
		//				items[n] = items[x];
		//				items[x] = tmp;
		//			}
		
		exit(0);
		
//		MR1Permutation mr1(test, 8, std::thread::hardware_concurrency());
//		std::vector<int> items;
//		for (int x = 0; x < mr1.GetMaxRank(); x++)
//		{
//			mr1.Unrank(x, items);
//			assert(mr1.GetRank(items) == x);
//			for (int i : items)
//				std::cout << i << " ";
//			std::cout << "\n";
//		}
	}
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges, corners);
	//RubikPDB *pdb3 = new RubikPDB(&cube, goal, test, blank);

	if (0)
	{
		for (uint64_t x = 0; x < pdb3->GetPDBSize(); x++)
		{
			if (0 == x%10000)
				printf("%llu of %llu\n", x, pdb3->GetPDBSize());
			goal.Reset();
			pdb3->GetStateFromPDBHash(x, goal);
			uint64_t hash = pdb3->GetPDBHash(goal);
			if (hash != x)
			{
				std::cout << "State " << x << ": " << goal << "\n";
				pdb3->GetStateFromPDBHash(x-1, goal);
				std::cout << "State " << x-1 << ": " << goal << "\n";
				pdb3->GetStateFromPDBHash(x, goal);
				std::cout << "State " << x << ": " << goal << "\n";
				pdb3->GetStateFromPDBHash(x, goal);

				std::cout << "Rank: " << pdb3->GetPDBHash(goal) << " (" << hash << ")\n";
				assert(pdb3->GetPDBHash(goal) == x);
			}
		}
		printf("Done\n");
		goal.Reset();
	}

	
	pdb3->BuildPDBForward(goal, std::thread::hardware_concurrency());
//	pdb3->BuildPDBBackward(goal, std::thread::hardware_concurrency());
	pdb3->BuildPDBForwardBackward(goal, std::thread::hardware_concurrency());
	exit(0);
}

void GetKorf1997(Heuristic<RubiksState> &result)
{
	const char * hprefix = "/Users/nathanst/Desktop/pdb/";
	
	RubiksState goal;
	RubiksCube cube;
	std::vector<int> blank;
	std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
	std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
	std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
	
	//	assert(!"File names are getting corrupted here. Perhaps by the abstraction of the goal state");
	//	assert(!"Need to abstract the goal state immediately when creating the pdb instead of only when I build the pdb");
	if (!pdb1->Load(hprefix))
	{
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb2->Load(hprefix))
	{
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb3->Load(hprefix))
	{
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	result.lookups.push_back({kMaxNode, 1, 3});
	result.lookups.push_back({kLeafNode, 0, 0});
	result.lookups.push_back({kLeafNode, 1, 0});
	result.lookups.push_back({kLeafNode, 2, 0});
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
}

void GetKorf1997(Heuristic<RubiksState> &result, const RubiksState &goal)
{
	const char * hprefix = "/Users/nathanst/Desktop/pdb/";
	
	RubiksCube cube;
	std::vector<int> blank;
	std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
	std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
	std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
	
	//	assert(!"File names are getting corrupted here. Perhaps by the abstraction of the goal state");
	//	assert(!"Need to abstract the goal state immediately when creating the pdb instead of only when I build the pdb");
	if (!pdb1->Load(hprefix))
	{
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb2->Load(hprefix))
	{
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb3->Load(hprefix))
	{
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	result.lookups.push_back({kMaxNode, 1, 3});
	result.lookups.push_back({kLeafNode, 0, 0});
	result.lookups.push_back({kLeafNode, 1, 0});
	result.lookups.push_back({kLeafNode, 2, 0});
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
}

void Test100Easy()
{
	Heuristic<RubiksState> result;
	std::vector<RubiksAction> path;
	GetKorf1997(result);
	Timer t;
	for (int x = 0; x < 100; x++)
	{
		RubiksCube cube;
		std::vector<RubiksAction> acts;
		cube.SetPruneSuccessors(true);
		RubiksState start, goal;
		GetRandom15(start, x);
		ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> ida;
		ida.SetHeuristic(&result);
		t.StartTimer();
		ida.GetPath(&cube, start, goal, path);
		t.EndTimer();
		printf("%1.5fs elapsed\n", t.GetElapsedTime());
		printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(), ida.GetNodesExpanded()/t.GetElapsedTime());
		printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(), ida.GetNodesTouched()/t.GetElapsedTime());
		printf("Solution cost: %lu\n", path.size());
		
		std::cout << "Acts: ";
		for (unsigned int x = 0; x < path.size(); x++)
		{
			std::cout << path[x] << " ";
		}
		std::cout << "\n";
	}
}

void DualTest()
{
	Heuristic<RubiksState> result;
	Heuristic<RubiksState> dual;
	std::vector<RubiksAction> path;
	GetKorf1997(result);
	dual = result;
	for (int x = 0; x < dual.heuristics.size(); x++)
	{
		dual.heuristics[x] = new RubikDualPDB((RubikPDB*)dual.heuristics[x]);
	}

	RubiksState goal;
	for (int x = 0; x < 1000; x++)
	{
		RubiksCube cube;
		RubiksState s, d;
//		std::cout << "start: " << s << "\n";
		std::vector<int> a;
		for (int y = 0; y < 20; y++)
		{
			a.push_back(random()%18);
		}
		for (int x = 0; x < a.size(); x++)
		{
			cube.ApplyAction(s, a[x]);
			cube.UndoAction(d, a[a.size()-x-1]);
		}
//		std::cout << "orig: " << s << "\n";
//		std::cout << "dual: " << d << "\n";
		printf("Regular heuristic: %f; dual: %f\n", result.HCost(s, goal), result.HCost(d, goal));
//		//dual.HCost(goal, s);
		printf("(dual) Regular heuristic: %f; dual: %f\n", dual.HCost(goal, d), dual.HCost(goal, s));
	}
	exit(0);
}

void ArbitraryGoalTest()
{
	Heuristic<RubiksState> result;
	Heuristic<RubiksState> reverse;
	Heuristic<RubiksState> arbitrary;
	std::vector<RubiksAction> path;

	RubiksState goal;
	RubiksCube cube;
	RubiksState s;
	for (int y = 0; y < 20; y++)
	{
		cube.ApplyAction(s, (y*7)%18);
	}

	GetKorf1997(result);
	GetKorf1997(reverse, s);
	arbitrary = result;
	for (int x = 0; x < arbitrary.heuristics.size(); x++)
	{
		arbitrary.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)arbitrary.heuristics[x]);
	}
	
	printf("Regular heuristic: %f; regular using arbitrary: %f\n\n", result.HCost(s, goal), arbitrary.HCost(s, goal));
	printf("Reverse heuristic: %f; reverse using arbitrary: %f\n\n", reverse.HCost(goal, s), arbitrary.HCost(goal, s));
	goal = s;
	for (int x = 0; x < 2000000; x++)
	{
		if (0 == x%1000)
			printf("--> New state %d of 1000000\n", x+1);
		cube.ApplyAction(s, random()%18);
		double h1, h2;
		h1 = reverse.HCost(s, goal);
		h2 = arbitrary.HCost(s, goal);
//		printf("Reverse heuristic: %f; reverse using arbitrary: %f\n", h1, h2);
		
		if (h1 != h2)
		{
			printf("FAILED\n");
			exit(0);
		}
	}
	printf("SUCCESS\n");
	
	exit(0);
}

#include "Map2DEnvironment.h"
#include "SVGUtil.h"
#include "TemplateAStar.h"
#include "WeightedVertexGraph.h"

int GetGCount(TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, double g, double opt);
void Animate(Map *m, xyLoc p1, xyLoc p2);

void Animate()
{
	Map *m = new Map("/Users/nathanst/hog2/maps/local/bidir.map");
	xyLoc p1 = {9,12};
	xyLoc p2 = {16,15};
	Animate(m, p1, p2);

//	Map *m = new Map("/Users/nathanst/hog2/maps/local/bidir2.map");
//	xyLoc p1 = { 5, 8};
//	xyLoc p2 = { 9, 8};
//	Animate(m, p1, p2);

}

void Animate(Map *m, xyLoc p1, xyLoc p2)
{
	const int spread = 2;
	MapEnvironment *me = new MapEnvironment(m);
	Graphics::Display d;
	me->SetDiagonalCost(1.5);

	// Basic map
	d.StartFrame();
	me->Draw(d);
	me->SetColor(1, 0, 0);
	me->Draw(d, p1);
	me->SetColor(0, 0, 1);
	me->Draw(d, p2);
	d.EndFrame();
	MakeSVG(d, "/Users/nathanst/bidir/anim-b0.svg", 800, 800);
	uint16_t rs = m->GetMapWidth()-3;
	uint16_t ls = 2;

	uint16_t h = m->GetMapHeight();
	
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astarf;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astarb;
	std::vector<xyLoc> path;
	astarf.GetPath(me, p1, p2, path);
	astarb.GetPath(me, p2, p1, path);
	double opt = me->GetPathLength(path);

	// Necessary forward expansions
	d.StartFrame();
	me->Draw(d);
	me->SetColor(1, 0, 0);
	me->Draw(d, p1);
	me->SetColor(0, 0, 1);
	me->Draw(d, p2);

	me->SetColor(1, 0, 0);
	for (int x = 0; x < astarf.GetNumItems(); x++)
	{
		auto i = astarf.GetItem(x);
		if (i.g+i.h < opt)
			me->DrawAlternate(d, i.data);
	}
	d.EndFrame();
	MakeSVG(d, "/Users/nathanst/bidir/anim-b1.svg", 800, 800);

	// Necessary backward expansions
	d.StartFrame();
	me->Draw(d);
	me->SetColor(1, 0, 0);
	me->Draw(d, p1);
	me->SetColor(0, 0, 1);
	me->Draw(d, p2);
	
	me->SetColor(0, 0, 1);
	for (int x = 0; x < astarb.GetNumItems(); x++)
	{
		auto i = astarb.GetItem(x);
		if (i.g+i.h < opt)
			me->DrawAlternate(d, i.data);
	}
	d.EndFrame();
	MakeSVG(d, "/Users/nathanst/bidir/anim-b2.svg", 800, 800);

	// animate necessary states

	const float numFrames = 64;
	for (int frame = 0; frame <= numFrames; frame++)
	{
		d.StartFrame();
		me->Draw(d);

		uint16_t actual = 0;
		me->SetColor(1, 0, 0);
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {ls, actual};
				me->Draw(d, i.data, l, frame/numFrames);
				actual++;
			}
		}
		actual = 0;
		me->SetColor(0, 0, 1);
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {rs, static_cast<uint16_t>(h-1-actual)};
				me->Draw(d, i.data, l, frame/numFrames);
				actual++;
			}
		}
		d.EndFrame();
		std::string path = "/Users/nathanst/bidir/part1/anim-a"+std::to_string(frame/10)+std::to_string(frame%10)+".svg";
		MakeSVG(d, path.c_str(), 800, 800);
	}


	// Draw graph with weights on nodes
	d.StartFrame();
	
	me->Draw(d);
	{
		uint16_t actual = 0;
		me->SetColor(1, 0, 0);
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {ls, actual};
				me->SetColor(1, 1, 1);
				me->Draw(d, l);
				me->SetColor(1, 0, 0);
				me->DrawAlternate(d, l);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l, to_string_with_precision(i.g, 2).c_str());
				//				me->Draw(d, i.data, l, 1.0);
				actual++;
			}
		}
		actual = 0;
		me->SetColor(0, 0, 1);
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {rs, static_cast<uint16_t>(h-1-actual)};
				me->SetColor(1, 1, 1);
				me->Draw(d, l);
				me->SetColor(0, 0, 1);
				me->DrawAlternate(d, l);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l, to_string_with_precision(i.g, 2).c_str());
				actual++;
			}
		}
	}
	d.EndFrame();
	MakeSVG(d, "/Users/nathanst/bidir/anim-b3.svg", 800, 800);

	
	// Draw bipartite graph
	d.StartFrame();
	
	uint16_t actual = 0;
	me->SetColor(0, 0, 0);
	for (int x = 0; x < astarf.GetNumItems(); x++)
	{
		auto i = astarf.GetItem(x);
		if (i.g+i.h < opt)
		{
			uint16_t actual2 = 0;
			for (int y = 0; y < astarb.GetNumItems(); y++)
			{
				auto j = astarb.GetItem(y);
				if (i.g+j.g < opt && j.g+j.h < opt)
				{
					xyLoc l1 = {ls, static_cast<uint16_t>(actual)};
					xyLoc l2 = {rs, static_cast<uint16_t>(h-1-actual2)};
					me->DrawLine(d, l1, l2, 0.5);
					actual2++;
				}
			}
			actual++;
		}
	}
	{
		uint16_t actual = 0;
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {ls, actual};
				me->SetColor(1, 1, 1);
				me->Draw(d, l);
				me->SetColor(1, 0, 0);
				me->DrawAlternate(d, l);
				me->SetColor(0, 0, 0);

				me->DrawStateLabel(d, l, to_string_with_precision(i.g, 2).c_str());
				actual++;
			}
		}
		actual = 0;
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l = {rs, static_cast<uint16_t>(h-1-actual)};
				me->SetColor(1, 1, 1);
				me->Draw(d, l);
				me->SetColor(0, 0, 1);
				me->DrawAlternate(d, l);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l, to_string_with_precision(i.g, 2).c_str());
				actual++;
			}
		}
	}
	d.EndFrame();
	MakeSVG(d, "/Users/nathanst/bidir/anim-b4.svg", 800, 800);

	// shouldnt' be hard coded!
	int newh = 21;
	
	// Animate moving of states according to g-cost
	for (int frame = 0; frame <= numFrames; frame++)
	{
		d.StartFrame();
		//me->Draw(d);
		
		uint16_t actual = 0;
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l1 = {ls, actual};
				xyLoc l2 = {ls, static_cast<uint16_t>(i.g*spread)};
				
				me->SetColor(1, 0, 0);
				me->Draw(d, l1, l2, frame/numFrames);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l1, l2, frame/numFrames, to_string_with_precision(i.g, 2).c_str());
				actual++;
			}
		}
		actual = 0;
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l1 = {rs, static_cast<uint16_t>(h-1-actual)};
				xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-i.g*spread)};
				
				me->SetColor(0, 0, 1);
				me->Draw(d, l1, l2, frame/numFrames);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l1, l2, frame/numFrames, to_string_with_precision(i.g, 2).c_str());
				actual++;
			}
		}
		d.EndFrame();
		std::string path = "/Users/nathanst/bidir/part2/anim-c"+std::to_string(frame/10)+std::to_string(frame%10)+".svg";
		MakeSVG(d, path.c_str(), 800, 800);
	}
	

	// Draw final graph
	// Animate moving of states according to g-cost
	{
		d.StartFrame();
//		me->Draw(d);
		
		uint16_t actual = 0;
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				xyLoc l2 = {ls, static_cast<uint16_t>(i.g*spread)};
				
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(1, 0, 0);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x--;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarf, i.g, opt)).c_str());
				l2.x--;
//				char tmp[2] = {0, 0};
//				tmp[0] = 'a'+(char)i.g;
//				me->DrawStateLabel(d, l2, tmp);
				actual++;
			}
		}
		actual = 0;
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				//xyLoc l1 = {rs, actual};
				xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-i.g*spread)};
				
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(0, 0, 1);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x++;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarb, i.g, opt)).c_str());
				l2.x++;
//				char tmp[2] = {0, 0};
//				tmp[0] = 'a'+(char)i.g;
//				me->DrawStateLabel(d, l2, tmp);
				actual++;
			}
		}
		d.EndFrame();
		std::string path = "/Users/nathanst/bidir/anim-b5.svg";
		MakeSVG(d, path.c_str(), 800, 800);
	}
	

	// Draw edges into final graph
	{
		d.StartFrame();
//		me->Draw(d);

		{
			me->SetColor(0, 0, 0);
			for (int x = 0; x < astarf.GetNumItems(); x++)
			{
				auto i = astarf.GetItem(x);
				if (i.g+i.h < opt)
				{
					for (int y = 0; y < astarb.GetNumItems(); y++)
					{
						auto j = astarb.GetItem(y);
						if (i.g+j.g < opt && j.g+j.h < opt)
						{
							xyLoc l1 = {ls, static_cast<uint16_t>(i.g*spread)};
							xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-j.g*spread)};
							me->DrawLine(d, l1, l2, 0.5);
						}
					}
				}
			}
		}

		
		uint16_t actual = 0;
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				//				xyLoc l1 = {ls, actual};
				xyLoc l2 = {ls, static_cast<uint16_t>(i.g*spread)};
			
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(1, 0, 0);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x--;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarf, i.g, opt)).c_str());
				actual++;
			}
		}
		actual = 0;
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				//xyLoc l1 = {23, actual};
				xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-i.g*spread)};
				
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(0, 0, 1);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x++;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarb, i.g, opt)).c_str());
				actual++;
			}
		}
		d.EndFrame();
		std::string path = "/Users/nathanst/bidir/anim-b6.svg";
		MakeSVG(d, path.c_str(), 800, 800);
	}
	
	// Draw limited edges into final graph
	{
		d.StartFrame();
		//		me->Draw(d);
		
		{
			me->SetColor(0, 0, 0);
			for (int x = 0; x < astarf.GetNumItems(); x++)
			{
				auto i = astarf.GetItem(x);
				if (i.g+i.h < opt)
				{
					double maxNeighbor = 0;
					for (int y = 0; y < astarb.GetNumItems(); y++)
					{
						auto j = astarb.GetItem(y);
						if (i.g+j.g < opt && j.g+j.h < opt)
						{
							maxNeighbor = std::max(maxNeighbor, j.g);
						}
					}
					if (maxNeighbor != 0 && i.g != 0)
					{
						xyLoc l1 = {ls, static_cast<uint16_t>(i.g*spread)};
						xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-maxNeighbor*spread)};
						me->DrawLine(d, l1, l2, 0.5);
					}
//					else {
//						xyLoc l1 = {ls, static_cast<uint16_t>(i.g*spread)};
//						xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-maxNeighbor*spread)};
//						float len = (ls-rs)*(ls-rs)+(i.g*spread-(newh-1-maxNeighbor*spread))*(i.g*spread-(newh-1-maxNeighbor*spread));
//						float xoff = (rs-ls)/len;
//						float yoff = (newh-1-maxNeighbor*spread)-(i.g*spread)/len;
//						len = sqrt(len);
//						xyLoc l3 = {static_cast<uint16_t>(ls+20*xoff), static_cast<uint16_t>(i.g*spread+20*yoff)};
//						xyLoc l4 = {static_cast<uint16_t>(rs-20*xoff), static_cast<uint16_t>((newh-1-maxNeighbor*spread)-20*yoff)};
//						me->DrawLine(d, l1, l3, 0.5);
//						me->DrawLine(d, l4, l2, 0.5);
//					}
				}
			}
		}
		
		
		uint16_t actual = 0;
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			auto i = astarf.GetItem(x);
			if (i.g+i.h < opt)
			{
				//				xyLoc l1 = {ls, actual};
				xyLoc l2 = {ls, static_cast<uint16_t>(i.g*spread)};
				
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(1, 0, 0);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x--;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarf, i.g, opt)).c_str());
				actual++;
			}
		}
		actual = 0;
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			auto i = astarb.GetItem(x);
			if (i.g+i.h < opt)
			{
				//xyLoc l1 = {23, actual};
				xyLoc l2 = {rs, static_cast<uint16_t>(newh-1-i.g*spread)};
				
				me->SetColor(1, 1, 1);
				me->Draw(d, l2);
				me->SetColor(0, 0, 1);
				me->DrawAlternate(d, l2);
				me->SetColor(0, 0, 0);
				me->DrawStateLabel(d, l2, to_string_with_precision(i.g, 2).c_str());
				l2.x++;
				me->DrawStateLabel(d, l2, std::to_string(GetGCount(astarb, i.g, opt)).c_str());
				actual++;
			}
		}
		d.EndFrame();
		std::string path = "/Users/nathanst/bidir/anim-b7.svg";
		MakeSVG(d, path.c_str(), 800, 800);
		// reset to align in next drawing
	}



	
	BidirectionalProblemAnalyzer<xyLoc, tDirection, MapEnvironment>::GetWeightedVertexGraph(p1, p2, me, me, me, "/Users/nathanst/bidir/z.svg");
//	GetWeightedVertexGraph<xyLoc, tDirection>(p2, p1, me, "/Users/nathanst/bidir/z.svg");
}

int GetGCount(TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, double g, double opt)
{
	int cnt = 0;
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
		if (i.g+i.h < opt && i.g == g)
		{
			cnt++;
		}
	}
	return cnt;
}

//void Test()
//{
//	Map *m = new Map(512, 512);
//	MakeMaze(m);
//
//	exit(0);
//}

template <class state, class action, class environment>
void PathTests()
{
	environment mnp;
	state s, g;
	std::vector<state> path;
	std::vector<state> tmp;
	s.Reset();
	TemplateAStar<state, action, environment> astar;
	NBS<state, action, environment> nbs;
	printf("Finding far state\n");
	astar.SetStopAfterGoal(false);
	astar.InitializeSearch(&mnp, s, s, path);
	while (astar.GetNumOpenItems() > 0)
	{
		g = astar.CheckNextNode();
		astar.DoSingleSearchStep(path);
	}
	std::cout << "Nodes: " << astar.GetNodesExpanded() << "\n";
	astar.ExtractPathToStart(g, path);
	std::cout << "Path: " << path.size() << "\n";
	ZeroHeuristic<state> z;
	// Run BFS
	printf("Depth\tBFS_b\tBFS_b^0.5\tBFS_f\tBFS_f^0.5\tNBS0\n");
	astar.SetStopAfterGoal(true);
	for (int x = 0; x < path.size(); x++)
	{
		astar.SetHeuristic(&z);
		astar.GetPath(&mnp, s, path[x], tmp);
		printf("%d\t%llu [%d]\t%1.2f", x, astar.GetNodesExpanded(), tmp.size(), sqrt(astar.GetNodesExpanded()));

		astar.SetHeuristic(&z);
		astar.GetPath(&mnp, path[x], s, tmp);
		printf("\t%llu [%d]\t%1.2f", astar.GetNodesExpanded(), tmp.size(), sqrt(astar.GetNodesExpanded()));

		nbs.GetPath(&mnp, s, path[x], &z, &z, tmp);
		printf("\t%llu [%d]\n", nbs.GetNodesExpanded(), tmp.size());
//		BidirectionalProblemAnalyzer<state, action, environment>::GetWeightedVertexGraph(s, path[x], &mnp, &z, &z);

	}
}

void PathTest()
{
	printf("3x3 STP\n");
	PathTests<MNPuzzleState<3, 3>, slideDir, MNPuzzle<3, 3>>();
	printf("9 Pancake\n");
	PathTests<PancakePuzzleState<9>, PancakePuzzleAction, PancakePuzzle<9>>();
	printf("10 Pancake\n");
	PathTests<PancakePuzzleState<10>, PancakePuzzleAction, PancakePuzzle<10>>();
//	printf("2x2x2 RC\n");
//	PathTests<RubiksCornerState, RubiksCornersAction, RubiksCorner>();
	exit(0);
}


