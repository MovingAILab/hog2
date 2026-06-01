//
//  MM.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include "MMRubik.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "Timer.h"
#include <string>
#include <unordered_set>
#include <iomanip>
#include "SharedQueue.h"
#include "FixedSizeSet.h"

NAMESPACE_OPEN(MM)

//const int fileBuckets = 8; // must be at least 8
//const uint64_t bucketBits = 3;
//const int bucketMask = 0x7;
//const int fileBuckets = 32; // must be at least 8
const uint64_t bucketBits = 5;
const int bucketMask = ((1<<bucketBits)-1);//0x1F;

const char *prefix1;
const char *prefix2;
const char *hprefix;
bool finished = false;

int bestSolution;
int currentC;
int minGForward;
int minGBackward;
int minFForward;
int minFBackward;
uint64_t expanded;
std::vector<uint64_t> gDistForward;
std::vector<uint64_t> gDistBackward;
std::mutex printLock;
std::mutex countLock;
std::mutex openLock;

RubiksState startState, goalState;
//unsigned __int128 i;
//typedef RubiksState diskState;
typedef uint64_t diskState;
#define MY_SET

#ifdef MY_SET
typedef FixedSizeSet<diskState> bucketSet;
#else
typedef std::unordered_set<diskState> bucketSet;
#endif

void GetBucketAndData(const RubiksState &s, int &bucket, diskState &data);
void GetState(RubiksState &s, int bucket, diskState data);
int GetBucket(const RubiksState &s);

void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result, heuristicType h);
RubiksCube cube;

Heuristic<RubiksState> forward;
Heuristic<RubiksState> reverse;

//namespace std
//{
//	template<>
//	struct hash<unsigned __int128> {
//		size_t operator()(const unsigned __int128 &x) const {
//			return (size_t)((x>>66)^(x<<3)^(x>>3));
//		}
//	};
//}

enum  tSearchDirection {
	kForward,
	kBackward,
};

//const int kWorkUnitSize = 128;
//
//struct workUnit {
//	uint8_t count;
//	diskState work[kWorkUnitSize];
//};
//
//SharedQueue<workUnit> work;

// This tells us the open list buckets
struct openData {
	tSearchDirection dir;  // at most 2 bits
	uint8_t priority;      // at most 6 bits
	uint8_t gcost;         // at most 4 bits
	//uint8_t fcost;         // at most 4 bits
	uint16_t bucket;        // at most (3) bits
};

static bool operator==(const openData &a, const openData &b)
{
	return (a.dir == b.dir && a.priority == b.priority && a.gcost == b.gcost /*&& a.fcost == b.fcost*/ && a.bucket == b.bucket);
}

static std::ostream &operator<<(std::ostream &out, const openData &d)
{
	out << "[" << ((d.dir==kForward)?"forward":"backward") << ", p:" << +d.priority << ", g:" << +d.gcost;// << ", f:" << +d.fcost;
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct openDataHash
{
	std::size_t operator()(const openData & x) const
	{
		//return (x.dir)|(x.priority<<2)|(x.gcost<<8)|(x.fcost<<12)|(x.bucket<<20);
		return (x.dir)|(x.priority<<2)|(x.gcost<<8)|(x.bucket<<20);
	}
};

struct openList {
	openList() :f(0), writtenStates(0), minf(0xFF) {}
	//std::unordered_set<uint64_t> states;
	FILE *f;
	uint64_t writtenStates;
	uint8_t minf;
};

static std::ostream &operator<<(std::ostream &out, const openList &d)
{
	out << "[" << d.writtenStates << ", " << +d.minf << "]";
	return out;
}

struct closedData {
	tSearchDirection dir;
	uint8_t depth;
	uint8_t bucket;
};

struct closedList {
	closedList() :f(0) {}
	FILE *f;
};

static bool operator==(const closedData &a, const closedData &b)
{
	return (a.dir == b.dir && a.depth == b.depth && a.bucket == b.bucket);
}

struct closedDataHash
{
	std::size_t operator()(const closedData & x) const
	{
		return (x.dir)|(x.bucket<<2)|(x.bucket<<7);
	}
};

//std::vector<std::unordered_set<uint64_t>> closed(fileBuckets);
std::unordered_map<closedData, closedList, closedDataHash> closed;
std::unordered_map<openData, openList, openDataHash> open;

std::string GetClosedName(closedData d)
{
	std::string s;
	if (d.dir == kForward)
	{
		if ((d.bucket%16) < 8)
		{
			s += prefix1;
		}
		else {
			s += prefix2;
		}
		s += "forward-";
	}
	else {
		if ((d.bucket%16) >= 8)
		{
			s += prefix1;
		}
		else {
			s += prefix2;
		}
		s += "backward-";
	}
	s += std::to_string(d.bucket);
	s += "-";
	s += std::to_string(d.depth);
	s += ".closed";
	return s;
}

std::string GetOpenName(const openData &d)
{
	/* Previously we split across disks according to forward/backward, but
	 * this is unbalanced. Now we split according to buckets. With an even
	 * number of buckets the files should now be even split across two disks.
	 */
	std::string s;
	if ((d.bucket%16) < 8)
	{
		s += prefix1;
	}
	else {
		s += prefix2;
	}
	if (d.dir == kForward)
	{
		s += "forward-";
	}
	else {
		s += "backward-";
	}
	s += std::to_string(d.priority);
	s += "-";
	s += std::to_string(d.gcost);
//	s += "-";
//	s += std::to_string(d.fcost);
//	s += "-";
//	s += std::to_string(d.hcost2);
	s += "-";
	s += std::to_string(d.bucket);
	s += ".open";
	return s;
}

openData GetBestFile()
{
	minGForward = 100;
	minGBackward = 100;
	minFForward = 100;
	minFBackward = 100;
	// actually do priority here
	openData best =(open.begin())->first;
	//return (open.begin())->first;
	for (const auto &s : open)
	{
		//std::cout << "--: " << s.first << " minf: " << +s.second.minf << "\n";
		if (s.first.dir == kForward && s.first.gcost < minGForward)
			minGForward = s.first.gcost;
		else if (s.first.dir == kBackward && s.first.gcost < minGBackward)
			minGBackward = s.first.gcost;
		
		if (s.first.dir == kForward && s.second.minf < minFForward)
			minFForward = s.second.minf;
		else if (s.first.dir == kBackward && s.second.minf < minFBackward)
			minFBackward = s.second.minf;
		
		if (s.first.priority < best.priority)
		{
			best = s.first;
		}
		else if (s.first.priority == best.priority)
		{
			if (s.first.gcost < best.gcost)
				best = s.first;
			else if (s.first.gcost == best.gcost)
			{
				if (s.first.dir == best.dir)
				{
//					if (s.first.fcost < best.fcost)
//						best = s.first;
//					else if (s.first.fcost == best.fcost)
//					{
						if (s.first.bucket < best.bucket)
							best = s.first;
//					}
				}
				else if (s.first.dir == kForward)
					best = s.first;
			}
		}
	}
	//printf("Min f forward: %d backward: %d\n", minFForward, minFBackward);
	return best;
}

void GetOpenData(const RubiksState &from, tSearchDirection dir, int cost,
				 openData &d, diskState &data, uint8_t &fcost)
{
	int bucket;
	//uint64_t data;
	GetBucketAndData(from, bucket, data);
	d.dir = dir;
	d.gcost = cost;
	fcost = cost + ((dir==kForward)?forward.HCost(from, goalState):reverse.HCost(from, startState));
	//d.hcost2 = (dir==kForward)?reverse.HCost(start, start):forward.HCost(start, start);
	d.bucket = bucket;
	//d.priority = d.gcost+d.hcost;
	d.priority = std::max(fcost, uint8_t(d.gcost*2));
}

void AddStatesToQueue(const openData &d, diskState *data, size_t count, uint8_t minFcost)
{
	openLock.lock();
	auto iter = open.find(d);
	if (iter == open.end())
	{
		open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
		if (open[d].f == 0)
		{
			printf("Error opening %s; Aborting!\n", GetOpenName(d).c_str());
			perror("Reason: ");
			exit(0);
		}
		iter = open.find(d);
		iter->second.minf = 100; // new file, start with high f-min
	}
	if (iter->second.f == 0)
	{
		printf("Error - file is null!\n");
		exit(0);
	}
	//iter->second.minf = std::min(iter->second.minf, d.fcost);
	size_t written = fwrite(data, sizeof(diskState), count, open[d].f);
	open[d].writtenStates += written;
	open[d].minf = std::min(open[d].minf, minFcost);
	openLock.unlock();
}

void AddStateToQueue(openData &d, diskState data, uint8_t fcost)
{
	AddStatesToQueue(d, &data, 1, fcost);
}

void AddStateToQueue(const RubiksState &start, tSearchDirection dir, int cost)
{
	openData d;
	diskState rank;
	uint8_t fcost;
	GetOpenData(start, dir, cost, d, rank, fcost);
	AddStateToQueue(d, rank, fcost);
}


bool CanTerminateSearch()
{
	int val;
	if (bestSolution <= (val = std::max(currentC, std::max(minFForward, std::max(minFBackward, minGBackward+minGForward)))))
	{
		printf("Done!\n");
		printf("Min fforward %d; minfbackward: %d; g: %d+%d; currentC: %d; solution: %d\n",
			   minFForward, minFBackward, minGBackward, minGForward, currentC, bestSolution);
		printf("%llu nodes expanded\n", expanded);
		printf("Forward Distribution:\n");
		for (int x = 0; x < gDistForward.size(); x++)
			if (gDistForward[x] != 0)
				printf("%d\t%llu\n", x, gDistForward[x]);
		printf("Backward Distribution:\n");
		for (int x = 0; x < gDistBackward.size(); x++)
			if (gDistBackward[x] != 0)
				printf("%d\t%llu\n", x, gDistBackward[x]);
		finished = true;
		if (val == currentC)
			printf("-Triggered by current priority\n");
		if (val == minFForward)
			printf("-Triggered by f in the forward direction\n");
		if (val == minFBackward)
			printf("-Triggered by f in the backward direction\n");
		if (val == minGBackward+minGForward)
			printf("-Triggered by gforward+gbackward\n");
		return true;
	}
	return false;
}

void FindSolutionThread(const openData &d, const openList &l, int g, const bucketSet &states)
{
	const size_t bufferSize = 128;
	diskState buffer[bufferSize];
	if (l.f == 0)
	{
		std::cout << "Error opening " << d << "\n";
		exit(0);
	}
	rewind(l.f);
	size_t numRead;
	do {
		numRead = fread(buffer, sizeof(diskState), bufferSize, l.f);
		for (int x = 0; x < numRead; x++)
		{
			if (states.find(buffer[x]) != states.end())
			{
				printLock.lock();
				printf("\nFound solution cost %d+%d=%d\n", d.gcost, g, d.gcost + g);
				bestSolution = std::min(d.gcost + g, bestSolution);
				printf("Current best solution: %d\n", bestSolution);
				printLock.unlock();
				
				if (CanTerminateSearch())
					return;
			}
		}
	} while (numRead == bufferSize);
}

void CheckSolution(std::unordered_map<openData, openList, openDataHash> currentOpen, openData d,
				   const bucketSet &states)
{
	std::vector<std::thread *> threads;
	for (const auto &s : currentOpen)
	{
		// Opposite direction, same bucket AND could be a solution (g+g >= C)
		// TODO: only need to check if we find a better solution (g+g < U)
		if (s.first.dir != d.dir && s.first.bucket == d.bucket &&
			d.gcost + s.first.gcost >= currentC && d.gcost + s.first.gcost < bestSolution)// && d.hcost2 == s.first.hcost)
		{
//			std::thread *t = new std::thread(FindSolutionThread, s.first, s.second, states);
//			threads.push_back(t);
			FindSolutionThread(s.first, s.second, d.gcost, states);
		}
	}
//	while (threads.size() > 0)
//	{
//		threads.back()->join();
//		delete threads.back();
//		threads.pop_back();
//	}
}

void ReadBucket(bucketSet &states, openData d)
{
	const size_t bufferSize = 128;
	diskState buffer[bufferSize];
	rewind(open[d].f);
#ifdef MY_SET
	states.resize(open[d].writtenStates);
#endif
	size_t numRead;
	do {
		numRead = fread(buffer, sizeof(diskState), bufferSize, open[d].f);
		for (size_t x = 0; x < numRead; x++)
			states.insert(buffer[x]);
	} while (numRead == bufferSize);
	fclose(open[d].f);
	remove(GetOpenName(d).c_str());
	open[d].f = 0;
}

void RemoveDuplicates(bucketSet &states, openData d)
{
	for (int depth = d.gcost-2; depth < d.gcost; depth++)
	{
		closedData c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = d.dir;
		
		closedList &cd = closed[c];
		if (cd.f == 0)
			continue;
		rewind(cd.f);
		
		const size_t bufferSize = 1024;
		diskState buffer[bufferSize];
		rewind(cd.f);
		size_t numRead;
		do {
			numRead = fread(buffer, sizeof(diskState), bufferSize, cd.f);
			for (size_t x = 0; x < numRead; x++)
			{
				auto i = states.find(buffer[x]);
				if (i != states.end())
					states.erase(i);
			}
		} while (numRead == bufferSize);
	}
}

void WriteToClosed(bucketSet &states, openData d)
{
	closedData c;
	c.bucket = d.bucket;
	c.depth = d.gcost;
	c.dir = d.dir;

	closedList &cd = closed[c];
	if (cd.f == 0)
	{
		cd.f = fopen(GetClosedName(c).c_str(), "w+b");
	}
	for (const auto &i : states)
	{
		fwrite(&i, sizeof(diskState), 1, cd.f);
	}
}

void ParallelExpandBucket(openData d, bucketSet &states, int myThread, int totalThreads)
{
	const int cacheSize = 1024;
	// stores the min f for this bucket, the files to write
	std::unordered_map<openData, std::pair<uint8_t, std::vector<diskState>>, openDataHash> cache;
	RubiksState tmp;
	uint64_t localExpanded = 0;
	int count = 0;
	for (const auto &values : states)
	{
		count++;
		if (myThread != count%totalThreads)
			continue;
		diskState v;
#ifdef MY_SET
		if (!values.valid)
			continue;
		v = values.item;
#else
		v = values;
#endif
		localExpanded++;
		// TODO: test whether we should undo or call GetState()
		GetState(tmp, d.bucket, v);
		for (int x = 0; x < 18; x++) // TODO: use getactions
		{
			cube.ApplyAction(tmp, x);
			openData newData;
			diskState newRank;
			uint8_t fcost;
			GetOpenData(tmp, d.dir, d.gcost+1, newData, newRank, fcost);
			
			auto &i = cache[newData];
			std::vector<diskState> &c = i.second;
			if (i.first == 0)
				i.first = fcost;
			else
				i.first = std::min(i.first, fcost);
			c.push_back(newRank);
			if (c.size() > cacheSize)
			{
				AddStatesToQueue(newData, &c[0], c.size(), i.first);
				c.clear();
			}
			cube.UndoAction(tmp, x);
		}
	}
	for (auto &i : cache)
	{
		if (i.second.second.size() > 0)
			AddStatesToQueue(i.first, &(i.second.second[0]), i.second.second.size(), i.second.first);
	}
	
	countLock.lock();
	expanded += localExpanded;
	if (d.dir == kForward)
	{
		gDistForward[d.gcost]+=localExpanded;
	}
	else {
		gDistBackward[d.gcost]+=localExpanded;
	}
	
	countLock.unlock();
}

void ReadAndDDBucket(bucketSet &states, const openData &d)
{
	ReadBucket(states, d);
	RemoveDuplicates(states, d); // delayed duplicate detection
	WriteToClosed(states, d); // this could run in parallel! (it is if we pre-load)
}

#define DO_PRELOAD

void ExpandNextFile()
{
	static bool preLoaded = false;
	static bucketSet nextStates(0);
	static bucketSet states(0);
	static openData next;
	// 1. Get next expansion target
	openData d = GetBestFile();
	currentC = d.priority;

//	std::cout << "---->Currently on open [" << bestSolution << "]:\n";
//	for (auto &d : open)
//	{
//		std::cout << d.first << "<->" << d.second << "\n";
//	}
//	std::cout << "<----\n";
	if (CanTerminateSearch())
		return;

	Timer timer;
	timer.StartTimer();
	
	states.clear();
	if (preLoaded == false)
	{
		ReadAndDDBucket(states, d);
	}
	else {
		if (d == next)
		{
			states.swap(nextStates);
			preLoaded = false;
		}
		else {
			std::cout << "ERROR: pre-loading changed buckets!\n";
			ReadAndDDBucket(states, d);
		}
	}
	timer.EndTimer();
	
	printLock.lock();
	std::cout << "Next: " << d << " (" << states.size() << " entries "<< open.find(d)->second.writtenStates<<") [" << timer.GetElapsedTime() << "s reading/dd] ";
	printLock.unlock();
	open.erase(open.find(d));

	timer.StartTimer();
	// Read in opposite buckets to check for solutions in parallel to expanding this bucket
	openLock.lock();
	std::thread t(CheckSolution, open, d, std::ref(states));
	openLock.unlock();
	
	std::thread *pre = 0;
	
#ifdef DO_PRELOAD
	// 2. Pre-read next bucket if it is the same as us
	next = GetBestFile();
	if (next.dir == d.dir && next.gcost == d.gcost && next.bucket != d.bucket)
	{
		pre = new std::thread(ReadAndDDBucket, std::ref(nextStates), std::ref(next));
		preLoaded = true;
	}
#endif
	
	// 3. expand all states in current bucket & write out successors
	const auto numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (size_t x = 0; x < numThreads; x++)
		threads.push_back(new std::thread(ParallelExpandBucket, d, std::ref(states), x, numThreads));
	for (size_t x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
	timer.EndTimer();
	printLock.lock();
	std::cout << "[" << timer.GetElapsedTime() << "s expanding]+";
	printLock.unlock();

	// Close thread that is doing DSD
	timer.StartTimer();
	t.join();
	timer.EndTimer();
	printLock.lock();
	std::cout << "[" << timer.GetElapsedTime() << "s] ";
	printLock.unlock();
	
	// Close thread that is reading previous bucket
	timer.StartTimer();
	if (pre != 0)
	{
		pre->join();
		delete pre;
		pre = 0;
	}
	timer.EndTimer();
	printLock.lock();
	std::cout << "[" << timer.GetElapsedTime() << "s]\n";
	printLock.unlock();

}


void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result, heuristicType h)
{
	RubiksCube cube;
	std::vector<int> blank;

	switch (h)
	{
		case kNone:
		{
			ZeroHeuristic<RubiksState> *zero = new ZeroHeuristic<RubiksState>();
			result.lookups.push_back({kLeafNode, 0, 0});
			result.heuristics.push_back(zero);
			break;
		}
		case k444:
		{
			std::vector<int> edges1 = {1, 3, 8, 9}; // first 4
			std::vector<int> edges2 = {0, 2, 4, 5}; // first 4
			std::vector<int> corners = {0, 1, 2, 3}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case kSmall:
		{
			assert(!"PDB not being saved!");
			std::vector<int> edges1 = {0, 1, 2, 4, 6};
			std::vector<int> edges2 = {3, 5};
			std::vector<int> edges3 = {7, 8, 9, 10, 11};
			std::vector<int> corners1 = {0, 1, 2, 3, 4, 5};
			std::vector<int> corners2 = {2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners1);
			RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners2);
			pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
			result.lookups.push_back({kMaxNode, 1, 5});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			break;
		}
		case k1997:
		{
			std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
			std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			
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
			break;
		}
		case k888:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7};
			std::vector<int> edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k839:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8};
			std::vector<int> edges2 = {9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k8210:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
			std::vector<int> edges2 = {10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
	}
}


int GetBucket(const RubiksState &s)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	return ehash&bucketMask;
//	return (s.edge.state^(s.corner.state<<2))&bucketMask;
}

void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
	bucket = ehash&bucketMask;
	data = (ehash>>bucketBits)*RubikCornerPDB::GetStateSpaceSize()+chash;
}

void GetBucketAndData(const RubiksState &s, int &bucket, RubiksState &data)
{
	bucket = GetBucket(s);
	data = s;
}

void GetState(RubiksState &s, int bucket, uint64_t data)
{
	RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
	RubikEdgePDB::GetStateFromHash(s.edge, bucket|((data/RubikCornerPDB::GetStateSpaceSize())<<bucketBits));
}

void GetState(RubiksState &s, int bucket, RubiksState data)
{
	s = data;
}

#pragma mark Main Code

void MM(RubiksState &start, RubiksState &goal, const char *p1, const char *p2,
		heuristicType h, const char *hloc)
//void MM(RubiksState &start, RubiksState &goal, const char *p1, const char *p2, const char *hloc)
{
#ifdef MY_SET
	printf("Using custom set container\n");
#endif
	startState = start;
	goalState = goal;
	prefix1 = p1;
	prefix2 = p2;
	hprefix = hloc;
	
	bestSolution = 100;
	gDistBackward.resize(12);
	gDistForward.resize(12);
	expanded = 0;
	BuildHeuristics(start, goal, forward, h);

	reverse = forward;
	for (int x = 0; x < reverse.heuristics.size(); x++)
	{
		reverse.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)reverse.heuristics[x]);
	}
	//BuildHeuristics(goal, start, reverse);

	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);
	
	Timer t;
	t.StartTimer();
	printf("---MM*---\n");
	AddStateToQueue(start, kForward, 0);
	AddStateToQueue(goal, kBackward, 0);
	while (!open.empty() && !finished)
	{
		ExpandNextFile();
	}
	t.EndTimer();
	printf("%1.2fs elapsed\n", t.GetElapsedTime());
}

void CompareIDA(RubiksState &start, RubiksState &goal, heuristicType h, const char *hloc)
//void CompareIDA(RubiksState &start, RubiksState &goal, const char *p1, const char *p2, const char *hloc)
{
	hprefix = hloc;
	
	bestSolution = 100;
	gDistBackward.resize(12);
	gDistForward.resize(12);
	expanded = 0;
	BuildHeuristics(start, goal, forward, h);
	//BuildHeuristics(goal, start, reverse);
	
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);
	
	printf("---IDA*---\n");
	std::vector<RubiksAction> path;
	Timer t;
	t.StartTimer();
	cube.SetPruneSuccessors(true);
	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> ida;
	ida.SetHeuristic(&forward);
	ida.GetPath(&cube, start, goal, path);
	t.EndTimer();
	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
		   ida.GetNodesExpanded()/t.GetElapsedTime());
	printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
		   ida.GetNodesTouched()/t.GetElapsedTime());
	printf("Solution cost: %lu\n", path.size());

	std::cout << "Acts: ";
	for (unsigned int x = 0; x < path.size(); x++)
	{
		std::cout << path[x] << " ";
	}
	std::cout << "\n";
}

NAMESPACE_CLOSE(MM)
