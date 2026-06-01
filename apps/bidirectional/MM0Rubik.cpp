//
//  MM0Rubik.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/19/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "MMRubik.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "Timer.h"
#include <string>
#include <unordered_set>
#include <iomanip>

#include "MM0Rubik.h"


NAMESPACE_OPEN(MM0)

//const int fileBuckets = 8; // must be at least 8
//const uint64_t bucketBits = 3;
//const int bucketMask = 0x7;
const int fileBuckets = 32; // must be at least 8
const uint64_t bucketBits = 5;
const int bucketMask = 0x1F;

const char *prefix1;
const char *prefix2;
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

void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data);
void GetState(RubiksState &s, int bucket, uint64_t data);
int GetBucket(const RubiksState &s);

void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result);
RubiksCube cube;

enum  tSearchDirection {
	kForward,
	kBackward,
};

// This tells us the open list buckets
struct openData {
	tSearchDirection dir;  // at most 2 bits
	uint8_t priority;      // at most 6 bits
	uint8_t gcost;         // at most 4 bits
	uint8_t bucket;        // at most (3) bits
};

static bool operator==(const openData &a, const openData &b)
{
	return (a.dir == b.dir && a.priority == b.priority && a.gcost == b.gcost && a.bucket == b.bucket);
}

static std::ostream &operator<<(std::ostream &out, const openData &d)
{
	out << "[" << ((d.dir==kForward)?"forward":"backward") << ", p:" << +d.priority << ", g:" << +d.gcost;
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct openDataHash
{
	std::size_t operator()(const openData & x) const
	{
		return (x.dir)|(x.priority<<2)|(x.gcost<<8)|(x.bucket<<20);
	}
};

struct openList {
	openList() :f(0) {}
	//std::unordered_set<uint64_t> states;
	FILE *f;
};

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
		if (s.first.dir == kForward && s.first.gcost < minGForward)
			minGForward = s.first.gcost;
		else if (s.first.dir == kBackward && s.first.gcost < minGBackward)
			minGBackward = s.first.gcost;
		
		if (s.first.dir == kForward && s.first.gcost < minFForward)
			minFForward = s.first.gcost;
		else if (s.first.dir == kBackward && s.first.gcost < minFBackward)
			minFBackward = s.first.gcost;
		
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
					if (best.bucket > s.first.bucket)
						best = s.first;
				}
				else if (s.first.dir == kForward)
					best = s.first;
			}
		}
	}
	return best;
}

void GetOpenData(const RubiksState &start, tSearchDirection dir, int cost,
				 openData &d, uint64_t &data)
{
	int bucket;
	//uint64_t data;
	GetBucketAndData(start, bucket, data);
	d.dir = dir;
	d.gcost = cost;
	//d.hcost = (dir==kForward)?forward.HCost(start, start):reverse.HCost(start, start);
	//d.hcost2 = (dir==kForward)?reverse.HCost(start, start):forward.HCost(start, start);
	d.bucket = bucket;
	//d.priority = d.gcost+d.hcost;
	d.priority = d.gcost*2;
}

void AddStatesToQueue(const openData &d, uint64_t *data, size_t count)
{
	openLock.lock();
	if (open.find(d) == open.end())
	{
		open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
		if (open[d].f == 0)
		{
			printf("Error opening %s; Aborting!\n", GetOpenName(d).c_str());
			perror("Reason: ");
			exit(0);
		}
	}
	if (open[d].f == 0)
	{
		printf("Error - file is null!\n");
		exit(0);
	}
	fwrite(data, sizeof(uint64_t), count, open[d].f);
	openLock.unlock();
}

void AddStateToQueue(openData &d, uint64_t data)
{
	AddStatesToQueue(d, &data, 1);
}

void AddStateToQueue(const RubiksState &start, tSearchDirection dir, int cost)
{
	openData d;
	uint64_t rank;
	GetOpenData(start, dir, cost, d, rank);
	AddStateToQueue(d, rank);
}


bool CanTerminateSearch()
{
	int val;
	if (bestSolution <= (val = std::max(currentC, std::max(minFForward, std::max(minFBackward, minGBackward+minGForward+1)))))
	{
		printf("Done!\n");
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
		if (val == minGBackward+minGForward+1)
			printf("-Triggered by gforward+gbackward+1\n");
		return true;
	}
	return false;
}

void CheckSolution(std::unordered_map<openData, openList, openDataHash> currentOpen, openData d,
				   const std::unordered_set<uint64_t> &states)
{
	for (const auto &s : currentOpen)
	{
		// Opposite direction, same bucket AND could be a solution (g+g >= C)
		if (s.first.dir != d.dir && s.first.bucket == d.bucket &&
			d.gcost + s.first.gcost >= currentC)// && d.hcost2 == s.first.hcost)
		{
			const size_t bufferSize = 128;
			uint64_t buffer[bufferSize];
			if (s.second.f == 0)
			{
				std::cout << "Error opening " << s.first << "\n";
				exit(0);
			}
			rewind(s.second.f);
			size_t numRead;
			do {
				numRead = fread(buffer, sizeof(uint64_t), bufferSize, s.second.f);
				for (size_t x = 0; x < numRead; x++)
				{
					if (states.find(buffer[x]) != states.end())
					{
						printLock.lock();
						//s.second.states.find(data) != s.second.states.end()
						printf("\nFound solution cost %d+%d=%d\n", d.gcost, s.first.gcost, d.gcost + s.first.gcost);
						bestSolution = std::min(d.gcost + s.first.gcost, bestSolution);
						printf("Current best solution: %d\n", bestSolution);
						printLock.unlock();
						
						if (CanTerminateSearch())
							return;
					}
				}
			} while (numRead == bufferSize);
		}
	}
}

void ReadBucket(std::unordered_set<uint64_t> &states, openData d)
{
	const size_t bufferSize = 128;
	uint64_t buffer[bufferSize];
	rewind(open[d].f);
	size_t numRead;
	do {
		numRead = fread(buffer, sizeof(uint64_t), bufferSize, open[d].f);
		for (size_t x = 0; x < numRead; x++)
			states.insert(buffer[x]);
	} while (numRead == bufferSize);
	fclose(open[d].f);
	remove(GetOpenName(d).c_str());
	open[d].f = 0;
}

void WriteBucket(std::unordered_set<uint64_t> &states, openData d)
{
	assert(open[d].f == 0);
	open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
	FILE *f = open[d].f;
	for (const uint64_t &s : states)
	{
		fwrite(&s, sizeof(s), 1, f);
	}
}

void RemoveDuplicates(std::unordered_set<uint64_t> &states, openData d)
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
		uint64_t buffer[bufferSize];
		rewind(cd.f);
		size_t numRead;
		do {
			numRead = fread(buffer, sizeof(uint64_t), bufferSize, cd.f);
			for (size_t x = 0; x < numRead; x++)
			{
				auto i = states.find(buffer[x]);
				if (i != states.end())
					states.erase(i);
			}
		} while (numRead == bufferSize);
	}
}

void WriteToClosed(std::unordered_set<uint64_t> &states, openData d)
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
		fwrite(&i, sizeof(uint64_t), 1, cd.f);
	}
}

void ParallelExpandBucket(openData d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads)
{
	const int cacheSize = 1024;
	std::unordered_map<openData, std::vector<uint64_t>, openDataHash> cache;
	RubiksState tmp;
	uint64_t localExpanded = 0;
	int count = 0;
	for (const auto &values : states)
	{
		if (finished)
			break;
		
		count++;
		if (myThread != (count%totalThreads))
			continue;
		
		localExpanded++;
		for (int x = 0; x < 18; x++) // TODO: use getactions
		{
			GetState(tmp, d.bucket, values);
			// copying 2 64-bit values is faster than undoing a move
			cube.ApplyAction(tmp, x);
			openData newData;
			uint64_t newRank;
			GetOpenData(tmp, d.dir, d.gcost+1, newData, newRank);
			
			std::vector<uint64_t> &c = cache[newData];
			c.push_back(newRank);
			if (c.size() > cacheSize)
			{
				AddStatesToQueue(newData, &c[0], c.size());
				c.clear();
			}
		}
	}
	for (auto &i : cache)
	{
		AddStatesToQueue(i.first, &(i.second[0]), i.second.size());
	}
	
	countLock.lock();
	expanded += localExpanded;
	if (d.dir == kForward)
		gDistForward[d.gcost]+=localExpanded;
	else
		gDistBackward[d.gcost]+=localExpanded;
	countLock.unlock();
}

void ExpandNextFile()
{
	// 1. Get next expansion target
	openData d = GetBestFile();
	currentC = d.priority;
	
	if (CanTerminateSearch())
		return;
	
	Timer timer;
	timer.StartTimer();
	
	std::unordered_set<uint64_t> states;
	ReadBucket(states, d);
	//RemoveDuplicates(states, d); // delayed duplicate detection
	WriteToClosed(states, d); // this could run in parallel!
	timer.EndTimer();
	
	printLock.lock();
	std::cout << "Next: " << d << " (" << states.size() << " entries) [" << timer.GetElapsedTime() << "s reading/dd] ";
	printLock.unlock();
	
	timer.StartTimer();
	// Read in opposite buckets to check for solutions in parallel to expanding this bucket
	//openLock.lock();
	//std::thread t(CheckSolution, open, d, std::ref(states));
	//openLock.unlock();
	
	// 3. expand all states in current bucket & write out successors
	const int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (int x = 0; x < numThreads; x++)
		threads.push_back(new std::thread(ParallelExpandBucket, d, std::ref(states), x, numThreads));
	for (size_t x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
	open.erase(open.find(d));
	timer.EndTimer();
	printLock.lock();
	std::cout << "[" << timer.GetElapsedTime() << "s expanding]\n";
	timer.StartTimer();
	printLock.unlock();
//	t.join();
//	printLock.lock();
//	timer.EndTimer();
//	std::cout << "[" << timer.GetElapsedTime() << "s]\n";
//	printLock.unlock();
}

void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result)
{
	ZeroHeuristic<RubiksState> *zero = new ZeroHeuristic<RubiksState>();
	result.lookups.push_back({kLeafNode, 0, 0});
	result.heuristics.push_back(zero);
}

int GetBucket(const RubiksState &s)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	return ehash&bucketMask;
}

void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data)
{
	uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
	bucket = ehash&bucketMask;
	data = (ehash>>bucketBits)*RubikCornerPDB::GetStateSpaceSize()+chash;
}

void GetState(RubiksState &s, int bucket, uint64_t data)
{
	RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
	RubikEdgePDB::GetStateFromHash(s.edge, bucket|((data/RubikCornerPDB::GetStateSpaceSize())<<bucketBits));
}

#pragma mark Main Code

void MM0(RubiksState &start, RubiksState &goal, const char *p1, const char *p2)
{
	prefix1 = p1;
	prefix2 = p2;
	
	bestSolution = 100;
	gDistBackward.resize(12);
	gDistForward.resize(12);
	expanded = 0;
	
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);
	
	Timer t;
	t.StartTimer();
	printf("---MM*---\n");
	AddStateToQueue(start, kForward, 0);
	AddStateToQueue(goal, kBackward, 0);
	openData last = GetBestFile();
	while (!open.empty() && !finished)
	{
		openData curr = GetBestFile();
		if (curr.dir != last.dir)
		{
			for (const auto &item : open)
			{
				if (item.first.dir == last.dir)
				{
					Timer s;
					s.StartTimer();
					printf("DDD/DSD on %s bucket %d g:%d", last.dir==kForward?"forward":"backward", item.first.bucket, item.first.gcost);
					std::unordered_set<uint64_t> states;
					ReadBucket(states, item.first);
					RemoveDuplicates(states, item.first); // DDD
					WriteBucket(states, item.first);
					CheckSolution(open, item.first, std::ref(states)); // DSD
					s.EndTimer();
					printf("[%1.2f]\n", s.GetElapsedTime());
				}
			}
		}
		ExpandNextFile();
		last = curr;
	}
	t.EndTimer();
	printf("%1.2fs elapsed\n", t.GetElapsedTime());
}


NAMESPACE_CLOSE(MM0)