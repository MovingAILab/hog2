//
//  PDBHeuristic.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/19/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_PDBHeuristic_h
#define hog2_glut_PDBHeuristic_h

#include <cassert>
#include <thread>
#include <string>
#include <cinttypes>

#include "Heuristic.h"
#include "SharedQueue.h"
#include "NBitArray.h"
#include "Timer.h"
#include "RangeCompression.h"

enum PDBLookupType {
	kPlain,
	kDivCompress,
//	kFractionalCompress,
//	kFractionalModCompress,
	kModCompress,
	kValueCompress,
	kDivPlusValueCompress,
	kDivPlusDeltaCompress, // two lookups with the same index, one is div, one is delta
	kDefaultHeuristic
};

const int coarseSize = 1024;
const int maxThreads = 64; // TODO: This isn't enforced in a static assert

template <class abstractState, class abstractAction, class abstractEnvironment, class state = abstractState, uint64_t pdbBits = 8>
class PDBHeuristic : public Heuristic<state> {
public:
	PDBHeuristic(abstractEnvironment *e) :type(kPlain), env(e)
	{ goalSet = false; }
	virtual ~PDBHeuristic() {}

	void SetGoal(const state &goal)
	{
		goalState.resize(1);
		GetStateFromPDBHash(this->GetAbstractHash(goal), goalState[0]);
		goalSet = true;
	}

	void SetGoal(const std::vector<state> &goals)
	{
		goalState.resize(0);
		for (auto &i : goals)
		{
			goalState.resize(goalState.size()+1);
			GetStateFromPDBHash(this->GetAbstractHash(i), goalState[goalState.size()-1]);
		}
		goalSet = true;
	}

	
	virtual double HCost(const state &a, const state &b) const;

	virtual uint64_t GetPDBSize() const = 0;

	virtual uint64_t GetPDBHash(const abstractState &s, int threadID = 0) const = 0;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const = 0;
	virtual void GetStateFromPDBHash(uint64_t hash, abstractState &s, int threadID = 0) const = 0;
	virtual state GetStateFromAbstractState(abstractState &s) const = 0;

	virtual bool Load(const char *prefix) = 0;
	virtual void Save(const char *prefix) = 0;
	virtual bool Load(FILE *f);
	virtual void Save(FILE *f);
	virtual std::string GetFileName(const char *prefix) = 0;
	
	/** This methods randomizes the entries in the PDB. Only useful for testing purposes. (eg to test structure in compression) */
	void ShuffleValues();
	void BuildPDB(const state &goal);
	void BuildPDB(const state &goal, int numThreads, bool verbose = true)
	{ BuildPDBForwardBackward(goal, numThreads, verbose); }
	void BuildPDBForward(const state &goal, int numThreads, bool useCoarseOpen = true, bool verbose = false);
	void BuildPDBForward(const std::vector<state> &goal, int numThreads, bool useCoarseOpen = true, bool verbose = false);
	void BuildPDBBackward(const state &goal, int numThreads);
	void BuildPDBForwardBackward(const state &goal, int numThreads, bool verbose = true);

	void BuildAdditivePDB(const state &goal, int numThreads, bool useCourseOpen = true);

	void DivCompress(int factor, bool print_histogram);
	void ModCompress(int factor, bool print_histogram);
	void ModCompress(uint64_t newEntries, bool print_histogram);
	void ZeroLowValues(int limit)
	{ for (uint64_t s = 0; s < PDB.Size(); s++)
		if (PDB.Get(s) < limit) PDB.Set(s, 0); }
	
	void DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram);
	
	void FractionalDivCompress(uint64_t count, bool print_histogram);
	void FractionalModCompress(uint64_t factor, bool print_histogram);
	void ValueCompress(int maxValue, bool print_histogram);
	void ValueCompress(std::vector<int> cutoffs, bool print_histogram);
	void ValueRangeCompress(int numBits, bool print_histogram);
	void CustomValueRangeCompress(std::vector<uint64_t> dist, int numBits, bool print_histogram);

	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 5> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 4> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 3> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 2> *, bool print_histogram);
	void ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 1> *, bool print_histogram);
	
	double PrintHistogram();
	double GetAverageValue();
	void GetHistogram(std::vector<uint64_t> &histogram);
//protected:
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 4>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 3>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 2>;
//	friend class PDBHeuristic<abstractState, abstractAction, abstractEnvironment, abstractState, 1>;

	// holds a Pattern Databases
	NBitArray<pdbBits> PDB;
	int vrcValues[1<<pdbBits];
	PDBLookupType type;
	uint64_t compressionValue;

	abstractEnvironment *env;
	std::vector<abstractState> goalState;
private:
	bool goalSet;
	void AdditiveForwardThreadWorker(int threadNum, int depth,
									 NBitArray<pdbBits> &DB,
									 std::vector<bool> &coarse,
									 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
									 SharedQueue<uint64_t> *results,
									 std::mutex *lock);
	void GetAdditiveNeighbors(const abstractState &s,
							  std::vector<std::pair<abstractState, int>> &neighbors);
	
	void ForwardThreadWorker(int threadNum, int depth,
							 NBitArray<pdbBits> &DB,
							 std::vector<bool> &coarse,
							 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
							 SharedQueue<uint64_t> *results,
							 std::mutex *lock);
	void BackwardThreadWorker(int threadNum, int depth,
							  NBitArray<pdbBits> &DB,
							  std::vector<bool> &coarse,
							  SharedQueue<std::pair<uint64_t, uint64_t> > *work,
							  SharedQueue<uint64_t> *results,
							  std::mutex *lock);
	void ForwardBackwardThreadWorker(int threadNum, int depth, bool forward,
									 NBitArray<pdbBits> &DB,
									 std::vector<bool> &coarseOpen,
									 std::vector<bool> &coarseClosed,
									 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
									 SharedQueue<uint64_t> *results,
									 std::mutex *lock);
};

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::HCost(const state &a, const state &b) const
{
	switch (type)
	{
		case kPlain:
		{
			return PDB.Get(GetAbstractHash(a)); //PDB[GetPDBHash(a)];
		}
		case kDivCompress:
		{
			return PDB.Get(GetAbstractHash(a)/compressionValue);
		}
		case kModCompress:
		{
			return PDB.Get(GetAbstractHash(a)%compressionValue);
		}
		case kValueCompress:
		{
			return vrcValues[PDB.Get(GetAbstractHash(a))]; //PDB[GetPDBHash(a)];
		}
		case kDivPlusValueCompress:
		{
			return vrcValues[PDB.Get(GetAbstractHash(a)/compressionValue)];
		}
		default:
			assert(!"Not implemented");
	}
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDB(const state &goal)
{
	assert(goalSet);
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	uint64_t entries = goalState.size();
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState[0] << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState[0]) << std::endl;
	
	std::vector<abstractAction> acts;
	
	Timer t;
	t.StartTimer();
	for (auto &i : goalState)
		PDB.Set(GetPDBHash(i), 0);
	
	int depth = 0;
	uint64_t newEntries = 0;
	abstractState s(goal), u(goal);
	do {
		Timer timer;
		timer.StartTimer();
		uint64_t total = 0;
		for (uint64_t i = 0; i < COUNT; i++)
		{
			if (PDB.Get(i) == depth)
			{
				GetStateFromPDBHash(i, s);
				env->GetActions(s, acts);
				for (size_t y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], u);
					assert(env->InvertAction(acts[y]) == true);
					
					uint64_t nextRank = GetPDBHash(u);
					int newCost = depth+(env->GCost(u, acts[y]));
					if (PDB.Get(nextRank) > newCost)
					{
						PDB.Set(nextRank, newCost);
						total++;
					}
				}
			}
		}
		
		entries += total;//newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states written; %" PRId64 " of %" PRId64 " total\n",
			   depth, timer.EndTimer(), total, entries, COUNT);
		depth++;
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBForward(const state &goal, int numThreads, bool useCoarseOpen, bool verbose)
{
	std::vector<state> tmp;
	tmp.push_back(goal);
	BuildPDBForward(tmp, numThreads, useCoarseOpen, verbose);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBForward(const std::vector<state> &goal, int numThreads, bool useCoarseOpen, bool verbose)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseOpenCurr((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenNext((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = goalState.size();
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState[0] << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState[0]) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	Timer t;
	t.StartTimer();
	for (auto &i : goalState)
	{
		PDB.Set(GetPDBHash(i), 0);
		coarseOpenCurr[GetPDBHash(i)/coarseSize] = true;
	}

	int depth = 0;
	uint64_t newEntries;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();

		// TODO: clean up interface
		if (!useCoarseOpen)
		{
			uint64_t smallestDepth = (1<<pdbBits)-1;
			for (uint64_t x = 0; x < COUNT; x++)
			{
				uint64_t next = PDB.Get(x);
				if (next >= depth && next < smallestDepth)
					smallestDepth = next;
			}
			depth = smallestDepth;
		}

		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardThreadWorker,
										 this,
										 x, depth, std::ref(PDB), std::ref(coarseOpenNext),
										 &workQueue, &resultQueue, &lock);
		}
		
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			if ((useCoarseOpen && coarseOpenCurr[x/coarseSize]) || !useCoarseOpen)
			{
				workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
			}
			coarseOpenCurr[x/coarseSize] = false;
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		
		entries += total;//newEntries;
		if (verbose)
			printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states written; %" PRId64 " of %" PRId64 " total\n",
				   depth, s.EndTimer(), total, entries, COUNT);
		depth++;
		coarseOpenCurr.swap(coarseOpenNext);
	} while (entries != COUNT);
	
	if (verbose)
		printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		if (verbose)
			printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
		assert(entries == COUNT);
	}
	if (verbose)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBBackward(const state &goal, int numThreads)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseClosed((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = goalState.size();
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState[0] << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState[0]) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	Timer t;
	t.StartTimer();
	for (auto &i : goalState)
		PDB.Set(GetPDBHash(i), 0);
	
	int depth = 0;
	uint64_t newEntries;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BackwardThreadWorker,
										 this,
										 x, depth, std::ref(PDB), std::ref(coarseClosed),
										 &workQueue, &resultQueue, &lock);
		}
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			if (coarseClosed[x/coarseSize] == false)
			{
				workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
			}
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		
		entries += total;//newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states written; %" PRId64 " of %" PRId64 " total\n",
			   depth, s.EndTimer(), total, entries, COUNT);
		depth++;
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildPDBForwardBackward(const state &goal, int numThreads, bool verbose)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseClosed((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenCurr((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenNext((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = goalState.size();
	if (verbose)
	{
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState[0] << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState[0]) << std::endl;
	}
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	std::vector<uint64_t> distribution;

	Timer t;
	t.StartTimer();
	int cnt = 0;
	for (auto &i : goalState)
	{
		PDB.Set(GetPDBHash(i), 0);
		coarseOpenCurr[GetPDBHash(i)/coarseSize] = true;
		cnt++;
	}
	distribution.push_back(cnt);
	
	int depth = 0;
	bool searchForward = true;
	std::vector<std::thread*> threads(numThreads);
	if (verbose)
	printf("Creating %d threads\n", numThreads);
	do {
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardBackwardThreadWorker,
										 this,
										 x, depth, searchForward,
										 std::ref(PDB), std::ref(coarseOpenNext), std::ref(coarseClosed),
										 &workQueue, &resultQueue, &lock);
		}
		if (searchForward)
		{
			for (uint64_t x = 0; x < COUNT; x+=coarseSize)
			{
				if (coarseOpenCurr[x/coarseSize])
				{
					workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
				}
				coarseOpenCurr[x/coarseSize] = false;
			}
		}
		else {
			for (uint64_t x = 0; x < COUNT; x+=coarseSize)
			{
				if (coarseClosed[x/coarseSize] == false)
				{
					workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
				}
			}
		}

		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		entries += total;//newEntries;
		distribution.push_back(total);
		if (verbose)
		printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states written; %" PRId64 " of %" PRId64 " total [%s]\n",
			   depth, s.EndTimer(), total, entries, COUNT, searchForward?"forward":"backward");
		if (double(total)*double(total)*0.4 > double(COUNT-entries)*double(distribution[distribution.size()-2]))// || depth == 8)
			searchForward = false;
		if (COUNT-entries <= total) // If we wrote more entries than there are left, switch directions
			searchForward = false;
		depth++;
		coarseOpenCurr.swap(coarseOpenNext);
	} while (entries != COUNT);
	if (verbose)
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		if (verbose)
		printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
		assert(entries == COUNT);
	}
	if (verbose)
		PrintHistogram();
}


template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardThreadWorker(int threadNum, int depth,
																	 NBitArray<pdbBits> &DB,
																	 std::vector<bool> &coarse,
																	 SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																	 SharedQueue<uint64_t> *results,
																	 std::mutex *lock)
{
//	lock->lock();
//	printf("Thread %d online\n", threadNum);
//	lock->unlock();

	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState[0]), t(goalState[0]);
	uint64_t count = 0;
	
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	while (true)
	{
		work->WaitRemove(p);
		if (p.first == p.second)
		{
			break;
		}
		start = p.first;
		end = p.second;
		//int nextDepth = 255;
		for (uint64_t x = start; x < end; x++)
		{
			int stateDepth = DB.Get(x);
			if (stateDepth == depth)
			{
				GetStateFromPDBHash(x, s, threadNum);
				//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
				env->GetActions(s, acts);
				for (int y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;
					
					uint64_t nextRank = GetPDBHash(t, threadNum);
					int newCost = stateDepth+(env->GCost(t, acts[y]));
					cache.push_back({nextRank, newCost});
				}
			}
		}
		// write out everything
		lock->lock();
		for (auto d : cache)
		{
			if (d.newGCost < DB.Get(d.rank)) // shorter path
			{
				if (DB.Get(d.rank) == (1<<pdbBits)-1)
					count++;
				coarse[d.rank/coarseSize] = true;
				DB.Set(d.rank, d.newGCost);
			}
		}
		lock->unlock();
		cache.resize(0);
	}
	results->Add(count);
//	lock->lock();
//	printf("Thread %d offline\n", threadNum);
//	lock->unlock();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BackwardThreadWorker(int threadNum, int depth,
																											NBitArray<pdbBits> &DB,
																											std::vector<bool> &coarse,
																											SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																											SharedQueue<uint64_t> *results,
																											std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState[0]), t(goalState[0]);
	uint64_t count = 0;
	int blankEntries = 0;
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	while (true)
	{
		work->WaitRemove(p);
		if (p.first == p.second)
		{
			break;
		}
		start = p.first;
		end = p.second;
		//int nextDepth = 255;
		blankEntries = 0;
		for (uint64_t x = start; x < end; x++)
		{
			int stateDepth = DB.Get(x);
			if (stateDepth == ((1<<pdbBits)-1))//depth) // pdbBits
			{
				blankEntries++;
				GetStateFromPDBHash(x, s, threadNum);
				//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
				env->GetActions(s, acts);
				for (int y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					//assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;

					uint64_t nextRank = GetPDBHash(t, threadNum);
					if (DB.Get(nextRank) == depth)
					{
						int newCost = depth+(env->GCost(t, acts[y]));
						cache.push_back({x, newCost});
						blankEntries--;
						break;
					}
				}
			}
		}
		// write out everything
		if (cache.size() > 0)
		{
			//printf("%d items to write\n", cache.size());
			lock->lock();
			if (blankEntries == 0)
				coarse[start/coarseSize] = true; // closed
			for (auto d : cache)
			{
				if (d.newGCost < DB.Get(d.rank)) // shorter path
				{
					count++;
					DB.Set(d.rank, d.newGCost);
				}
			}
			lock->unlock();
		}
		cache.resize(0);
	}
	results->Add(count);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ForwardBackwardThreadWorker(int threadNum, int depth, bool forward,
																												   NBitArray<pdbBits> &DB,
																												   std::vector<bool> &coarseOpen,
																												   std::vector<bool> &coarseClosed,
																												   SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																												   SharedQueue<uint64_t> *results,
																												   std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	abstractState s(goalState[0]), t(goalState[0]);
	uint64_t count = 0;
	int blankEntries = 0;
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	if (forward)
	{
		bool allEntriesWritten;
		while (true)
		{
			work->WaitRemove(p);
			if (p.first == p.second)
			{
				break;
			}
			start = p.first;
			end = p.second;

			allEntriesWritten = true;
			for (uint64_t x = start; x < end; x++)
			{
				int stateDepth = DB.Get(x);
				if (stateDepth > depth)
					allEntriesWritten = false;
				if (stateDepth == depth)
				{
					GetStateFromPDBHash(x, s, threadNum);
					//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
					env->GetActions(s, acts);
					for (size_t y = 0; y < acts.size(); y++)
					{
						env->GetNextState(s, acts[y], t);
						assert(env->InvertAction(acts[y]) == true);
						//virtual bool InvertAction(action &a) const = 0;
						
						uint64_t nextRank = GetPDBHash(t, threadNum);
						int newCost = stateDepth+(env->GCost(t, acts[y]));
						cache.push_back({nextRank, newCost});
					}
				}
			}
			// write out everything
			lock->lock();
			if (allEntriesWritten)
				coarseClosed[start/coarseSize] = true;
			for (auto d : cache)
			{
				if (d.newGCost < DB.Get(d.rank)) // shorter path
				{
					count++;
					coarseOpen[d.rank/coarseSize] = true;
					DB.Set(d.rank, d.newGCost);
				}
			}
			lock->unlock();
			cache.resize(0);
		}
	}
	else {
		while (true)
		{
			work->WaitRemove(p);
			if (p.first == p.second)
			{
				break;
			}
			start = p.first;
			end = p.second;
			//int nextDepth = 255;
			blankEntries = 0;
			for (uint64_t x = start; x < end; x++)
			{
				int stateDepth = DB.Get(x);
				if (stateDepth == ((1<<pdbBits)-1))//depth) // pdbBits
				{
					blankEntries++;
					GetStateFromPDBHash(x, s, threadNum);
					//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
					env->GetActions(s, acts);
					for (size_t y = 0; y < acts.size(); y++)
					{
						env->GetNextState(s, acts[y], t);
						//assert(env->InvertAction(acts[y]) == true);
						//virtual bool InvertAction(action &a) const = 0;
						
						uint64_t nextRank = GetPDBHash(t, threadNum);
						if (DB.Get(nextRank) == depth)
						{
							int newCost = depth+(env->GCost(t, acts[y]));
							cache.push_back({x, newCost});
							blankEntries--;
							break;
						}
					}
				}
			}
			// write out everything
			if (cache.size() > 0)
			{
				//printf("%d items to write\n", cache.size());
				lock->lock();
				if (blankEntries == 0)
					coarseClosed[start/coarseSize] = true; // closed
				for (auto d : cache)
				{
					if (d.newGCost < DB.Get(d.rank)) // shorter path
					{
						count++;
						DB.Set(d.rank, d.newGCost);
					}
				}
				lock->unlock();
			}
			cache.resize(0);
		}
	}
	results->Add(count);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::BuildAdditivePDB(const state &goal, int numThreads, bool useCourseOpen)
{
	assert(goalSet);
	SharedQueue<std::pair<uint64_t, uint64_t> > workQueue(numThreads*20);
	SharedQueue<uint64_t> resultQueue;
	std::mutex lock;
	
	uint64_t COUNT = GetPDBSize();
	PDB.Resize(COUNT);
	PDB.FillMax();
	
	// with weights we have to store the lowest weight stored to make sure
	// we don't skip regions
	std::vector<bool> coarseOpenCurr((COUNT+coarseSize-1)/coarseSize);
	std::vector<bool> coarseOpenNext((COUNT+coarseSize-1)/coarseSize);
	
	uint64_t entries = goalState.size();
	std::cout << "Num Entries: " << COUNT << std::endl;
	std::cout << "Goal State: " << goalState[0] << std::endl;
	//std::cout << "State Hash of Goal: " << GetStateHash(goal) << std::endl;
	std::cout << "PDB Hash of Goal: " << GetPDBHash(goalState[0]) << std::endl;
	
	std::deque<state> q_curr, q_next;
	std::vector<state> children;
	
	Timer t;
	t.StartTimer();
	for (auto &i : goalState)
	{
		PDB.Set(GetPDBHash(i), 0);
		coarseOpenCurr[GetPDBHash(i)/coarseSize] = true;
	}
	int depth = 0;
	uint64_t newEntries;
	std::vector<std::thread*> threads(numThreads);
	printf("Creating %d threads\n", numThreads);
	do {
		newEntries = 0;
		Timer s;
		s.StartTimer();
		for (int x = 0; x < numThreads; x++)
		{
			threads[x] = new std::thread(&PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::AdditiveForwardThreadWorker,
										 this,
										 x, depth, std::ref(PDB), std::ref(coarseOpenNext),
										 &workQueue, &resultQueue, &lock);
		}
		
		for (uint64_t x = 0; x < COUNT; x+=coarseSize)
		{
			if (!useCourseOpen || coarseOpenCurr[x/coarseSize])
			{
				workQueue.WaitAdd({x, std::min(COUNT, x+coarseSize)});
			}
			coarseOpenCurr[x/coarseSize] = false;
		}
		for (int x = 0; x < numThreads; x++)
		{
			workQueue.WaitAdd({0,0});
		}
		for (int x = 0; x < numThreads; x++)
		{
			threads[x]->join();
			delete threads[x];
			threads[x] = 0;
		}
		// read out node counts
		uint64_t total = 0;
		{
			uint64_t val;
			while (resultQueue.Remove(val))
			{
				total+=val;
			}
		}
		
		entries += total;//newEntries;
		printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states written; %" PRId64 " of %" PRId64 " total\n",
			   depth, s.EndTimer(), total, entries, COUNT);
		depth++;
		coarseOpenCurr.swap(coarseOpenNext);

//		if (total == 0)
//		{
//			for (int x = 0; x < PDB.Size(); x++)
//			{
//				if (PDB.Get(x) == PDB.GetMaxValue())
//				{
//					abstractState s(goalState[0]);
//					GetStateFromPDBHash(x, s, 0);
//					std::cout << "Unassigned: [" << x << "]: " << s << "\n";
//				}
//			}
//			break;
//		}
	} while (entries != COUNT);
	
	printf("%1.2fs elapsed\n", t.EndTimer());
	if (entries != COUNT)
	{
		printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
		assert(entries == COUNT);
	}
	PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::AdditiveForwardThreadWorker(int threadNum, int depth,
																												   NBitArray<pdbBits> &DB,
																												   std::vector<bool> &coarse,
																												   SharedQueue<std::pair<uint64_t, uint64_t> > *work,
																												   SharedQueue<uint64_t> *results,
																												   std::mutex *lock)
{
	std::pair<uint64_t, uint64_t> p;
	uint64_t start, end;
	std::vector<abstractAction> acts;
	std::vector<std::pair<abstractState, int>> neighbors;
	abstractState s(goalState[0]), t(goalState[0]);
	uint64_t count = 0;
	
	struct writeInfo {
		uint64_t rank;
		int newGCost;
	};
	std::vector<writeInfo> cache;
	std::vector<uint64_t> zeroCostNeighbors;
	while (true)
	{
		work->WaitRemove(p);
		if (p.first == p.second)
		{
			break;
		}
		start = p.first;
		end = p.second;
		for (uint64_t x = start; x < end; x++)
		{
			int stateDepth = DB.Get(x);
			if (stateDepth == depth)
			{
				GetStateFromPDBHash(x, s, threadNum);
				//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
				env->GetActions(s, acts);

				for (size_t y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;
					
					uint64_t nextRank = GetPDBHash(t, threadNum);
					int newCost = stateDepth+(env->AdditiveGCost(t, acts[y]));
					cache.push_back({nextRank, newCost});
				}
			}
		}
		// write out everything
		do {
			lock->lock();
			for (auto d : cache)
			{
				auto val = DB.Get(d.rank);
				if (d.newGCost < val) // shorter path
				{
					if (val == DB.GetMaxValue()) // only update count if the value hasn't been set before
						count++;
					coarse[d.rank/coarseSize] = true;
					DB.Set(d.rank, d.newGCost);
					if (d.newGCost == depth) // zero-cost action
						zeroCostNeighbors.push_back(d.rank);
				}
			}
			lock->unlock();
			cache.resize(0);
			
			for (auto i : zeroCostNeighbors)
			{
				GetStateFromPDBHash(i, s, threadNum);
				env->GetActions(s, acts);
				
				for (size_t y = 0; y < acts.size(); y++)
				{
					env->GetNextState(s, acts[y], t);
					assert(env->InvertAction(acts[y]) == true);
					//virtual bool InvertAction(action &a) const = 0;
					
					uint64_t nextRank = GetPDBHash(t, threadNum);
					int newCost = depth+(env->AdditiveGCost(t, acts[y]));
					cache.push_back({nextRank, newCost});
				}

			}
			zeroCostNeighbors.clear();
		} while (cache.size() > 0);
	}
	results->Add(count);
}

//template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
//void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::DivCompressHelper(NBitArray<pdbBits> &original,
//																										 NBitArray<pdbBits> &copy)
//{
//	
//}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::DivCompress(int factor, bool print_histogram)
{
	if (type != kPlain)
		return;
	type = kDivCompress;
	compressionValue = factor;
	NBitArray<pdbBits> copy(PDB);
	PDB.Resize((PDB.Size()+compressionValue-1)/compressionValue);
	PDB.FillMax();
	for (uint64_t x = 0; x < copy.Size(); x++)
	{
		uint64_t newIndex = x/compressionValue;
		PDB.Set(newIndex, std::min(copy.Get(x), PDB.Get(newIndex)));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ModCompress(int factor, bool print_histogram)
{
	ModCompress((PDB.Size()+factor-1)/factor, print_histogram);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ModCompress(uint64_t newEntries, bool print_histogram)
{
	type = kModCompress;
	compressionValue = newEntries;
	NBitArray<pdbBits> copy(PDB);
	PDB.Resize(compressionValue);
	PDB.FillMax();
	for (uint64_t x = 0; x < copy.Size(); x++)
	{
		uint64_t newIndex = x%compressionValue;
		PDB.Set(newIndex, std::min(copy.Get(x), PDB.Get(newIndex)));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::DeltaCompress(Heuristic<state> *h, state goal, bool print_histogram)
{
	abstractState s;
	state s1;
	for (size_t x = 0; x < PDB.Size(); x++)
	{
		GetStateFromPDBHash(x, s);
		s1 = GetStateFromAbstractState(s);
		PDB.Set(x, PDB.Get(x) - h->HCost(s1, goal));
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::FractionalDivCompress(uint64_t count, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::FractionalModCompress(uint64_t factor, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueCompress(int maxValue, bool print_histogram)
{
	assert(!"Not currently implemented.");
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueCompress(std::vector<int> cutoffs, bool print_histogram)
{
	std::vector<uint64_t> dist;
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", cutoffs.size());
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::CustomValueRangeCompress(std::vector<uint64_t> dist, int numBits, bool print_histogram)
{
	std::vector<int> cutoffs;
	GetOptimizedBoundaries(dist, 1<<numBits, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<numBits));
		for (size_t x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (size_t y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(int numBits, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<numBits, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<numBits));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	cutoffs.push_back(256);
	
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				PDB.Set(x, cutoffs[y]);
				break;
			}
		}
	}
	if (print_histogram)
		PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(
																										  PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 5> *newPDB,
																										  bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<5, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}
	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(
												PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 4> *newPDB,
																										   bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<4, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}
	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 3> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<3, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (int x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (int y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();

}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 2> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<2, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (size_t x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (size_t x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (size_t y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
	
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ValueRangeCompress(PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, 1> *newPDB, bool print_histogram)
{
	std::vector<uint64_t> dist;
	std::vector<int> cutoffs;
	GetHistogram(dist);
	GetOptimizedBoundaries(dist, 1<<1, cutoffs);
	if (print_histogram)
	{
		printf("Setting boundaries [%d values]: ", (1<<4));
		for (size_t x = 0; x < cutoffs.size(); x++)
			printf("%d ", cutoffs[x]);
		printf("\n");
	}
	if (type == kPlain)
		newPDB->type = kValueCompress;
	else if (type == kDivCompress)
	{
		newPDB->type = kDivPlusValueCompress;
		newPDB->compressionValue = compressionValue;
	}
	else {
		printf("Unknown PDB type: %d\n", type);
	}

	for (int x = 0; x < cutoffs.size(); x++)
		newPDB->vrcValues[x] = cutoffs[x];
	newPDB->PDB.Resize(PDB.Size());
	cutoffs.push_back(256);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		for (size_t y = 0; y < cutoffs.size(); y++)
		{
			if (PDB.Get(x) >= cutoffs[y] && PDB.Get(x) < cutoffs[y+1])
			{
				//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
				newPDB->PDB.Set(x, y);
				break;
			}
		}
	}
	if (print_histogram)
		newPDB->PrintHistogram();
	
}


template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
bool PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::Load(FILE *f)
{
	if (fread(&type, sizeof(type), 1, f) != 1)
		return false;
	goalState.resize(0);
	if (fread(&goalState[0], sizeof(goalState[0]), 1, f) != 1)
		return false;
	return PDB.Read(f);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::Save(FILE *f)
{
	fwrite(&type, sizeof(type), 1, f);
	fwrite(&goalState[0], sizeof(goalState[0]), 1, f);
	PDB.Write(f);
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::GetHistogram(std::vector<uint64_t> &histogram)
{
	histogram.resize(0);
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		if (PDB.Get(x)+1 > histogram.size())
		{
			histogram.resize(PDB.Get(x)+1);
		}
		histogram[PDB.Get(x)]++;
	}
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::PrintHistogram()
{
	int factor = 1;
	if (type == kDivCompress || type == kDivPlusValueCompress)
		factor = compressionValue;
	double average = 0;
	std::vector<uint64_t> histogram;
	if (type == kValueCompress || type == kDivPlusValueCompress)
	{
		for (uint64_t x = 0; x < PDB.Size(); x++)
		{
			if (vrcValues[PDB.Get(x)]+1 > histogram.size())
			{
				histogram.resize(vrcValues[PDB.Get(x)]+1);
			}
			histogram[vrcValues[PDB.Get(x)]]+=factor;
			average += vrcValues[PDB.Get(x)];
		}
	}
	else {
		for (uint64_t x = 0; x < PDB.Size(); x++)
		{
			if (PDB.Get(x)+1 > histogram.size())
			{
				histogram.resize(PDB.Get(x)+1);
			}
			histogram[PDB.Get(x)]+=factor;
			average += PDB.Get(x);
		}
	}
	for (size_t x = 0; x < histogram.size(); x++)
	{
		if (histogram[x] > 0)
			printf("%d: %" PRId64 "\n", (int)x, histogram[x]);
	}
	printf("Average: %f; count: %" PRId64 "\n", average/PDB.Size(), PDB.Size());
	return average/PDB.Size();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
double PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::GetAverageValue()
{
	double average = 0;
	for (uint64_t x = 0; x < PDB.Size(); x++)
	{
		average += PDB.Get(x);
	}
	return average/PDB.Size();
}

template <class abstractState, class abstractAction, class abstractEnvironment, class state, uint64_t pdbBits>
void PDBHeuristic<abstractState, abstractAction, abstractEnvironment, state, pdbBits>::ShuffleValues()
{
	for (uint64_t x = PDB.Size(); x > 0; x--)
	{
		uint64_t index = (((uint64_t)random()<<32)^(uint64_t)random())%(x);
		uint64_t tmp = PDB.Get(x-1);
		PDB.Set(x-1, PDB.Get(index));
		PDB.Set(index, tmp);
	}
}

#endif
