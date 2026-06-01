#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stdint.h>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <deque>
#include "SearchEnvironment.h"
#include "Timer.h"
#include "SharedQueue.h"
#include "RangeCompression.h"
#include "MR1Permutation.h"

#ifndef PERMPUZZ_H
#define PERMPUZZ_H

#pragma mark -
#pragma mark Declarations
#pragma mark -

namespace PermutationPuzzle {
	
	template <class state, class action>
	class PermutationPuzzleEnvironment;
	
	enum PDBTreeNodeType {
		kMaxNode,
		kAddNode,
		kLeafNode,
		kLeafMinCompress,
		kLeafFractionalCompress,
		kLeafFractionalModCompress,
		kLeafModCompress,
		kLeafValueCompress,
		kLeafDivPlusDeltaCompress, // two lookups with the same index, one is div, one is delta
		kLeafDefaultHeuristic
	};
	
	struct PDBTreeNode
	{
		PDBTreeNodeType t;
		uint8_t numChildren;
		uint8_t firstChildID;
		uint8_t PDBID;
	};
	
	const uint64_t kDone = -1;
	
	/**
	 Note, assumes that state has a public vector<int> called puzzle in which the
	 permutation is held.
	 **/
	template <class state, class action>
	class PermutationPuzzleEnvironment : public SearchEnvironment<state, action>
	{
	public:
		PermutationPuzzleEnvironment()
		:maxItem(0), minPattern(100), additive(false) {}
		
		double HCost(const state &s) const;
		virtual double AdditiveGCost(const state &s, const action &d)
		{ assert(!"Additive Gost used but not defined for this class\n"); }
		
		void GetStateFromHash(state &s, uint64_t hash) const;
		uint64_t GetStateHash(const state &s) const;
		static uint64_t Hash(const state &s)
		{
			MR1KPermutation mr1;
			state tmp = s;
			state dual;
			for (size_t x = 0; x < tmp.size(); x++)
				dual.puzzle[tmp.puzzle[x]] = x;
			return mr1.Rank(tmp.puzzle, dual.puzzle, s.size(), s.size());
		}

		state TranformToStandardGoal(const state &a, const state &b) const;
		virtual void FinishUnranking(state &s) const {}
//		void PrintPDBHistogram(int which) const;
//		void GetPDBHistogram(int which, std::vector<uint64_t> &values) const;
		
		static bool Check_Permutation(const std::vector<int> &to_check);
		static bool Output_Puzzles(std::vector<state> &puzzle_vector, bool write_puzz_num);
		static bool Read_In_Permutations(const char *filename, unsigned size, unsigned max_puzzles,
										 std::vector<std::vector<int> > &permutations, bool puzz_num_start);
		static std::vector<int> Get_Random_Permutation(unsigned size);
		
		
	private:
//		void DeltaWorker(std::vector<uint8_t> *array,
//						 std::vector<int> *distinct,
//						 int puzzleSize,
//						 uint64_t start, uint64_t end);
//		void ThreadWorker(int depth, int totalTiles,
//						  std::vector<uint8_t> *DB,
//						  //std::vector<uint8_t> *coarseOpen,
//						  const std::vector<int> *distinct,
//						  SharedQueue<std::pair<uint64_t, uint64_t> > *work,
//						  SharedQueue<uint64_t> *results,
//						  std::mutex *lock,
//						  bool additive);
//		double HCost(const state &s, int treeNode,
//					 std::vector<int> &c1, std::vector<int> &c2) const;
		virtual double DefaultH(const state &s) const { return 0; }
		void buildCaches() const;
		
	protected:
		uint64_t Factorial(int val) const;
		uint64_t nUpperk(int n, int k) const;
		
		
		/**
		 Checks that the given state is a valid state for this domain. Note, is
		 not a check of solvability, just legality (ie. has the right size, dimensions).
		 Note, this method should be a quick check that can be used for debugging
		 purposes and so should not call Check_Permutation which is O(state size)
		 **/
		virtual bool State_Check(const state &to_check) = 0;
		
	public:
		bool Validate_Problems(std::vector<state> &puzzles);
		
		bool additive;
		int maxItem, minPattern;
		// holds a set of Pattern Databases which can be maxed over later
		std::vector<std::vector<uint8_t> > PDB;
		// holds the set of distinct items used to build the associated PDB (and therefore needed for hashing)
		std::vector<std::vector<int> > PDB_distincts;
		std::vector<PDBTreeNode> lookups;
		mutable std::vector<std::vector<uint64_t> > factorialCache;

		MR1KPermutation mr1;
	};
	
	template <typename state, typename environment>
	class ArbitraryGoalPermutation : public Heuristic<state> {
	public:
		ArbitraryGoalPermutation(Heuristic<state> *h, environment *env) { this->h = h; e = env; }
		virtual double HCost(const state &a, const state &b) const
		{ state g; return h->HCost(e->TranformToStandardGoal(a, b), g); }
	private:
		Heuristic<state> *h;
		environment *e;
	};

//#pragma mark -
//#pragma mark Implementation - In-Memory Operations
//#pragma mark -
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Min_Compress_PDB(int whichPDB, int factor, bool print_histogram)
//	{
//		printf("Performing min compression, reducing from %lu entries to %lu entries\n",
//			   PDB[whichPDB].size(), (PDB[whichPDB].size()+factor-1)/factor);
//		std::vector<uint8_t> newPDB((PDB[whichPDB].size()+factor-1)/factor);
//		for (uint64_t x = 0; x < PDB[whichPDB].size(); x+=factor)
//		{
//			int minVal = PDB[whichPDB][x];
//			for (uint64_t y = 1; y < factor; y++)
//			{
//				if (x+y < PDB[whichPDB].size())
//					minVal = min(minVal, PDB[whichPDB][x+y]);
//			}
//			newPDB[x/factor] = minVal;
//		}
//		PDB[whichPDB].swap(newPDB);
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Fractional_Compress_PDB(int whichPDB, uint64_t count, bool print_histogram)
//	{
//		PDB[whichPDB].resize(count);
//	}
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Fractional_Mod_Compress_PDB(int whichPDB, uint64_t factor,
//																				  bool print_histogram)
//	{
//		std::vector<uint8_t> newPDB(PDB[whichPDB].size()/factor);
//		for (int x = 0; x < PDB[whichPDB].size(); x+= factor)
//		{
//			newPDB[x/factor] = PDB[whichPDB][x];
//		}
//		PDB[whichPDB].swap(newPDB);
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Mod_Compress_PDB(int whichPDB, uint64_t newEntries, bool print_histogram)
//	{
//		if (newEntries == 0)
//		{
//			printf("Error -- cannot reduce to 0 entries\n");
//			return;
//		}
//		printf("Performing mod compression, reducing from %lu entries to %" PRId64 " entries\n", PDB[whichPDB].size(), newEntries);
//		std::vector<uint8_t> newPDB(newEntries);
//		for (uint64_t x = 0; x < newEntries; x++)
//			newPDB[x] = PDB[whichPDB][x];
//		for (uint64_t x = newEntries; x < PDB[whichPDB].size(); x++)
//			newPDB[x%newEntries] = min(newPDB[x%newEntries], PDB[whichPDB][x]);
//		PDB[whichPDB].swap(newPDB);
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Value_Compress_PDB(int whichPDB, int maxValue, bool print_histogram)
//	{
//		for (uint64_t x = 0; x < PDB[whichPDB].size(); x++)
//			if (PDB[whichPDB][x] > maxValue)
//				PDB[whichPDB][x] = maxValue;
//		
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Value_Range_Compress_PDB(int whichPDB, int numBits, bool print_histogram)
//	{
//		std::vector<uint64_t> dist;
//		std::vector<int> cutoffs;
//		GetPDBHistogram(whichPDB, dist);
//		GetOptimizedBoundaries(dist, 1<<numBits, cutoffs);
//		printf("Setting boundaries [%d values]: ", (1<<numBits));
//		for (int x = 0; x < cutoffs.size(); x++)
//			printf("%d ", cutoffs[x]);
//		printf("\n");
//		cutoffs.push_back(256);
//		
//		for (uint64_t x = 0; x < PDB[whichPDB].size(); x++)
//		{
//			for (int y = 0; y < cutoffs.size(); y++)
//			{
//				if (PDB[whichPDB][x] >= cutoffs[y] && PDB[whichPDB][x] < cutoffs[y+1])
//				{
//					//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
//					PDB[whichPDB][x] = cutoffs[y];
//					break;
//				}
//			}
//		}
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Value_Compress_PDB(int whichPDB, std::vector<int> cutoffs, bool print_histogram)
//	{
//		
//		for (uint64_t x = 0; x < PDB[whichPDB].size(); x++)
//		{
//			for (int y = 0; y < cutoffs.size(); y++)
//			{
//				if (PDB[whichPDB][x] >= cutoffs[y] && PDB[whichPDB][x] < cutoffs[y+1])
//				{
//					//printf("%d -> %d\n", PDB[whichPDB][x], cutoffs[y]);
//					PDB[whichPDB][x] = cutoffs[y];
//					break;
//				}
//			}
//		}
//		
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Delta_Compress_PDB(state goal, int whichPDB, bool print_histogram)
//	{
//		Timer t;
//		t.StartTimer();
//		uint64_t COUNT = PDB[whichPDB].size();
//		if (1) // use threads
//		{
//			printf("Starting threaded delta computation (%" PRId64 " entries)\n", COUNT);
//			int numThreads = std::thread::hardware_concurrency();
//			std::vector<std::thread*> threads(numThreads);
//			uint64_t workSize = COUNT/numThreads+1;
//			uint64_t start = 0;
//			for (int x = 0; x < numThreads; x++)
//			{
//				threads[x] = new std::thread(&PermutationPuzzleEnvironment<state, action>::DeltaWorker, this,
//											 &PDB[whichPDB], &PDB_distincts[whichPDB], goal.puzzle.size(), start, min(start+workSize, COUNT));
//				start += workSize;
//			}
//			for (int x = 0; x < numThreads; x++)
//			{
//				threads[x]->join();
//				delete threads[x];
//				threads[x] = 0;
//			}
//		}
//		else {
//			printf("Starting sequential delta computation\n");
//			state tmp;
//			for (uint64_t x = 0; x < COUNT; x++)
//			{
//				GetStateFromPDBHash(x, tmp, goal.puzzle.size(), PDB_distincts[whichPDB]);
//				int h1 = HCost(tmp);
//				int h2 = PDB.back()[x];
//				//		if ((h1%2)||(h2%2))
//				//			printf("%d - %d = %d\n", h2, h1, h2-h1);
//				PDB.back()[x] = h2 - h1;
//			}
//		}
//		printf("%1.2fs doing delta conversion\n", t.EndTimer());
//		
//		if (print_histogram)
//			PrintPDBHistogram(whichPDB);
//	}
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::DeltaWorker(std::vector<uint8_t> *array,
//																  std::vector<int> *distinct,
//																  int puzzleSize,
//																  uint64_t start, uint64_t end)
//	{
//		std::vector<int> dual;
//		std::vector<int> c1;
//		std::vector<int> c2;
//		state tmp;
//		for (uint64_t x = start; x < end; x++)
//		{
//			GetStateFromPDBHash(x, tmp, puzzleSize, *distinct, dual);
//			int h1 = HCost(tmp, 0, c1, c2);
//			int h2 = (*array)[x];
//			(*array)[x] = h2 - h1;
//			assert(h2 >= h1);
//		}
//	}
//	
//	
//#pragma mark -
//#pragma mark Implementation - Buildling PDBs
//#pragma mark -
//	
//	const int coarseSize = 1024;
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::ThreadWorker(int depth, int totalTiles,
//																   std::vector<uint8_t> *DB,
//																   //std::vector<uint8_t> *coarseOpen,
//																   const std::vector<int> *distinct,
//																   SharedQueue<std::pair<uint64_t, uint64_t> > *work,
//																   SharedQueue<uint64_t> *results,
//																   std::mutex *lock,
//																   bool additive)
//	{
//		std::pair<uint64_t, uint64_t> p;
//		std::vector<uint64_t> additiveQueue;
//		std::vector<int> cache1;
//		std::vector<int> cache2;
//		uint64_t start, end;
//		std::vector<action> acts;
//		state s, t;
//		uint64_t count = 0;
//		
//		struct writeInfo {
//			uint64_t rank;
//			int newGCost;
//		};
//		std::vector<writeInfo> cache;
//		while (true)
//		{
//			if (work->Remove(p) == false)
//			{
//				std::this_thread::sleep_for(std::chrono::microseconds(10));
//				continue;
//			}
//			if (p.first == p.second)
//			{
//				break;
//			}
//			start = p.first;
//			end = p.second;
//			//int nextDepth = 255;
//			for (uint64_t x = start; x < end; x++)
//			{
//				int stateDepth = (*DB)[x];
//				if (stateDepth == depth)
//				{
//					count++;
//					GetStateFromPDBHash(x, s, totalTiles, *distinct, cache1);
//					//std::cout << "Expanding[r][" << stateDepth << "]: " << s << std::endl;
//					this->GetActions(s, acts);
//					for (int y = 0; y < acts.size(); y++)
//					{
//						this->GetNextState(s, acts[y], t);
//						assert(this->InvertAction(acts[y]) == true);
//						//virtual bool InvertAction(action &a) const = 0;
//						
//						uint64_t nextRank = GetPDBHash(t, *distinct, cache1, cache2);
//						int newCost = stateDepth+(additive?this->AdditiveGCost(t, acts[y]):this->GCost(t, acts[y]));
//						cache.push_back({nextRank, newCost});
//					}
//				}
//			}
//			do {
//				// write out everything
//				lock->lock();
//				for (auto d : cache)
//				{
//					if (d.newGCost < (*DB)[d.rank]) // shorter path
//					{
//						(*DB)[d.rank] = d.newGCost;
//						if (d.newGCost == depth) // 0-cost action; will expand immediately
//						{
//							additiveQueue.push_back(d.rank);
//						}
//					}
//				}
//				lock->unlock();
//				cache.resize(0);
//				
//				while (additiveQueue.size() > 0)
//				{
//					uint64_t x = additiveQueue.back();
//					additiveQueue.pop_back();
//					int stateDepth = (*DB)[x];
//					assert(stateDepth == depth);
//					
//					count++;
//					GetStateFromPDBHash(x, s, totalTiles, *distinct, cache1);
//					//std::cout << "Expanding[a][" << stateDepth << "]: " << s << std::endl;
//					this->GetActions(s, acts);
//					for (int y = 0; y < acts.size(); y++)
//					{
//						this->GetNextState(s, acts[y], t);
//						assert(this->InvertAction(acts[y]) == true);
//						//virtual bool InvertAction(action &a) const = 0;
//						
//						uint64_t nextRank = GetPDBHash(t, *distinct, cache1, cache2);
//						int newCost = stateDepth+(additive?this->AdditiveGCost(t, acts[y]):this->GCost(t, acts[y]));
//						cache.push_back({nextRank, newCost});
//					}
//				}
//				
//			} while (cache.size() != 0);
//		}
//		results->Add(count);
//	}
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Build_PDB(state &start, const std::vector<int> &distinct,
//																const char *pdb_filename, int numThreads, bool additive)
//	{
//		SharedQueue<std::pair<uint64_t, uint64_t> > workQueue;
//		SharedQueue<uint64_t> resultQueue;
//		std::mutex lock;
//		
//		maxItem = max(maxItem,start.puzzle.size());
//		minPattern = min(minPattern, distinct.size());
//		buildCaches();
//		
//		uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
//		std::vector<uint8_t> DB(COUNT);
//		
//		std::fill(DB.begin(), DB.end(), 255);
//		
//		// with weights we have to store the lowest weight stored to make sure
//		// we don't skip regions
//		std::vector<uint8_t> coarseOpen((COUNT+coarseSize-1)/coarseSize);
//		std::fill(coarseOpen.begin(), coarseOpen.end(), 255);
//		
//		uint64_t entries = 0;
//		std::cout << "Num Entries: " << COUNT << std::endl;
//		std::cout << "Goal State: " << start << std::endl;
//		std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
//		std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		
//		std::deque<state> q_curr, q_next;
//		std::vector<state> children;
//		
//		for (unsigned i = 0; i < start.puzzle.size(); i++)
//		{
//			bool is_distinct = false;
//			for (unsigned j = 0; j < distinct.size(); j++)
//			{
//				if (start.puzzle[i] == distinct[j])
//				{
//					is_distinct = true;
//					break;
//				}
//			}
//			
//			if (!is_distinct)
//			{
//				start.puzzle[i] = -1;
//			}
//		}
//		
//		std::cout << "Abstract Goal State: " << start << std::endl;
//		std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		Timer t;
//		t.StartTimer();
//		//	q_curr.push_back(start);
//		DB[GetPDBHash(start, distinct)] = 0;
//		coarseOpen[GetPDBHash(start, distinct)/coarseSize] = 0;
//		int depth = 0;
//		uint64_t newEntries;
//		std::vector<std::thread*> threads(numThreads);
//		printf("Creating %d threads\n", numThreads);
//		do {
//			newEntries = 0;
//			Timer s;
//			s.StartTimer();
//			for (int x = 0; x < numThreads; x++)
//			{
//				threads[x] = new std::thread(&PermutationPuzzleEnvironment<state, action>::ThreadWorker, this,
//											 depth, start.puzzle.size(), &DB,// &coarseOpen,
//											 &distinct,
//											 &workQueue, &resultQueue, &lock, additive);
//			}
//			
//			for (uint64_t x = 0; x < COUNT; x+=coarseSize)
//			{
//				bool submit = false;
//				for (uint64_t t = x; t < min(COUNT, x+coarseSize); t++)
//				{
//					if (DB[t] == depth)
//					{
//						submit = true;
//						//newEntries++;
//					}
//				}
//				if (submit)
//				{
//					while (workQueue.size() > 10*numThreads)
//					{ std::this_thread::sleep_for(std::chrono::microseconds(10)); }
//					workQueue.Add({x, min(COUNT, x+coarseSize)});
//				}
//			}
//			for (int x = 0; x < numThreads; x++)
//			{
//				workQueue.Add({0,0});
//			}
//			for (int x = 0; x < numThreads; x++)
//			{
//				threads[x]->join();
//				delete threads[x];
//				threads[x] = 0;
//			}
//			// read out node counts
//			while (true)
//			{
//				uint64_t val;
//				if (resultQueue.Remove(val))
//				{
//					//newEntries+=val;
//				}
//				else {
//					break;
//				}
//			}
//			
//			newEntries = 0;
//			for (uint64_t x = 0; x < COUNT; x++)
//			{
//				if (DB[x] == depth)
//				{
//					newEntries++;
//				}
//			}
//			entries += newEntries;
//			printf("Depth %d complete; %1.2fs elapsed. %" PRId64 " new states seen; %" PRId64 " of %" PRId64 " total\n",
//				   depth, s.EndTimer(), newEntries, entries, COUNT);
//			depth++;
//			//		depth = coarseOpen[0];
//			//		for (int x = 1; x < coarseOpen.size(); x++)
//			//			depth = min(depth, coarseOpen[x]);
//			//		if (depth == 255) // no new entries!
//			//			break;
//		} while (entries != COUNT);
//		
//		printf("%1.2fs elapsed\n", t.EndTimer());
//		if (entries != COUNT)
//		{
//			printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
//			assert(entries == COUNT);
//		}
//		
//		//TODO fix the output of PDBs
//		FILE *f = fopen(pdb_filename, "w");
//		int num = distinct.size();
//		assert(fwrite(&num, sizeof(num), 1, f) == 1);
//		assert(fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
//		assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//		fclose(f);
//		
//		PDB.push_back(DB); // increase the number of regular PDBs being stored
//		PDB_distincts.push_back(distinct); // stores distinct
//		PrintPDBHistogram(PDB.size()-1);
//	}
//	
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Build_Regular_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename)
//	{
//		maxItem = max(maxItem,start.puzzle.size());
//		minPattern = min(minPattern, distinct.size());
//		buildCaches();
//		
//		uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
//		std::vector<uint8_t> DB(COUNT);
//		
//		for (unsigned int x = 0; x < DB.size(); x++)
//			DB[x] = 255;
//		
//		uint64_t entries = 0;
//		std::cout << "Num Entries: " << COUNT << std::endl;
//		std::cout << "Goal State: " << start << std::endl;
//		std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
//		std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		
//		std::deque<state> q_curr, q_next;
//		std::vector<state> children;
//		
//		for (unsigned i = 0; i < start.puzzle.size(); i++)
//		{
//			bool is_distinct = false;
//			for (unsigned j = 0; j < distinct.size(); j++)
//			{
//				if (start.puzzle[i] == distinct[j])
//				{
//					is_distinct = true;
//					break;
//				}
//			}
//			
//			if (!is_distinct)
//			{
//				start.puzzle[i] = -1;
//			}
//		}
//		
//		std::cout << "Abstract Goal State: " << start << std::endl;
//		std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		Timer t;
//		t.StartTimer();
//		q_curr.push_back(start);
//		DB[GetPDBHash(start, distinct)] = 0;
//		entries++;
//		int depth = 1;
//		std::vector<uint64_t> depths(2);
//		do {
//			while (!q_curr.empty())
//			{
//				state next = q_curr.front();
//				q_curr.pop_front();
//				children.clear();
//				this->GetSuccessors(next, children);
//				for (unsigned int x = 0; x < children.size(); x++)
//				{
//					if (DB[GetPDBHash(children[x], distinct)] == 255)
//					{
//						int hval = depth;//-this->HCost(children[x]);
//						assert(hval >= 0);
//						DB[GetPDBHash(children[x], distinct)] = hval;
//						depths[hval]++;
//						
//						q_next.push_back(children[x]);
//						entries++;
//						if ((entries % 100000) == 0)
//						{
//							std::cout << entries << std::endl;
//						}
//						//					std::cout << children[x] << std::endl;
//						/* For Debugging
//						 if (entries < 20)
//						 {
//						 std::cout << children[x] << ": " << (unsigned) DB[GetPDBHash(children[x], distinct)] << "\n";
//						 }*/
//					}
//				}
//			}
//			depth++;
//			depths.resize(depth+1);
//			q_curr.swap(q_next);
//		} while (!q_curr.empty());
//		
//		printf("%1.2fs elapsed\n", t.EndTimer());
//		if (entries != COUNT)
//		{
//			printf("Entries: %" PRId64 "; count: %" PRId64 "\n", entries, COUNT);
//			assert(entries == COUNT);
//		}
//		
//		//TODO fix the output of PDBs
//		FILE *f = fopen(pdb_filename, "w");
//		int num = distinct.size();
//		assert(fwrite(&num, sizeof(num), 1, f) == 1);
//		assert(fwrite(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
//		assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//		//fprintf(f, "%ld\n", distinct.size()); // first element is the number of distinct pieces
//		//	for (unsigned i = 0; i < distinct.size(); i++)
//		//	{
//		//		fprintf(f, "%d\n", distinct[i]); //first few lines are these distinct elements
//		//	}
//		//	for (unsigned i = 0; i < COUNT; i++)
//		//	{
//		//		fprintf(f, "%d\n", DB[i]);
//		//	}
//		
//		fclose(f);
//		
//		printf("Wrote %lld entries to '%s'\n", entries, pdb_filename);
//		for (int x = 0; x < depths.size(); x++)
//			printf("%d %lld\n", x, depths[x]);
//		
//		PDB.push_back(DB); // increase the number of regular PDBs being stored
//		PDB_distincts.push_back(distinct); // stores distinct
//	}
//	
//	// The distinct states should not include blanks
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Build_Additive_PDB(state &start, const std::vector<int> &distinct, const char *pdb_filename, bool blank)
//	{
//		maxItem = max(maxItem,start.puzzle.size());
//		minPattern = min(minPattern, distinct.size());
//		buildCaches();
//		
//		uint64_t COUNT = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size());
//		uint64_t closedSize = nUpperk(start.puzzle.size(), start.puzzle.size() - distinct.size() - 1);
//		std::vector<int> closedPattern(distinct);
//		std::vector<uint8_t> DB(COUNT);
//		std::vector<bool> closed(closedSize);
//		
//		if (blank)
//			closedPattern.push_back(0);
//		
//		for (unsigned int x = 0; x < DB.size(); x++)
//			DB[x] = 255;
//		
//		uint64_t entries = 0;
//		std::cout << "Num Entries: " << COUNT << std::endl;
//		std::cout << "Closed list entries: " << closedSize << std::endl;
//		std::cout << "Goal State: " << start << std::endl;
//		std::cout << "State Hash of Goal: " << GetStateHash(start) << std::endl;
//		std::cout << "PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		
//		std::deque<state> q_curr, q_next;
//		std::vector<state> children;
//		
//		for (unsigned i = 0; i < start.puzzle.size(); i++)
//		{
//			bool is_distinct = false;
//			for (unsigned j = 0; j < distinct.size(); j++)
//			{
//				if (start.puzzle[i] == distinct[j])
//				{
//					is_distinct = true;
//					break;
//				}
//			}
//			
//			if (!is_distinct)
//			{
//				if (start.puzzle[i] != 0 || !blank)
//					start.puzzle[i] = -1;
//			}
//		}
//		
//		std::cout << "Abstract Goal State: " << start << std::endl;
//		std::cout << "Abstract PDB Hash of Goal: " << GetPDBHash(start, distinct) << std::endl;
//		
//		q_curr.push_back(start);
//		DB[GetPDBHash(start, distinct)] = 0;
//		entries++;
//		int depth = 1;
//		std::vector<uint64_t> depths(2);
//		depths[0]++;
//		do {
//			while (!q_curr.empty())
//			{
//				state next = q_curr.front();
//				q_curr.pop_front();
//				
//				//printf("%" PRId64 "\n", GetPDBHash(next, distinct));
//				assert(DB[GetPDBHash(next, distinct)] != 255);
//				
//				uint64_t closedHash = GetPDBHash(next, closedPattern);
//				if (closed[closedHash]) // closed
//				{
//					continue;
//				}
//				else {
//					closed[closedHash] = true;
//				}
//				
//				children.resize(0);
//				this->GetSuccessors(next, children);
//				
//				for (unsigned int x = 0; x < children.size(); x++)
//				{
//					if (children[x] == next)
//					{
//						if (closed[GetPDBHash(children[x], closedPattern)] == false)
//						{
//							q_curr.push_back(children[x]);
//						}
//					}
//					else {
//						uint64_t pdbHash = GetPDBHash(children[x], distinct);
//						if (DB[pdbHash] == 255)
//						{
//							int hval = this->HCost(children[x]);//*2-40;
//							//						if (0 == hval%2) // even
//							//						{
//							//							hval=hval*2-40;
//							//						}
//							//						else {
//							//							hval=hval*2-41;
//							//						}
//							//if (hval > 12) hval = 12;
//							hval = depth-hval;
//							assert(hval >= 0);
//							DB[pdbHash] = hval;
//							if (hval >= depths.size())
//								depths.resize(hval+1);
//							depths[hval]++;
//							entries++;
//							
//							// updates on how much built
//							if ((entries % 100000) == 0)
//							{
//								std::cout << entries << std::endl;
//							}
//						}
//						if (closed[GetPDBHash(children[x], closedPattern)] == false)
//						{
//							q_next.push_back(children[x]);
//						}
//					}
//				}
//			}
//			depth++;
//			//depths.resize(depth+1);
//			q_curr.swap(q_next);
//		} while (!q_curr.empty());
//		
//		assert(entries == COUNT);
//		
//		//TODO fix the output of PDBs
//		FILE *f = fopen(pdb_filename, "w");
//		fprintf(f, "%ld\n", distinct.size()); // first element is the number of distinct pieces
//		for (unsigned i = 0; i < distinct.size(); i++)
//		{
//			fprintf(f, "%d\n", distinct[i]); //first few lines are these distinct elements
//		}
//		assert(fwrite(&DB[0], sizeof(uint8_t), COUNT, f) == COUNT);
//		//	for (unsigned i = 0; i < COUNT; i++)
//		//	{
//		//		fprintf(f, "%d\n", DB[i]);
//		//	}
//		
//		fclose(f);
//		printf("%lld entries\n", entries);
//		for (int x = 0; x < depths.size(); x++)
//			printf("%d %lld\n", x, depths[x]);
//	}
//	
//	
//#pragma mark -
//#pragma mark Implementation - Loading PDBs
//#pragma mark -
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Load_Regular_PDB(const char *fname, state &goal, bool print_histogram)
//	{
//		//additive = false;
//		PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored
//		printf("Loading PDB '%s'\n", fname);
//		std::vector<int> distinct;
//		
//		FILE *f;
//		f = fopen(fname, "r");
//		if (f == 0)
//		{
//			printf("Failed to open pdb '%s'\n", fname);
//			exit(0);
//		}
//		
//		int num_distinct;
//		assert(fread(&num_distinct, sizeof(num_distinct), 1, f) == 1);
//		distinct.resize(num_distinct);
//		assert(fread(&distinct[0], sizeof(distinct[0]), distinct.size(), f) == distinct.size());
//		
//		maxItem = max(maxItem,goal.puzzle.size());
//		minPattern = min(minPattern, distinct.size());
//		buildCaches();
//		
//		uint64_t COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());
//		PDB.back().resize(COUNT);
//		
//		size_t index;
//		if ((index = fread(&PDB.back()[0], sizeof(uint8_t), COUNT, f)) != COUNT)
//		{
//			printf("Error; did not correctly read %" PRId64 " entries from PDB (%lu instead)\n", COUNT, index);
//			exit(0);
//		}
//		fclose(f);
//		
//		PDB_distincts.push_back(distinct); // stores distinct
//		
//		if (print_histogram)
//			PrintPDBHistogram(PDB.size()-1);
//	}
//	
//	
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::Load_Additive_PDB(const state &goal, const char *pdb_filename)
//	{
//		//additive = true;
//		
//		PDB.resize(PDB.size()+1); // increase the number of regular PDBs being stored
//		
//		std::vector<int> distinct;
//		
//		FILE *f;
//		f = fopen(pdb_filename, "r");
//		if (f == 0)
//		{
//			printf("Failed to open pdb '%s'\n", pdb_filename);
//			exit(0);
//		}
//		
//		unsigned num_distinct;
//		if (fscanf(f, "%d\n", &num_distinct) != 1)
//		{
//			printf("Failure reading tile count from PDB\n");
//			exit(0);
//		}
//		
//		for (unsigned i = 0; i < num_distinct; i++)
//		{
//			int tmp;
//			if (fscanf(f, "%d\n", &tmp) != 1)
//			{
//				printf("Failure reading tile from PDB\n");
//				exit(0);
//			}
//			
//			distinct.push_back(tmp);
//		}
//		
//		maxItem = max(maxItem,goal.puzzle.size());
//		minPattern = min(minPattern, distinct.size());
//		buildCaches();
//		
//		uint64_t COUNT = nUpperk(goal.puzzle.size(), goal.puzzle.size() - distinct.size());
//		PDB.back().resize(COUNT);
//		
//		size_t index;
//		if ((index = fread(&PDB.back()[0], sizeof(uint8_t), COUNT, f)) != COUNT)
//		{
//			printf("Error; did not correctly read %lu entries from PDB (%lu instead)\n", COUNT, index);
//			exit(0);
//		}
//		fclose(f);
//		
//		PDB_distincts.push_back(distinct); // stores distinct
//		
//		PrintPDBHistogram(PDB.size()-1);
//	}
//	
//#pragma mark -
//#pragma mark Implementation - Runtime Usage
//#pragma mark -
//	
//	/**
//	 * Deprecated: this code should no longer be used.
//	 */
//	template <class state, class action>
//	double PermutationPuzzleEnvironment<state, action>::PDB_Lookup(const state &s) const
//	{
//		if (true)//!additive)
//		{
//			double val = 0;
//			for (unsigned int x = 0; x < PDB.size(); x++)
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[x]);
//				//histogram[PDB[x][index]]++;
//				val = std::max(val, (double)PDB[x][index]);
//			}
//			return val;
//		}
//		else {
//			double val = 0;
//			uint8_t tmp;
//			for (unsigned int x = 0; x < PDB.size(); x++)
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[x]);
//				//histogram[PDB[x][index]]++;
//				tmp = PDB[x][index];
//				if (tmp > 4) tmp = 4;
//				val += (double)tmp;
//			}
//			return val;
//		}
//	}
//	
	template <class state, class action>
	double PermutationPuzzleEnvironment<state, action>::HCost(const state &s) const
	{
//		if (lookups.size() == 0)
//			return 0;
//		static std::vector<int> c1, c2;
//		return HCost(s, 0, c1, c2);
		return 0;
	}
//
//	template <class state, class action>
//	double PermutationPuzzleEnvironment<state, action>::HCost(const state &s, int treeNode,
//															  std::vector<int> &c1, std::vector<int> &c2) const
//	{
//		double hval = 0;
//		switch (lookups[treeNode].t)
//		{
//			case kMaxNode:
//			{
//				for (int x = 0; x < lookups[treeNode].numChildren; x++)
//				{
//					hval = max(hval, HCost(s, lookups[treeNode].firstChildID+x, c1, c2));
//				}
//			} break;
//			case kAddNode:
//			{
//				for (int x = 0; x < lookups[treeNode].numChildren; x++)
//				{
//					hval += HCost(s, lookups[treeNode].firstChildID+x, c1, c2);
//				}
//			} break;
//			case kLeafNode:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				hval = PDB[lookups[treeNode].PDBID][index];
//			} break;
//			case kLeafFractionalCompress:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				if (index < PDB[lookups[treeNode].PDBID].size())
//					hval = PDB[lookups[treeNode].PDBID][index];
//				else
//					hval = 0;
//			} break;
//			case kLeafFractionalModCompress: // num children is the compression factor
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				if (0 == index%lookups[treeNode].numChildren)
//					hval = PDB[lookups[treeNode].PDBID][index/lookups[treeNode].numChildren];
//				else
//					hval = 0;
//			} break;
//			case kLeafModCompress:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				hval = PDB[lookups[treeNode].PDBID][index%PDB[lookups[treeNode].PDBID].size()];
//			} break;
//			case kLeafMinCompress:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2)/lookups[treeNode].numChildren;
//				hval = PDB[lookups[treeNode].PDBID][index];
//			} break;
//			case kLeafValueCompress:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				hval = PDB[lookups[treeNode].PDBID][index];
//				if (hval > lookups[treeNode].numChildren)
//					hval = lookups[treeNode].numChildren;
//			} break;
//			case kLeafDivPlusDeltaCompress:
//			{
//				uint64_t index = GetPDBHash(s, PDB_distincts[lookups[treeNode].PDBID], c1, c2);
//				hval = PDB[lookups[treeNode].PDBID][index/lookups[treeNode].numChildren];
//				hval += PDB[lookups[treeNode].firstChildID][index];
//			}
//			case kLeafDefaultHeuristic:
//			{
//				hval = DefaultH(s);
//			} break;
//		}
//		return hval;
//	}
//	
//#pragma mark -
//#pragma mark Implementation - Utilities
//#pragma mark -
//	
	template <class state, class action>
	uint64_t PermutationPuzzleEnvironment<state, action>::Factorial(int val) const
	{
		static uint64_t table[21] =
		{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
			6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
			6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
		if (val > 20)
			return (uint64_t)-1;
		return table[val];
	}

	/**
	 Builds caches for nUpperk
	 **/
	template <class state, class action>
	void PermutationPuzzleEnvironment<state, action>::buildCaches() const
	{
		factorialCache.resize(maxItem+1);
		for (int n = 0; n < factorialCache.size(); n++)
		{
			factorialCache[n].resize(maxItem-minPattern+1);
			for (int k = 0; k < factorialCache[n].size(); k++)
			{
				uint64_t value = 1;
				for (int i = n; i > k; i--)
				{
					value *= i;
				}
				factorialCache[n][k] = value;
				
			}
		}
	}

	/**
	 Returns the value of n! / k!
	 **/
	template <class state, class action>
	uint64_t PermutationPuzzleEnvironment<state, action>::nUpperk(int n, int k) const
	{
		assert(factorialCache.size() > n);
		assert(factorialCache[n].size() > k);
		return factorialCache[n][k];
		
		//	static std::vector<std::vector<uint64_t> > cache;
		//	if (n >= cache.size())
		//		cache.resize(n+1);
		//	if (k >= cache[n].size())
		//		cache[n].resize(k+1);
		//	if (cache[n][k] == 0)
		//	{
		//		uint64_t value = 1;
		//		assert(n >= 0 && k >= 0);
		//
		//		for (int i = n; i > k; i--)
		//		{
		//			value *= i;
		//		}
		//
		//		cache[n][k] = value;
		//		return value;
		//	}
		//	return cache[n][k];
	}
//
//	/**
//	 Returns the Hash Value of the given state using the given set of distinct items
//	 **/
//	template <class state, class action>
//	uint64_t PermutationPuzzleEnvironment<state, action>::GetPDBHash(const state &s,
//																	 const std::vector<int> &distinct) const
//	{
//		static std::vector<int> locs;
//		static std::vector<int> dual;
//		return GetPDBHash(s, distinct, locs, dual);
//	}
//	
//	/**
//	 Compute the size of a PDB using a given state & set of tiles
//	 **/
//	template <class state, class action>
//	uint64_t PermutationPuzzleEnvironment<state, action>::Get_PDB_Size(state &start, int pdbEntries)
//	{
//		maxItem = max(maxItem,start.puzzle.size());
//		minPattern = min(minPattern, pdbEntries);
//		buildCaches();
//		return nUpperk(start.puzzle.size(), start.puzzle.size()-pdbEntries);
//	}
//	
//	// TODO Change to Myrvold and Ruskey ranking function
//	/**
//	 Returns the Hash Value of the given state using the given set of distinct items
//	 This version is thread safe -- caches are passed in
//	 **/
//	template <class state, class action>
//	uint64_t PermutationPuzzleEnvironment<state, action>::GetPDBHash(const state &s,
//																	 const std::vector<int> &distinct,
//																	 std::vector<int> &locs,
//																	 std::vector<int> &dual) const
//	{
//		locs.resize(distinct.size()); // vector for distinct item locations
//		dual.resize(s.puzzle.size()); // vector for distinct item locations
//		
//		// find item locations
//		for (unsigned int x = 0; x < s.puzzle.size(); x++)
//		{
//			if (s.puzzle[x] != -1)
//				dual[s.puzzle[x]] = x;
//		}
//		for (int x = 0; x < distinct.size(); x++)
//		{
//			locs[x] = dual[distinct[x]];
//		}
//		
//		uint64_t hashVal = 0;
//		int numEntriesLeft = s.puzzle.size();
//		
//		for (unsigned int x = 0; x < locs.size(); x++)
//		{
//			hashVal += locs[x]*nUpperk(numEntriesLeft-1, s.puzzle.size()-distinct.size());
//			numEntriesLeft--;
//			
//			// decrement locations of remaining items
//			for (unsigned y = x; y < locs.size(); y++)
//			{
//				if (locs[y] > locs[x])
//					locs[y]--;
//			}
//		}
//		return hashVal;
//	}
//	
//	/**
//	 Returns the state from the hash value given the pattern and number of items in the puzzle
//	 This function is not thread safe.
//	 **/
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::GetStateFromPDBHash(uint64_t hash, state &s, int count,
//																		  const std::vector<int> &pattern)
//	{
//		static std::vector<int> dual;
//		GetStateFromPDBHash(hash, s, count, pattern, dual);
//	}
//	
//	/**
//	 Returns the state from the hash value given the pattern and number of items in the puzzle
//	 This function needs a vector which it shouldn't re-create at every time step. As such, we
//	 require the user to pass one in. (This enables concurrency; although better solutions exist.)
//	 **/
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::GetStateFromPDBHash(uint64_t hash, state &s, int count,
//																		  const std::vector<int> &pattern,
//																		  std::vector<int> &dual)
//	{
//		uint64_t hashVal = hash;
//		dual.resize(pattern.size());
//		
//		int numEntriesLeft = count-pattern.size()+1;
//		for (int x = pattern.size()-1; x >= 0; x--)
//		{
//			dual[x] = hashVal%numEntriesLeft;
//			hashVal /= numEntriesLeft;
//			numEntriesLeft++;
//			for (int y = x+1; y < pattern.size(); y++)
//			{
//				if (dual[y] >= dual[x])
//					dual[y]++;
//			}
//		}
//		s.puzzle.resize(count);
//		std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
//		for (int x = 0; x < dual.size(); x++)
//			s.puzzle[dual[x]] = pattern[x];
//	}
	
	template <class state, class action>
	void PermutationPuzzleEnvironment<state, action>::GetStateFromHash(state &s, uint64_t hash) const
	{
		state tmp;
		mr1.Unrank(hash, s.puzzle, tmp.puzzle, s.size(), s.size());
		s.FinishUnranking();
	}
	
	template <class state, class action>
	uint64_t PermutationPuzzleEnvironment<state, action>::GetStateHash(const state &s) const
	{
		state tmp = s;
		state dual;
		for (size_t x = 0; x < tmp.size(); x++)
			dual.puzzle[tmp.puzzle[x]] = x;

		uint64_t hash = mr1.Rank(&tmp.puzzle[0], &dual.puzzle[0], s.size(), s.size());
		return hash;
	}
	
	template <class state, class action>
	state PermutationPuzzleEnvironment<state, action>::TranformToStandardGoal(const state &a, const state &b) const
	{
		state dual;
		state result;
		for (size_t x = 0; x < b.size(); x++)
		{
			dual.puzzle[b.puzzle[x]] = x;
		}
		for (size_t x = 0; x < b.size(); x++)
		{
			result.puzzle[x] = dual.puzzle[a.puzzle[x]];
		}
		result.FinishUnranking();
		return result;
	}

//	/**
//	 * Show the distribution and average value of a PDB.
//	 */
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::PrintPDBHistogram(int which) const
//	{
//		double average = 0;
//		uint8_t maxval = 0;
//		std::vector<uint64_t> values(256);
//		// performs histogram count
//		for (uint64_t x = 0; x < PDB[which].size(); x++)
//		{
//			values[(PDB[which][x])]++;
//			maxval = max(maxval, PDB[which][x]);
//		}
//		// outputs histogram of heuristic value counts
//		for (uint64_t x = 0; x <= maxval; x++)
//		{
//			printf("%lld:\t%" PRId64 "\n", x, values[x]);
//			average += x*values[x];
//		}
//		printf("Average value: %1.4f\n", average/PDB[which].size());
//	}
//	
//	/**
//	 * Return the distribution of values in a PDB.
//	 */
//	template <class state, class action>
//	void PermutationPuzzleEnvironment<state, action>::GetPDBHistogram(int which, std::vector<uint64_t> &values) const
//	{
//		uint8_t maxval = 0;
//		values.resize(0);
//		values.resize(256);
//		// performs histogram count
//		for (uint64_t x = 0; x < PDB[which].size(); x++)
//		{
//			values[(PDB[which][x])]++;
//			maxval = max(maxval, PDB[which][x]);
//		}
//		values.resize(maxval+1);
//	}
//	
//	
	/**
	 Ensures that the state contains a valid permutation. That is, if the state is of size
	 n, each of the integers from 0 to n-1 occurs exactly once.
	 **/
	template <class state, class action>
	bool PermutationPuzzleEnvironment<state, action>::Check_Permutation(const std::vector<int> &to_check)
	{
		const auto size = to_check.size();
		
		bool in_puzzle[size];
		
		for (size_t i = 0; i < size; i++)
		{
			in_puzzle[i] = false;
		}
		
		for (size_t i = 0; i < size; i++)
		{
			in_puzzle[to_check[i]] = true;
		}
		
		for (size_t i = 0; i < size; i++)
		{
			if (!in_puzzle[i])
				return false;
		}
		
		return true;
	}
	
	/**
	 Outputs the set of puzzles in puzzle_vector to standard output. The output is of the
	 form "I_0 I_1 ... I_(MN - 1)" where I_K is the element in the Kth position of the
	 puzzle. If the write_puzz_num flag is set as true, the first item of the output is
	 the puzzle number. Puzzles are checked for validity before they are outputted.
	 True is return if all the puzzles are valid, otherwise false is returned and
	 the puzzles are not outputted.
	 **/
	template <class state, class action>
	bool PermutationPuzzleEnvironment<state, action>::Output_Puzzles(std::vector<state> &puzzle_vector, bool write_puzz_num)
	{
		// check validity of puzzles
		for (unsigned i = 0; i < puzzle_vector.size(); i++)
		{
			/**
			 Make sure all puzzles are for this environment
			 **/
			if (!Check_Permutation(puzzle_vector[i].puzzle))
			{
				std::cerr << "Invalid Puzzle: " << puzzle_vector[i] << '\n';
				return false;
			}
		}
		
		for (unsigned i = 0; i < puzzle_vector.size(); i++)
		{
			if (write_puzz_num)
			{
				printf("%u ", i + 1);
			}
			printf("%d", puzzle_vector[i].puzzle[0]);
			
			for (unsigned j = 1; j < puzzle_vector[i].puzzle.size(); j++)
			{
				printf(" %d", puzzle_vector[i].puzzle[j]);
			}
			printf("\n");
		}
		return true;
	}
	
	// TODO clean up using split
	template <class state, class action>
	bool PermutationPuzzleEnvironment<state, action>::Read_In_Permutations(const char *filename, unsigned size, unsigned max_puzzles,
																		   std::vector<std::vector<int> > &permutations, bool puzz_num_start)
	{
		std::ifstream ifs(filename, std::ios::in);
		
		if (ifs.fail())
		{
			fprintf(stderr, "File Reading Failed\n");
			return false;
		}
		
		std::string s, temp;
		
		std::vector<int> puzz_ints;
		
		bool first = true;
		unsigned puzz_count = 0;
		
		while(!ifs.eof() && (puzz_count < max_puzzles || max_puzzles==0) )
		{
			puzz_ints.clear();
			
			getline(ifs, s);
			// indicates are starting new permutation
			first = true;
			for (unsigned int i = 0; i < s.length(); i++)
			{
				if (s.at(i) == ' ' || s.at(i) == '\t')
				{
					if (temp.length() > 0)
					{
						if (puzz_num_start && first)
						{
							temp.clear();
							first = false;
						}
						else
						{
							puzz_ints.push_back(atoi(temp.c_str()));
							temp.clear();
						}
					}
				}
				else
				{
					temp.push_back(s.at(i));
				}
			}
			
			
			if (temp.length() > 0)
			{
				puzz_ints.push_back(atoi(temp.c_str()));
				temp = "";
			}
			
			// ensure is proper permutation, otherwise don't add it to the list
			if (puzz_ints.size() == size && Check_Permutation(puzz_ints))
			{
				puzz_count++;
				permutations.push_back(puzz_ints);
			}
			
		}
		
		ifs.close();
		
		return true;
	}

	template <class state, class action>
	std::vector<int> PermutationPuzzleEnvironment<state, action>::Get_Random_Permutation(unsigned size)
	{
		std::vector<int> permutation;
		
		for (unsigned i = 0; i < size; i++)
		{
			permutation.push_back(i);
		}
		int index = 0;
		int temp;
		
		// randomizes elements in the permutation
		while(size > 1)
		{		index = rand() % size;
			temp = permutation[size - 1];
			permutation[size - 1] = permutation[index];
			permutation[index] = temp;
			
			size--;
		}
		
		return permutation;
	}
	
	template <class state, class action>
	bool PermutationPuzzleEnvironment<state, action>::Validate_Problems(std::vector<state> &puzzles)
	{
		for (unsigned i = 0; i < puzzles.size(); i++)
		{
			if (!State_Check(puzzles[i]) || !Check_Permutation(puzzles[i].puzzle))
			{
				std::cerr << puzzles[i] << '\n';
				std::cerr << "Invalid Puzzle\n";
			}
		}
		return true;
	}
	
}

#endif // PERMPUZZ_H
