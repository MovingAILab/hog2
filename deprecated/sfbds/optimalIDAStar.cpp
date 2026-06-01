#include "optimalIDAStar.h"
#include "statehash.h"

template<>
uint64_t OptimalIDAStar<PancakePuzzleState,unsigned>::GetHashFromState( PancakePuzzleState &s ) {
	std::vector<int> puzzle = s.puzzle;
	uint64_t hash = 0;
	int numEntriesLeft = s.puzzle.size();
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		hash += puzzle[x]*factorial(numEntriesLeft-1);
		numEntriesLeft--;
		for (unsigned y = x; y < puzzle.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hash;
};


template<>
void OptimalIDAStar<PancakePuzzleState,unsigned>::GetStateFromHash( uint64_t hash, PancakePuzzleState &s ) {
	std::vector<int> puzzle = s.puzzle;

	int numEntriesLeft = 1;
	for (int x = s.puzzle.size()-1; x >= 0; x--)
	{
		puzzle[x] = hash%numEntriesLeft;
		hash     /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < (int) s.puzzle.size(); y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}

	s.puzzle = puzzle;
};
