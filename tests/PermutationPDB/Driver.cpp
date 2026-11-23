#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>
#include "PancakePuzzle.h"
#include "LexPermutationPDB.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

template <int N>
using PDB = LexPermutationPDB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>>;

template <int N>
void TestExhaustive(const std::vector<int> &pattern)
{
	PancakePuzzle<N> env;
	PancakePuzzleState<N> s;
	PDB<N> pdb(&env, s, pattern);
	for (uint64_t r = 0; r < pdb.GetPDBSize(); r++)
	{
		pdb.GetStateFromPDBHash(r, s);
		CHECK(pdb.GetPDBHash(s) == r);
	}
}

template <int N>
void TestRandom(const std::vector<int> &pattern, int samples)
{
	PancakePuzzle<N> env;
	PancakePuzzleState<N> goal, s, back;
	PDB<N> pdb(&env, goal, pattern);
	std::mt19937_64 rng(1);
	for (int i = 0; i < samples; i++)
	{
		std::shuffle(s.puzzle, s.puzzle+N, rng);
		uint64_t h = pdb.GetPDBHash(s);
		CHECK(h < pdb.GetPDBSize());
		pdb.GetStateFromPDBHash(h, back);
		for (int p : pattern)
		{
			int loc = std::find(s.puzzle, s.puzzle+N, p) - s.puzzle;
			CHECK(back.puzzle[loc] == p);
		}
	}
}

template <int N>
bool Rejected(const std::vector<int> &pattern)
{
	PancakePuzzle<N> env;
	PancakePuzzleState<N> s;
	try { PDB<N> pdb(&env, s, pattern); }
	catch (const std::length_error &) { return true; }
	return false;
}

static std::vector<int> Evens(int n, int k)
{
	std::vector<int> v;
	for (int i = 0; i < n && (int)v.size() < k; i += 2) v.push_back(i);
	return v;
}

static std::vector<int> First(int k)
{
	std::vector<int> v(k);
	std::iota(v.begin(), v.end(), 0);
	return v;
}

int main()
{
	TestExhaustive<6>(First(4));
	TestExhaustive<7>({1, 3, 5});
	TestRandom<30>(First(14), 20000);
	TestRandom<30>(Evens(30, 12), 20000);
	TestRandom<20>(First(20), 20000);
	TestRandom<142>(First(9), 2000);
	CHECK(!Rejected<30>(First(14)));
	CHECK(!Rejected<142>(First(9)));
	CHECK(Rejected<142>(First(10)));
	CHECK(Rejected<30>(First(15)));
	CHECK(Rejected<30>(Evens(30, 15)));
	CHECK(Rejected<21>(First(21)));
	printf(failures ? "FAILED (%d)\n" : "PASSED\n", failures);
	return failures != 0;
}
