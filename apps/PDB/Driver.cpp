/*
 *  Driver.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 10/1/19.
 *
 *  This file is part of HOG.
 */

#include <cstring>
#include "Common.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "Driver.h"
#include "MNPuzzle.h"
#include "RubiksCube.h"
#include "Timer.h"
#include "TopSpin.h"
#include "STPInstances.h"
#include "TOH.h"

template <int width, int height>
void BuildSTPPDB();
void BuildRC();
template <int tiles>
void BuildTS();
void BuildTOH();

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

enum Domain {
	kRC,
	kSTP44,
	kSTP55,
	kTS16,
	kTOH
};
Domain domain;
bool additive = false;
bool load = false;
bool delta = false;
std::vector<int> pattern;
std::string path;
int threads = std::thread::hardware_concurrency();
int compression = 1;

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallWindowHandler(MyWindowHandler);
	InstallCommandLineHandler(MyCLHandler, "-domain", "-domain", "Select domain to build for. {STP44 STP55 RC, TS16, TOH}");
	InstallCommandLineHandler(MyCLHandler, "-pattern", "-pattern", "Choose tiles in PDB");
	InstallCommandLineHandler(MyCLHandler, "-additive", "-additive", "Build additive PDB");
	InstallCommandLineHandler(MyCLHandler, "-load", "-load", "Load from disk and print stats; optionally compress");
	InstallCommandLineHandler(MyCLHandler, "-threads", "-threads <N>", "Use <N> threads");
	InstallCommandLineHandler(MyCLHandler, "-path", "-path <path>", "Set path for output PDB");
	InstallCommandLineHandler(MyCLHandler, "-delta", "-delta", "Use delta PDB");
	InstallCommandLineHandler(MyCLHandler, "-compress", "-compress <factor>", "DIV compress PDB by a factor of <factor>");
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
	}
	else if (eType == kWindowCreated)
	{
		printf("Building PDB\n");
		switch (domain)
		{
			case kRC:
				BuildRC();
				break;
			case kSTP44:
				BuildSTPPDB<4, 4>();
				break;
			case kSTP55:
				BuildSTPPDB<5, 5>();
				break;
			case kTS16:
				BuildTS<16>();
				break;
			case kTOH:
				BuildTOH();
				break;
		}
		printf("Done.\n");
		exit(0);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-domain") == 0)
	{
		if (maxNumArgs <= 1)
		{
			printf("Need domain: STP44 STP55 RC TS16 TOH\n");
			exit(0);
		}
		if (strcmp(argument[1], "RC") == 0)
			domain = kRC;
		else if (strcmp(argument[1], "STP44") == 0)
			domain = kSTP44;
		else if (strcmp(argument[1], "STP55") == 0)
			domain = kSTP55;
		else if (strcmp(argument[1], "TS16") == 0)
			domain = kTS16;
		else if (strcmp(argument[1], "TOH") == 0)
			domain = kTOH;
		return 2;
	}
	if (strcmp(argument[0], "-additive") == 0)
	{
		additive = true;
		return 1;
	}
	if (strcmp(argument[0], "-delta") == 0)
	{
		delta = true;
		return 1;
	}
	if (strcmp(argument[0], "-load") == 0)
	{
		load = true;
		return 1;
	}
	if (strcmp(argument[0], "-threads") == 0)
	{
		if (maxNumArgs > 1)
			threads = atoi(argument[1]);
		else
			printf("Error processing thread count; defaulting to %d\n", threads);
		return 2;
	}
	if (strcmp(argument[0], "-compress") == 0)
	{
		if (maxNumArgs > 1)
			compression = atoi(argument[1]);
		else
			printf("Error processing compression factor; defaulting to %d\n", compression);
		return 2;
	}
	if (strcmp(argument[0], "-path") == 0)
	{
		path = argument[1];
		return 2;
	}
	
	if (strcmp(argument[0], "-pattern") == 0)
	{
		int cnt = 1;
		maxNumArgs--;
		while (maxNumArgs > 0 && argument[cnt][0] != '-')
		{
			maxNumArgs--;
			pattern.push_back(atoi(argument[cnt]));
			cnt++;
		}
		return cnt;
	}

	return 1;
}

template <int width, int height>
void BuildSTPPDB()
{
	MNPuzzle<width, height> mnp;
	MNPuzzleState<width, height> goal;
	std::vector<slideDir> moves;
	goal.Reset();
	mnp.StoreGoal(goal);

	LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(&mnp, goal, pattern);
	if (load)
	{
		if (pdb.Load(path.c_str()))
		{
			printf("Loaded successfully\n");
			pdb.PrintHistogram();

			if (compression != 1)
			{
				pdb.DivCompress(compression, true);
			}
			return;
		}
	}
	
	if (additive)
	{
		mnp.SetPattern(pattern);
		pdb.BuildAdditivePDB(goal, threads); // parallelism not fixed yet
		if (delta)
			pdb.DeltaCompress(&mnp, goal, true);
		if (compression != 1)
		{
			pdb.DivCompress(compression, true);
		}
		pdb.Save(path.c_str());
	}
	else {
		pdb.BuildPDB(goal, threads);
		if (delta)
			pdb.DeltaCompress(&mnp, goal, true);
		if (compression != 1)
		{
			pdb.DivCompress(compression, true);
		}
		pdb.Save(path.c_str());
	}
}

void BuildRC()
{
	RubiksCube cube;
	RubiksState goal;
	goal.Reset();
	
	std::vector<int> edges;
	std::vector<int> corners;
	for (auto i : pattern)
	{
		if (i < 12)
		{
			edges.push_back(i);
		}
		else {
			corners.push_back(i-12);
		}
	}
	printf("\nEdges: ");
	for (auto i : edges) printf("%d ", i);
	printf("\nCorners: ");
	for (auto i : corners) printf("%d ", i);
	printf("\n");
	
	RubikPDB pdb(&cube, goal, edges, corners);
	if (load)
	{
		if (pdb.Load(path.c_str()))
		{
			printf("Loaded successfully\n");
			pdb.PrintHistogram();
			return;
		}
	}
	
	pdb.BuildPDB(goal, threads);

	if (delta)
	{
		if (edges.size() > 4)
			edges.resize(4);
		if (corners.size() > 4)
			corners.resize(4);
		printf("Delta compression - building delta\n");
		RubikPDB pdb2(&cube, goal, edges, corners);
		pdb2.BuildPDB(goal, threads);
		pdb.DeltaCompress(&pdb2, goal, true);
	}
	if (compression != 1)
	{
		pdb.DivCompress(compression, true);
	}
	pdb.Save(path.c_str());
}


template <int tiles>
void BuildTS()
{
	TopSpin<tiles, 4> ts;
	TopSpinState<tiles> goal;
	std::vector<TopSpinAction> moves;
	goal.Reset();
	std::vector<TopSpinState<tiles>> goals;
	ts.GetGoals(goals);

	LexPermutationPDB<TopSpinState<tiles>, TopSpinAction, TopSpin<tiles, 4>> pdb(&ts, goals, pattern);

	if (load)
	{
		if (pdb.Load(path.c_str()))
		{
			printf("Loaded successfully\n");
			pdb.PrintHistogram();
			return;
		}
	}
	
	if (additive)
	{
		printf("WARNING: Additive TS PDBs appear to work but are still being tested.\n");
		ts.SetPattern(pattern);
		pdb.BuildAdditivePDB(goal, threads, false); // course open doesn't work on non-unit cost domains

		exit(0);
	}
	else {
		pdb.BuildPDB(goal, threads);
		//pdb.BuildPDBForward(goals, threads, true, true);
		if (compression != 1)
		{
			pdb.DivCompress(compression, true);
		}
		pdb.Save(path.c_str());
	}
}


template<int pdbDisks, int numDisks>
void BuildTOHHelper()
{
	TOH<numDisks> toh;
	TOH<pdbDisks> absToh1;
	TOHState<pdbDisks> absTohState1;

	TOHState<numDisks> goal;
	goal.Reset();
	
	TOHPDB<pdbDisks, numDisks> *pdb1 = new TOHPDB<pdbDisks, numDisks>(&absToh1, goal); // top disks
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	if (compression != 1)
	{
		pdb1->DivCompress(compression, true);
		pdb1->Save(path.c_str());
	}
	else {
		pdb1->Save(path.c_str());
	}
	delete pdb1;
}

void BuildTOH()
{
	assert(pattern.size() > 1);
	int pdbDisks = pattern[0];
	int numDisks = pattern[1];
	assert(numDisks >= 10 && numDisks <= 16);
	assert(pdbDisks < numDisks && pdbDisks > 0);
	
	switch (numDisks)
	{
		case 10:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 10>(); break;
				case 2: BuildTOHHelper<2, 10>(); break;
				case 3: BuildTOHHelper<3, 10>(); break;
				case 4: BuildTOHHelper<4, 10>(); break;
				case 5: BuildTOHHelper<5, 10>(); break;
				case 6: BuildTOHHelper<6, 10>(); break;
				case 7: BuildTOHHelper<7, 10>(); break;
				case 8: BuildTOHHelper<8, 10>(); break;
				case 9: BuildTOHHelper<9, 10>(); break;
				default: break;
			} break;
		case 11:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 11>(); break;
				case 2: BuildTOHHelper<2, 11>(); break;
				case 3: BuildTOHHelper<3, 11>(); break;
				case 4: BuildTOHHelper<4, 11>(); break;
				case 5: BuildTOHHelper<5, 11>(); break;
				case 6: BuildTOHHelper<6, 11>(); break;
				case 7: BuildTOHHelper<7, 11>(); break;
				case 8: BuildTOHHelper<8, 11>(); break;
				case 9: BuildTOHHelper<9, 11>(); break;
				case 10: BuildTOHHelper<10, 11>(); break;
				default: break;
			} break;
		case 12:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 12>(); break;
				case 2: BuildTOHHelper<2, 12>(); break;
				case 3: BuildTOHHelper<3, 12>(); break;
				case 4: BuildTOHHelper<4, 12>(); break;
				case 5: BuildTOHHelper<5, 12>(); break;
				case 6: BuildTOHHelper<6, 12>(); break;
				case 7: BuildTOHHelper<7, 12>(); break;
				case 8: BuildTOHHelper<8, 12>(); break;
				case 9: BuildTOHHelper<9, 12>(); break;
				case 10: BuildTOHHelper<10, 12>(); break;
				case 11: BuildTOHHelper<11, 12>(); break;
				default: break;
			} break;
		case 13:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 13>(); break;
				case 2: BuildTOHHelper<2, 13>(); break;
				case 3: BuildTOHHelper<3, 13>(); break;
				case 4: BuildTOHHelper<4, 13>(); break;
				case 5: BuildTOHHelper<5, 13>(); break;
				case 6: BuildTOHHelper<6, 13>(); break;
				case 7: BuildTOHHelper<7, 13>(); break;
				case 8: BuildTOHHelper<8, 13>(); break;
				case 9: BuildTOHHelper<9, 13>(); break;
				case 10: BuildTOHHelper<10, 13>(); break;
				case 11: BuildTOHHelper<11, 13>(); break;
				case 12: BuildTOHHelper<12, 13>(); break;
				default: break;
			} break;
		case 14:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 14>(); break;
				case 2: BuildTOHHelper<2, 14>(); break;
				case 3: BuildTOHHelper<3, 14>(); break;
				case 4: BuildTOHHelper<4, 14>(); break;
				case 5: BuildTOHHelper<5, 14>(); break;
				case 6: BuildTOHHelper<6, 14>(); break;
				case 7: BuildTOHHelper<7, 14>(); break;
				case 8: BuildTOHHelper<8, 14>(); break;
				case 9: BuildTOHHelper<9, 14>(); break;
				case 10: BuildTOHHelper<10, 14>(); break;
				case 11: BuildTOHHelper<11, 14>(); break;
				case 12: BuildTOHHelper<12, 14>(); break;
				case 13: BuildTOHHelper<13, 14>(); break;
				default: break;
			} break;
		case 15:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 15>(); break;
				case 2: BuildTOHHelper<2, 15>(); break;
				case 3: BuildTOHHelper<3, 15>(); break;
				case 4: BuildTOHHelper<4, 15>(); break;
				case 5: BuildTOHHelper<5, 15>(); break;
				case 6: BuildTOHHelper<6, 15>(); break;
				case 7: BuildTOHHelper<7, 15>(); break;
				case 8: BuildTOHHelper<8, 15>(); break;
				case 9: BuildTOHHelper<9, 15>(); break;
				case 10: BuildTOHHelper<10, 15>(); break;
				case 11: BuildTOHHelper<11, 15>(); break;
				case 12: BuildTOHHelper<12, 15>(); break;
				case 13: BuildTOHHelper<13, 15>(); break;
				case 14: BuildTOHHelper<14, 15>(); break;
				default: break;
			} break;
		case 16:
			switch (pdbDisks)
			{
				case 1: BuildTOHHelper<1, 16>(); break;
				case 2: BuildTOHHelper<2, 16>(); break;
				case 3: BuildTOHHelper<3, 16>(); break;
				case 4: BuildTOHHelper<4, 16>(); break;
				case 5: BuildTOHHelper<5, 16>(); break;
				case 6: BuildTOHHelper<6, 16>(); break;
				case 7: BuildTOHHelper<7, 16>(); break;
				case 8: BuildTOHHelper<8, 16>(); break;
				case 9: BuildTOHHelper<9, 16>(); break;
				case 10: BuildTOHHelper<10, 16>(); break;
				case 11: BuildTOHHelper<11, 16>(); break;
				case 12: BuildTOHHelper<12, 16>(); break;
				case 13: BuildTOHHelper<13, 16>(); break;
				case 14: BuildTOHHelper<14, 16>(); break;
				case 15: BuildTOHHelper<15, 16>(); break;
				default: break;
			} break;
	}
}

