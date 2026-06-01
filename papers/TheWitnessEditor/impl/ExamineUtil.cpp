#include <mutex>
#include <thread>

#include "Combinations.h"
#include "ExamineUtil.h"
#include "SolutionUtil.h"

std::mutex lock;
std::vector<Witness<puzzleWidth, puzzleHeight>> best;
// std::vector<uint64_t> otherbest;

void Load(uint64_t which)
{
    lock.lock();
    if (best.size() > which)
    {
        witness = best[which];
        iws.Reset();
    }
    lock.unlock();
    //	iws.Reset();
    //
    //		w.ClearMustCrossConstraints();
    //		int *items = new int[req];
    //		int *items2 = new int[sep];
    //		Combinations<w.GetNumMustCrossConstraints()> c;
    //		Combinations<w.GetNumSeparationConstraints()> regionCombs;
    //
    //	if (best.size() > 0)
    //	{
    //		c.Unrank(best[which], items, req);
    //		for (int x = 0; x < req; x++)
    //		{
    //			w.SetMustCrossConstraint(items[x]);
    //		}
    //	}
    //
    //	if (otherbest.size() > 0)
    //	{
    //		w.ClearSeparationConstraints();
    //		uint64_t bits = otherbest[which]; // will only use bottom bits
    //		uint64_t hash = otherbest[which]/(1<<sep);
    //		regionCombs.Unrank(hash, items2, sep);
    //		for (int x = 0; x < sep; x++)
    //		{
    //			w.AddSeparationConstraint(items2[x], ((bits>>x)&1)?Colors::white:Colors::black);
    //		}
    //	}
    //
    //	delete [] items;
    //	delete [] items2;
}

void ExamineMustCross(int count)
{
    Timer t;
    t.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
    GetAllSolutions(allSolutions);

    uint64_t minCount = allSolutions.size();
    int *items = new int[count];
    Combinations<witness.GetNumPathConstraints()> c;

    Witness<puzzleWidth, puzzleHeight> w;
    WitnessState<puzzleWidth, puzzleHeight> s;

    uint64_t maxRank = c.MaxRank(count);
    for (uint64_t n = 0; n < maxRank; n++)
    {
        if (0 == n % 50000) printf("%llu of %llu\n", n, maxRank);
        c.Unrank(n, items, count);
        for (int x = 0; x < count; x++)
        {
            w.SetMustCrossConstraint(items[x]);
        }

        int pathLen = 0;
        int result = CountSolutions(w, allSolutions, pathLen, (int)minCount + 1);
        if (result > minSolutions)
        {
            // ignore
        }
        else if (result < minCount && result > 0)
        {
            minCount = result;
            best.clear();
            best.push_back(w);
        }
        else if (result == minCount)
        {
            best.push_back(w);
        }

        for (int x = 0; x < count; x++)
        {
            w.ClearMustCrossConstraint(items[x]);
        }
    }

    printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
    if (!best.empty())
    {
        currBoard = 0;
        Load(currBoard);
    }

    delete[] items;
}

void ExamineMustCrossAndRegions(int crossCount, int regionCount)
{
    Timer timer;
    timer.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
    GetAllSolutions(allSolutions);

    uint64_t minCount = allSolutions.size();
    int bestPathSize = 0;
    //	std::vector<uint64_t> best;
    int *crossItems = new int[crossCount];
    int *regionItems = new int[regionCount];
    Combinations<witness.GetNumPathConstraints()> c;
    Combinations<witness.GetNumSeparationConstraints()> regionCombs;

    {
        Witness<puzzleWidth, puzzleHeight> w;
        WitnessState<puzzleWidth, puzzleHeight> s;

        uint64_t maxRank = c.MaxRank(crossCount);
        for (uint64_t n = 0; n < maxRank; n++)
        {
            if (0 == n % 50000 && regionCount == 0) printf("%llu of %llu\n", n, maxRank);
            c.Unrank(n, crossItems, crossCount);
            for (int x = 0; x < crossCount; x++)
            {
                w.SetMustCrossConstraint(crossItems[x]);
            }
            uint64_t colorComb = pow(2, regionCount);
            for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount) * colorComb; t++)
            {
                uint64_t globalPuzzle = n * regionCombs.MaxRank(regionCount) * colorComb + t;
                if ((0 == globalPuzzle % 50000))
                    printf("-->%llu of %llu\n", globalPuzzle, maxRank * regionCombs.MaxRank(regionCount) * colorComb);
                uint64_t bits = t; // will only use bottom bits
                uint64_t hash = t / colorComb;

                // easy way to reduce symmetry
                if (t & 1) continue;

                //				if (0 != bits%3)
                //					continue;
                //				if (1 != (bits/3)%3)
                //					continue;
                //				if (2 != (bits/9)%3)
                //					continue;

                w.ClearSeparationConstraints();
                regionCombs.Unrank(hash, regionItems, regionCount);
                for (int x = 0; x < regionCount; x++)
                {
                    int colour = bits % 2;
                    bits = bits / 2;
                    //					w.AddSeparationConstraint(regionItems[x],
                    //((bits>>x)&1)?Colors::white:Colors::black);
                    w.AddSeparationConstraint(
                        regionItems[x], (colour == 0) ? Colors::white : ((colour == 1) ? Colors::black : Colors::blue));
                }

                int pathSize = 0;
                int result = CountSolutions(w, allSolutions, pathSize, (int)minCount + 1);

                if (result < minCount && result > 0)
                {
                    minCount = result;
                    bestPathSize = pathSize;
                    best.clear();
                    best.push_back(w);
                }
                else if (result == minCount && pathSize == bestPathSize)
                //				else if (result == minCount)
                {
                    best.push_back(w);
                }
            }

            w.ClearPathConstraints();
        }
    }

    printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize,
        timer.EndTimer());
    if (!best.empty())
    {
        currBoard = 0;
        Load(currBoard);
    }

    delete[] crossItems;
    delete[] regionItems;
}

void ExamineMustCrossAnd3Regions(int crossCount, int regionCount)
{
    Timer timer;
    timer.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
    GetAllSolutions(allSolutions);

    uint64_t minCount = allSolutions.size();
    int bestPathSize = 0;
    //	std::vector<uint64_t> best;
    int *crossItems = new int[crossCount];
    int *regionItems = new int[regionCount];
    Combinations<witness.GetNumPathConstraints()> c;
    //	Combinations<w.GetNumMustCrossConstraints()> c;
    Combinations<witness.GetNumSeparationConstraints()> regionCombs;

    {
        Witness<puzzleWidth, puzzleHeight> w;
        WitnessState<puzzleWidth, puzzleHeight> s;

        uint64_t maxRank = c.MaxRank(crossCount);
        for (uint64_t n = 0; n < maxRank; n++)
        {
            if (0 == n % 50000 && regionCount == 0) printf("%llu of %llu\n", n, maxRank);
            c.Unrank(n, crossItems, crossCount);
            for (int x = 0; x < crossCount; x++)
            {
                w.SetMustCrossConstraint(crossItems[x]);
            }
            uint64_t colorComb = pow(3, regionCount);
            for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount) * colorComb; t++)
            {
                uint64_t globalPuzzle = n * regionCombs.MaxRank(regionCount) * colorComb + t;
                if ((0 == globalPuzzle % 50000))
                    printf("-->%llu of %llu\n", globalPuzzle, maxRank * regionCombs.MaxRank(regionCount) * colorComb);
                uint64_t bits = t; // will only use bottom bits
                uint64_t hash = t / colorComb;

                //				// easy way to reduce symmetry
                //				if (t&1) continue;
                //
                if (0 != bits % 3) continue;
                if (1 != (bits / 3) % 3) continue;
                if (2 != (bits / 9) % 3) continue;

                w.ClearSeparationConstraints();
                regionCombs.Unrank(hash, regionItems, regionCount);
                for (int x = 0; x < regionCount; x++)
                {
                    int colour = bits % 3;
                    bits = bits / 3;
                    w.AddSeparationConstraint(
                        regionItems[x], (colour == 0) ? Colors::white : ((colour == 1) ? Colors::black : Colors::blue));
                }

                int pathSize = 0;
                int result = CountSolutions(w, allSolutions, pathSize, minCount + 1);

                if (result < minCount && result > 0)
                {
                    minCount = result;
                    bestPathSize = pathSize;
                    best.clear();
                    best.push_back(w);
                }
                else if (result == minCount && pathSize == bestPathSize)
                //				else if (result == minCount)
                {
                    best.push_back(w);
                }
            }

            w.ClearPathConstraints();
        }
    }

    printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize,
        timer.EndTimer());
    if (best.size() > 0)
    {
        currBoard = 0;
        Load(currBoard);
    }

    delete[] crossItems;
    delete[] regionItems;
}

void ExamineTetris(int count)
{
    Timer t;
    t.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

    Witness<puzzleWidth, puzzleHeight> wp;
    WitnessState<puzzleWidth, puzzleHeight> s;

    //	for (int y = 0; y < puzzleHeight+1; y++)
    //		for (int x = 0; x < puzzleWidth+1; x++)
    //			wp.AddMustCrossConstraint(x, y);

    GetAllSolutions(wp, allSolutions);

    uint64_t minCount = allSolutions.size();
    int *items = new int[count];
    Combinations<wp.GetNumTetrisConstraints()> c;

    const int pieceTypes = 24 + 2;

    uint64_t pCount = pow(pieceTypes, count);
    uint64_t maxRank = c.MaxRank(count) * pCount;
    for (uint64_t rank = 0; rank < maxRank; rank++)
    {
        wp.ClearTetrisConstraints();
        // wp.ClearTriangleConstraints();
        wp.ClearStarConstraints();
        wp.ClearSeparationConstraints();
        if (0 == rank % 50000) printf("%llu of %llu\n", rank, maxRank);
        uint64_t n = rank / pCount;      // arrangement on board
        uint64_t pieces = rank % pCount; // pieces in locations
        c.Unrank(n, items, count);
        bool t1 = false, t2 = false;
        for (int x = 0; x < count; x++)
        {
            //			if (x == 0 && count > 1)
            //			{
            //				wp.AddNegativeTetrisConstraint(items[x], (1+(pieces%24)));
            //			}
            //			else {
            if ((pieces % pieceTypes) < 24)
            {
                wp.AddTetrisConstraint(items[x], 1 + (pieces % pieceTypes));
                t1 = true;
            }
            else
            {
                // wp.AddStarConstraint(items[x], wp.tetrisYellow);
                // wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
                if ((pieces % pieceTypes) - 23)
                    wp.AddSeparationConstraint(items[x], Colors::black);
                else
                    wp.AddSeparationConstraint(items[x], Colors::white);
                // wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
                t2 = true;
            }
            //			}
            pieces /= pieceTypes;
        }
        if (!(t1 && t2)) continue;
        //		w = wp;
        //		if (rank == 2)
        //			break;
        int pathLen = 0;
        int result = CountSolutions(wp, allSolutions, pathLen, minCount + 1);
        if (result > minSolutions)
        {
            // ignore
        }
        else if (result < minCount && result > 0)
        {
            minCount = result;
            best.clear();
            best.push_back(wp);
            witness = wp;
        }
        else if (result == minCount)
        {
            best.push_back(wp);
        }
    }

    printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
    //	return;

    //	if (best.size() > 0)
    //	{
    //		currBoard = 0;
    //		Load(currBoard, count, 0);
    //	}

    delete[] items;
}

void ExamineTriangles(int count)
{
    Timer t;
    t.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;

    Witness<puzzleWidth, puzzleHeight> wp;
    WitnessState<puzzleWidth, puzzleHeight> s;

    //	for (int y = 0; y < puzzleHeight+1; y++)
    //		for (int x = 0; x < puzzleWidth+1; x++)
    //			wp.AddMustCrossConstraint(x, y);

    GetAllSolutions(wp, allSolutions);

    uint64_t minCount = allSolutions.size();
    int *items = new int[count];
    Combinations<wp.GetNumTriangleConstraints()> c;

    const int NUM_TRI = 4;
    uint64_t pCount = pow(NUM_TRI, count);
    uint64_t maxRank = c.MaxRank(count) * pCount;
    for (uint64_t rank = maxRank / 10; rank < maxRank && best.size() < 5000; rank += 1)
    {
        if (0 == rank % 50000) printf("%llu of %llu [%zu solutions]\n", rank, maxRank, best.size());
        uint64_t n = rank / pCount;      // arrangement on board
        uint64_t pieces = rank % pCount; // pieces in locations
        c.Unrank(n, items, count);
        for (int x = 0; x < count; x++)
        {
            if (0 == pieces % NUM_TRI)
            {
                wp.AddStarConstraint(items[x], wp.triangleColor);
            }
            else
            {
                wp.AddTriangleConstraint(items[x], 0 + (pieces % NUM_TRI));
            }
            pieces /= NUM_TRI;
        }
        int pathLen = 0;
        int result = CountSolutions(wp, allSolutions, pathLen, minCount + 1);
        if (result > minSolutions)
        {
            // ignore
        }
        else if (result < minCount && result > 0)
        {
            minCount = result;
            best.clear();
            best.push_back(wp);
            witness = wp;
        }
        else if (result == minCount)
        {
            best.push_back(wp);
        }
        wp.ClearStarConstraints();
        wp.ClearTriangleConstraints();
    }

    printf("\n%lu boards with %llu solutions; %1.2fs elapsed\n", best.size(), minCount, t.EndTimer());
    //	return;

    //	if (best.size() > 0)
    //	{
    //		currBoard = 0;
    //		Load(currBoard, count, 0);
    //	}

    delete[] items;
}

void ExamineRegionsAndStars(int count)
{
    Timer t;
    t.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
    GetAllSolutions(allSolutions);

    uint64_t minCount = allSolutions.size();
    int bestPathSize = 0;
    //	std::vector<uint64_t> best;
    std::vector<int> items(count);
    //	Combinations<w.GetNumStarConstraints()> c;
    Combinations<witness.GetNumPathConstraints()> mc;
    std::vector<int> forbidden;
    std::vector<int> currSolutions;

    {
        Witness<puzzleWidth, puzzleHeight> w;
        WitnessState<puzzleWidth, puzzleHeight> s;

        //        if (puzzleWidth != 4 || puzzleHeight != 4) {
        //            printf("This code only works for 4x4");
        //            return;
        //        }
        // static_assert((4==puzzleHeight)&&(puzzleWidth==4), "This code only works for 4x4");
        //		uint64_t variations = pow(4, count-1)*2;
        //		uint64_t maxRank = c.MaxRank(count)*variations;
        uint64_t variations = pow(4, 8);
        uint64_t mcRank = mc.MaxRank(count);
        uint64_t maxRank = mcRank * variations; // c.MaxRank(count)*variations;

        for (uint64_t n = 0; n < maxRank; n++)
        {
            if (0 == n % 50000) printf("%llu of %llu\n", n, maxRank);
            w.ClearInnerConstraints();
            w.ClearPathConstraints();
            //			uint64_t rankPart = n/variations;
            //			uint64_t varPart = n%variations;
            //			c.Unrank(rankPart, &items[0], count);
            //			for (int i : items)
            //			{
            //				bool colorPart = (varPart/2)%2;
            //				bool shapePart = varPart%2;
            //				varPart /= 4;
            //				if (shapePart)
            //					w.AddStarConstraint(i, colorPart?Colors::black:Colors::blue);
            //				else
            //					w.AddSeparationConstraint(i, colorPart?Colors::black:Colors::blue);
            //			}

            uint64_t rank = n % variations;
            for (int x = 0; x < 4; x++)
            {
                rgbColor color;
                color = (rank & 1) ? Colors::white : Colors::orange;
                if (rank & 2)
                    w.AddStarConstraint(x, 1, color);
                else
                {
                    w.AddTriangleConstraint(x, 1, 1 + (rank & 1));
                    // w.AddSeparationConstraint(x, 0, color);
                }
                rank >>= 2;

                color = (rank & 1) ? Colors::white : Colors::orange;
                if (rank & 2)
                    w.AddStarConstraint(x, 2, color);
                else
                {
                    //					w.AddSeparationConstraint(x, 3, color);
                    w.AddTriangleConstraint(x, 2, 1 + (rank & 1));
                }
                rank >>= 2;
            }
            //			for (int y = 0; y < 4; y+=3)
            //			{
            //				rgbColor color;
            //				color = (rank&2)?Colors::white:Colors::red;
            //
            //				if (rank&1)
            //					w.AddStarConstraint(0, y, color);
            //				else
            //					w.AddSeparationConstraint(0, y, color);
            //				rank>>=2;
            //				if (rank&1)
            //					w.AddStarConstraint(3, y, color);
            //				else
            //					w.AddSeparationConstraint(3, y, color);
            //				rank>>=2;
            //			}
            mc.Unrank(n / variations, &items[0], count);
            for (int x = 0; x < items.size(); x++)
            {
                w.SetMustCrossConstraint(items[x]);
            }

            int pathSize = 0;
            // int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
            int result = CountSolutions(w, allSolutions, currSolutions, forbidden, pathSize, minCount + 1);

            // don't return two puzzles with the same solution
            if (currSolutions.size() == 1) forbidden.push_back(currSolutions[0]);

            if (result < minCount && result > 0)
            {
                minCount = result;
                bestPathSize = pathSize;
                best.clear();
                best.push_back(w);
            }
            else if (result == minCount && pathSize == bestPathSize)
            //				else if (result == minCount)
            {
                best.push_back(w);
            }
        }
    }

    printf(
        "\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
    if (best.size() > 0)
    {
        currBoard = 0;
        Load(currBoard);
    }
}

// void ExamineRegions(int regionCount)
//{
//	Timer t;
//	t.StartTimer();
//	std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
//	GetAllSolutions(allSolutions);
//
//	uint64_t minCount = allSolutions.size();
//	int bestPathSize = 0;
//	//	std::vector<uint64_t> best;
//	int *regionItems = new int[regionCount];
//	Combinations<w.GetNumMustCrossConstraints()> c;
//	Combinations<w.GetNumSeparationConstraints()> regionCombs;
//
//	{
//
//		Witness<puzzleWidth, puzzleHeight> w;
//		WitnessState<puzzleWidth, puzzleHeight> s;
//
//		for (uint64_t t = 0; t < regionCombs.MaxRank(regionCount)*(1<<regionCount); t++)
//		{
//			if (0 == t%1000)
//				printf("-->%llu of %llu\n", t, regionCombs.MaxRank(regionCount)*(1<<regionCount));
//			uint64_t bits = t; // will only use bottom bits
//			uint64_t hash = t/(1<<regionCount);
//
//			// easy way to reduce symmetry
//			if (t&1) continue;
//
//			w.ClearSeparationConstraints();
//			regionCombs.Unrank(hash, regionItems, regionCount);
//			for (int x = 0; x < regionCount; x++)
//			{
//				w.AddSeparationConstraint(regionItems[x], ((bits>>x)&1)?Colors::white:Colors::black);
//			}
//
//			int pathSize = 0;
//			int result = CountSolutions(w, allSolutions, pathSize, minCount+1);
//			if (result > minSolutions)
//			{
//				// ignore
//			}
//			else if (result < minCount && result > 0)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize > bestPathSize)
//			{
//				minCount = result;
//				bestPathSize = pathSize;
//				best.clear();
//				otherbest.clear();
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//			else if (result == minCount && pathSize == bestPathSize)
//			{
//				best.push_back(n);
//				otherbest.push_back(t);
//			}
//		}
//
//		w.ClearMustCrossConstraints();
//	}
//
//	printf("\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize,
// t.EndTimer()); 	if (best.size() > 0)
//	{
//		currBoard = 0;
//		Load(currBoard, 0, regionCount);
//	}
//
//	delete [] regionItems;
// }

void ParallelExamineHelper(int count, int threadID, int numThreads)
{
    best.clear();
    currBoard = 0;
    Timer t;
    t.StartTimer();
    std::vector<WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
    GetAllSolutions(allSolutions);

    Witness<puzzleWidth, puzzleHeight> wp;
    WitnessState<puzzleWidth, puzzleHeight> s;
    // wp.SetGoal(puzzleWidth+1, puzzleHeight);

    uint64_t minCount = allSolutions.size();
    int bestPathSize = 0;
    std::vector<int> items(count);
    Combinations<wp.GetNumStarConstraints()> c;
    //	Combinations<wp.GetNumMustCrossConstraints()> mc;
    std::vector<int> forbidden;
    std::vector<int> currSolutions;

    // int *items = new int[count];

    const int pieceTypes = 3; // 24+2;

    uint64_t pCount = pow(pieceTypes, count);
    uint64_t maxRank = c.MaxRank(count) * pCount;
    for (uint64_t rank = threadID; rank < maxRank; rank += numThreads)
    {
        //		wp.ClearTetrisConstraints();
        //		wp.ClearStarConstraints();
        wp.ClearInnerConstraints();
        if (0 == rank % 50000) printf("%llu of %llu\n", rank, maxRank);
        uint64_t n = rank / pCount;      // arrangement on board
        uint64_t pieces = rank % pCount; // pieces in locations
        c.Unrank(n, &items[0], count);
        //		bool t1 = false, t2 = false, t3 = false, t4 = false, t5 = false, t6 = false;
        for (int x = 0; x < count; x++)
        {
            switch (pieces % pieceTypes)
            {
            case 0:
                wp.AddSeparationConstraint(items[x], Colors::orange);
                break;
            case 1:
                wp.AddStarConstraint(items[x], Colors::orange);
                break;
            case 2:
                wp.AddTriangleConstraint(items[x], 2);
                //					wp.AddSeparationConstraint(items[x], Colors::blue);
                break;
            case 3:
                wp.AddStarConstraint(items[x], Colors::blue);
                break;
            case 4:
                wp.AddSeparationConstraint(items[x], Colors::green); //
                break;
            case 5:
                wp.AddStarConstraint(items[x], Colors::green);
                break;
            }
            pieces /= pieceTypes;
        }
        //		if (0)
        //		{
        //			int x=0;
        //			if ((pieces%pieceTypes) < 24)
        //			{
        //				wp.AddTetrisConstraint(items[x], 1+(pieces%pieceTypes));
        //				t1 = true;
        //			}
        //			else {
        ////				wp.AddStarConstraint(items[x], wp.tetrisYellow);
        ////				wp.AddTriangleConstraint(items[x], (pieces%pieceTypes)-23);
        //				if ((pieces%pieceTypes)-24)
        //				{
        //					wp.AddSeparationConstraint(items[x], Colors::black);
        //					t2 = true;
        //				}
        //				else {
        //					wp.AddSeparationConstraint(items[x], Colors::white);
        //					t3 = true;
        //				}
        //			}
        //			pieces/=pieceTypes;
        //		}
        //		if (!(t1 && t2 && t3))// && t4 && t5 && t6))
        //			continue;

        int pathSize = 0;
        int result = CountSolutions(wp, allSolutions, pathSize, minCount + 1);
        // int result = CountSolutions(wp, allSolutions, currSolutions, forbidden, pathSize, minCount+1);

        // don't return two puzzles with the same solution
        //			if (currSolutions.size() == 1)
        //				forbidden.push_back(currSolutions[0]);

        if ((result < minCount && result > 0) || (result == minCount && pathSize > bestPathSize))
        {
            lock.lock();
            printf("Decreased number of solutions to %d / best path up to %d\n", result, bestPathSize);
            minCount = result;
            bestPathSize = pathSize;
            currBoard = 0;
            best.clear();
            best.push_back(wp);
            lock.unlock();
        }
        else if (result == minCount && pathSize == bestPathSize)
        //				else if (result == minCount)
        {
            lock.lock();
            best.push_back(wp);
            lock.unlock();
        }
    }

    printf(
        "\n%lu boards with %llu solutions len %d; %1.2fs elapsed\n", best.size(), minCount, bestPathSize, t.EndTimer());
    //	if (best.size() > 0)
    //	{
    //		currBoard = 0;
    //		Load(currBoard);
    //	}
    parallel = false;
}

void ParallelExamine(int count)
{
    const unsigned int numThreads = std::thread::hardware_concurrency();
    std::vector<std::thread *> t(numThreads);
    if (!parallel)
    {
        parallel = true;
        for (int x = 0; x < numThreads; x++)
        {
            delete t[x];
            t[x] = new std::thread(ParallelExamineHelper, count, x, numThreads);
        }
    }
}
