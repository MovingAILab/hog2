//
//  ExamineUtil.h
//  The Witness Editor
//
//  Created by Samarium on 2023-06-29.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef THE_WITNESS_EDITOR_INCLUDE_EXAMINE_UTIL_H
#define THE_WITNESS_EDITOR_INCLUDE_EXAMINE_UTIL_H

void Load(uint64_t which);

void ExamineMustCross(int count);

void ExamineMustCrossAndRegions(int crossCount, int regionCount);

void ExamineMustCrossAnd3Regions(int crossCount, int regionCount);

void ExamineTetris(int count);

void ExamineTriangles(int count);

void ExamineRegionsAndStars(int count);

void ParallelExamine(int count);

#endif /* THE_WITNESS_EDITOR_INCLUDE_EXAMINE_UTIL_H */
