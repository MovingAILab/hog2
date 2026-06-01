/*
 *  $Id: statUtil.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "StatCollection.h"

void setupAverageRatio(StatCollection *stats, char *stat1, char *stat2);
void measureAverageRatio(StatCollection *stats);

double SumStatEntries(StatCollection *stats, const char *category, const char *owner);
double averageStatEntries(StatCollection *stats, const char *category, const char *owner);
double stdevStatEntries(StatCollection *stats, const char *category, const char *owner);
double maxStatEntries(StatCollection *stats, const char *category, const char *owner);
long unsigned countStatEntries(StatCollection *stats, const char *category, const char *owner);
