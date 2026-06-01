/*
 *  $Id: statUtil.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/9/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "StatUtil.h"
#include <math.h>
#include <cstdio>

inline double max(double a, double b)
{ if (a > b) return a; return b; }

/**
* Setup the stats so a ratio between two stats for each unit can be
 * properly measured.
 */
void setupAverageRatio(StatCollection *stats, char *stat1, char *stat2)
{
	stats->ClearAllStats();
	stats->ClearFilters();
	stats->AddIncludeFilter(stat1);
	stats->AddIncludeFilter(stat2);
	stats->AddIncludeFilter("simulationTime");
}

/**
* Measures the ration between the two stats set-up from
 * setupAverageRatio.
 *
 */
void measureAverageRatio(StatCollection *stats)
{
//	stats->PrintStatsTable();
	
	int stat1Type = -1, stat2Type = -1;
	std::vector<const statistics *> stat1;
	std::vector<const statistics *> stat2;
	int currOwner = -1;
	
	for (int x = 0; x < stats->GetNumStats(); x++)
	{
		const statistics *s = stats->GetStatNum(x);
		if (s->owner == stats->LookupOwner("unitSimulation"))
		{
			while (stat1.size() > stat2.size())
				stat1.pop_back();
			while (stat2.size() > stat1.size())
				stat2.pop_back();
			continue;
		}
		if (currOwner == -1)
		{
			currOwner = s->owner;
			printf("Collecting stats for %s\n", stats->LookupOwnerID(currOwner));
		}
		if (s->owner != currOwner)
			continue;
		
		if (stat1Type == -1)
		{
			stat1Type = s->category;
			stat1.push_back(s);
		}
		else if (stat1Type == s->category)
		{
			stat1.push_back(s);
		}
		else if (stat2Type == -1)
		{
			stat2Type = s->category;
			stat2.push_back(s);
		}
		else if (stat2Type == s->category)
		{
			stat2.push_back(s);
		}
	}
	
	while (stat1.size() > stat2.size())
		stat1.pop_back();
	while (stat2.size() > stat1.size())
		stat2.pop_back();
	
	double sum1=0.0, sum2=0.0;
	
	for (unsigned int x = 0; x < stat1.size(); x++)
	{
		if (stat1[x]->sType == longStored)
			sum1 += (double)stat1[x]->value.lval;
		else
			sum1 += stat1[x]->value.fval;
		
		if (stat2[x]->sType == longStored)
			sum2 += (double)stat2[x]->value.lval;
		else
			sum2 += stat2[x]->value.fval;
	}
	
//	printf("Final table of values:\n");
//	for (unsigned int x = 0; x < stat1.size(); x++)
//	{
//		if (stat1[x]->sType == longStored)
//			printf("%6d\t", stat1[x]->value.lval);
//		else
//			printf("%1.4f\t", stat1[x]->value.fval);
//		
//		if (stat2[x]->sType == longStored)
//			printf("%6d\n", stat2[x]->value.lval);
//		else
//			printf("%1.4f\n", stat2[x]->value.fval);
//	}
	
	
	printf("Measuring ratio between %s and %s as collected by %s\n",
				 stats->lookupCategoryID(stat1Type),
				 stats->lookupCategoryID(stat2Type),
				 stats->LookupOwnerID(currOwner));
	printf("%d stats collected\n%s average %1.4e\n%s average %1.4e\n",
				 (int)stat1.size(), stats->lookupCategoryID(stat1Type), 
				 (double)sum1/stat1.size(), stats->lookupCategoryID(stat2Type), 
				 (double)sum2/stat2.size());
	printf("Ratios:\n%s / %s: %1.4e\n%s / %s: %1.4e\n",
				 stats->lookupCategoryID(stat1Type), stats->lookupCategoryID(stat2Type),
				 sum1/sum2,
				 stats->lookupCategoryID(stat2Type), stats->lookupCategoryID(stat1Type),
				 sum2/sum1);
}


/**
 * Sum the values of all stat entries for the same (category, owner).
 * Type is automatically known from the entry.
 */
double SumStatEntries(StatCollection *stats, const char *category, const char *owner)
{
	double sum = 0.0;
	int catID, ownerID;
	catID = stats->LookupCategory(category);
	ownerID = stats->LookupOwner(owner);
	
	for (int x = 0; x < (int)stats->GetNumStats(); x++)
	{
		if ((stats->GetStatNum(x)->category == catID) && (stats->GetStatNum(x)->owner == ownerID))
		{
			if (stats->GetStatNum(x)->sType == floatStored)
				sum += stats->GetStatNum(x)->value.fval;
			else
				sum += (double)stats->GetStatNum(x)->value.lval;
		}
	}
	
	return sum;
}

double maxStatEntries(StatCollection *stats, const char *category, const char *owner)
{
	double maxval = -9999999999.9;
	int catID, ownerID;
	catID = stats->LookupCategory(category);
	ownerID = stats->LookupOwner(owner);
	
	for (int x = 0; x < (int)stats->GetNumStats(); x++)
	{
		if ((stats->GetStatNum(x)->category == catID) && (stats->GetStatNum(x)->owner == ownerID))
		{
			if (stats->GetStatNum(x)->sType == floatStored)
				maxval = max(maxval, stats->GetStatNum(x)->value.fval);
			else
				maxval = max(maxval, (double)stats->GetStatNum(x)->value.lval);
		}
	}
	return maxval;
}

/** Count the number of state instances in the stat collection */
long unsigned countStatEntries(StatCollection *stats, const char *category, const char *owner)
{
	long unsigned count = 0;
	int catID, ownerID;
	catID = stats->LookupCategory(category);
	ownerID = stats->LookupOwner(owner);
	
	for (int x = 0; x < (int)stats->GetNumStats(); x++)
		if ((stats->GetStatNum(x)->category == catID) && (stats->GetStatNum(x)->owner == ownerID))
			count++;
				
	return count;
}

double averageStatEntries(StatCollection *stats, const char *category, const char *owner)
{
	double sum = 0.0;
	double count = 0.0;
	int catID, ownerID;
	catID = stats->LookupCategory(category);
	ownerID = stats->LookupOwner(owner);
	
	for (int x = 0; x < (int)stats->GetNumStats(); x++)
	{
		if ((stats->GetStatNum(x)->category == catID) && (stats->GetStatNum(x)->owner == ownerID))
		{
			if (stats->GetStatNum(x)->sType == floatStored)
				sum += stats->GetStatNum(x)->value.fval;
			else
				sum += (double)stats->GetStatNum(x)->value.lval;
			count++;
		}
	}
	if (count > 0)
		return sum/count;
	return 0;
}

double stdevStatEntries(StatCollection *stats, const char *category, const char *owner)
{
	double average = averageStatEntries(stats, category, owner);
	double stdev = 0;
	double count = 0;
	int catID, ownerID;
	catID = stats->LookupCategory(category);
	ownerID = stats->LookupOwner(owner);
	
	for (int x = 0; x < (int)stats->GetNumStats(); x++)
	{
		if ((stats->GetStatNum(x)->category == catID) && (stats->GetStatNum(x)->owner == ownerID))
		{
			if (stats->GetStatNum(x)->sType == floatStored)
				stdev += (average-stats->GetStatNum(x)->value.fval)*
					(average-stats->GetStatNum(x)->value.fval);
			else
				stdev += (average-(double)stats->GetStatNum(x)->value.lval)*
					(average-(double)stats->GetStatNum(x)->value.lval);
			count++;
		}
	}
	if (count > 0)
		return sqrt(stdev/count);
	return 0;	
}
