/*
 *  $Id: StatCollection.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/1/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <vector>
#include <string>

#ifndef STATCOLLECTION_H
#define STATCOLLECTION_H

typedef union { double fval; long lval; } statValue;
enum storedType { floatStored, longStored };
// renamed from "stat" due to conflict with c file stat
class statistics {
public:
	int category, owner;
	statValue value;
	storedType sType;
};

/**
* The StatCollection class is for collecting stats across different parts of
 * the simulation. This class aggregates results and allows access to the
 * collected information.
 */ 

class StatCollection {
public:
	StatCollection();
	~StatCollection();
	void AddStat(const char *category, const char *owner, double value);
	void AddStat(const char *category, const char *owner, long value);
	void SumStat(const char *category, const char *owner, double value);
	void SumStat(const char *category, const char *owner, long value);
	
	void ClearAllStats();
	//	void clearOwnerStats(const char *owner); // not define for now; can be defined if needed
	//	void clearCategoryStats(const char *category);
	
	int GetNumStats() const;
	const statistics *GetStatNum(int which) const;
	
	int LookupCategory(const char *category) const;
	int LookupOwner(const char *owner) const;
	const char *lookupCategoryID(int id) const;
	const char *LookupOwnerID(int id) const;
	bool LookupStat(const char *category, const char *owner, statValue &) const;
	bool LookupStat(unsigned int index, statValue &) const;
	
	void AddFilter(const char *category); // this is an include filter
	void AddIncludeFilter(const char *category); // include only added categories
	void AddExcludeFilter(const char *category); // exclude only added categories
	void ClearFilters();
	void EnablePrintOutput(bool pO) { printOutput = pO; }
	
	int FindNextStat(const char *category, const char *owner, int startIndex=0) const;
	int FindPrevStat(const char *category, const char *owner, int startIndex=-1) const;
	int FindNextStat(const char *what, bool findCategory, int startIndex = 0) const;
	int FindPrevStat(const char *what, bool findCategory, int startIndex = -1) const;
	
	void PrintStatsTable() const;
	
private:
		int addCategory(const char *category);
	int addOwner(const char *owner);
	bool passFilter(const char *category) const;
	statValue *getLastStat(const char *category, const char *owner);
	
	std::vector<std::string> categories;
	std::vector<std::string> owners;
	std::vector<std::string> includeFilters;
	std::vector<std::string> excludeFilters;
	std::vector<statistics> stats;
	bool printOutput;
};

#endif
