/*
 *  $Id: StatCollection.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/1/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "StatCollection.h"
#include <cstring>
#include <cstdio>


// typedef union { double fval; long lval; } statValue;

//class stat {
//public:
//	int category, owner;
//	statValue value;
//};

//std::vector<char *> categories;
//std::vector<char *> owners;
//std::vector<stat> stats;

StatCollection::StatCollection()
:categories(), owners(), stats()
{
	printOutput = false;
}

StatCollection::~StatCollection()
{
	/*
	for (unsigned int x = 0; x < categories.size(); x++)
	{
		delete [] categories[x];
		categories[x] = 0;
	}

	for (unsigned int x = 0; x < owners.size(); x++)
	{
		delete [] owners[x];
		owners[x] = 0;
	}

	for (unsigned int x = 0; x < excludeFilters.size(); x++)
	{
		delete [] excludeFilters[x];
		excludeFilters[x] = 0;
	}

	for (unsigned int x = 0; x < includeFilters.size(); x++)
	{
		delete [] includeFilters[x];
		includeFilters[x] = 0;
	}
	*/
}

/**
* Add a new stat entry for the given category, owner and value.
 */
void StatCollection::AddStat(const char *category, const char *owner, double value)
{
	if (!passFilter(category))
		return;
	int catID, ownerID;
	catID = addCategory(category);
	ownerID = addOwner(owner);
	stats.resize(stats.size()+1);
	stats[stats.size()-1].category = catID;
	stats[stats.size()-1].owner = ownerID;
	stats[stats.size()-1].value.fval = value;
	stats[stats.size()-1].sType = floatStored;
	
	if (printOutput)
		printf("%s\t%s\t%1.2f\n", category, owner, value);
}

/**
* Add a new stat entry for the given category, owner and value.
 */
void StatCollection::AddStat(const char *category, const char *owner, long value)
{
	if (!passFilter(category))
		return;
	int catID, ownerID;
	catID = addCategory(category);
	ownerID = addOwner(owner);
	stats.resize(stats.size()+1);
	stats[stats.size()-1].category = catID;
	stats[stats.size()-1].owner = ownerID;
	stats[stats.size()-1].value.lval = value;
	stats[stats.size()-1].sType = longStored;
	if (printOutput)
		printf("%s\t%s\t%ld\n", category, owner, value);
}

/**
* Given stats for the category and owner, find an existing stat (chronologically
																 * backwards search) with the same category and owner, and add this stat to the existing
 * value. If the stat doesn't exist with the same category and owner doesn't exist,
 * a new one will be initialized with the given value.
 */
void StatCollection::SumStat(const char *category, const char *owner, double value)
{
	statValue *sv = getLastStat(category, owner);
	if (sv)
	{
//		if (sv->sType != floatStored)
//			printf("Warning: Adding double to value previous stored as long\n");
		sv->fval += value;
		if (printOutput)
			printf("%s\t%s\t%1.2f\n", category, owner, sv->fval);
	}
	else
		AddStat(category, owner, value);
}

/**
* Given stats for the category and owner, find an existing stat (chronologically
																 * backwards search) with the same category and owner, and add this stat to the existing
 * value. If the stat doesn't exist with the same category and owner doesn't exist,
 * a new one will be initialized with the given value.
 */
void StatCollection::SumStat(const char *category, const char *owner, long value)
{
	statValue *sv = getLastStat(category, owner);
	if (sv)
	{
//		if (sv->sType != floatStored)
//			printf("Warning: Adding long to value previous stored as double\n");
		sv->lval += value;
		if (printOutput)
			printf("%s\t%s\t%ld\n", category, owner, sv->lval);
	}
	else
		AddStat(category, owner, value);
}

/**
* Remove all stat entries from the collection.
 */
void StatCollection::ClearAllStats()
{
	stats.resize(0);
}

//void StatCollection::clearOwnerStats(char *owner);
//void StatCollection::clearCategoryStats(char *category);
/**
* The number of stats collected so far
 */
int StatCollection::GetNumStats() const
{
	return (int)stats.size();
}

/**
* Return the nth stat which has been collected.
 */
const statistics *StatCollection::GetStatNum(int which) const
{
	return &stats[which];
}

/**
* Given a category ID, return the text description.
 */
const char *StatCollection::lookupCategoryID(int id) const
{
	return categories[id].c_str();
}

/**
* Given a owner ID, return the text description.
 */
const char *StatCollection::LookupOwnerID(int id) const
{
	return owners[id].c_str();
}

/**
* Adding a filter will cause only stats of the given category to be
 * collected. All other stats added will be ignored. As many categories
 * can be added as needed.
 */
void StatCollection::AddFilter(const char *category)
{
	AddIncludeFilter(category);
}

void StatCollection::AddIncludeFilter(const char *category) // include only added categories
{
	includeFilters.push_back(category);
	/*
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	includeFilters.push_back(str);
	*/
}

void StatCollection::AddExcludeFilter(const char *category) // exclude only added categories
{
	excludeFilters.push_back(category);
	/*
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	excludeFilters.push_back(str);
	*/
}

/**
* Clear any filters being used for stat entry.
 */
void StatCollection::ClearFilters()
{
	/*
	for (unsigned int x = 0; x < excludeFilters.size(); x++)
	{
			delete [] excludeFilters[x];
			excludeFilters[x] = 0;
	}
	for (unsigned int x = 0; x < includeFilters.size(); x++)
	{
			delete [] includeFilters[x];
			includeFilters[x] = 0;
	}
	*/
	excludeFilters.resize(0);
	includeFilters.resize(0);
}

/**
* Given a category, look up the ID. O(# categories) operation. If not found, returns -1.
 */
int StatCollection::LookupCategory(const char *category) const
{
	for (unsigned int x = 0; x < categories.size(); x++)
	{
		if (categories[x] == category)
			return x;
	}
	//if (strcmp(category, categories[x]) == 0)
	//return x;
	return -1;
}


/**
* Add a new category to the category list. If the category exists, returns the id.
 * Otherwise creates the category and returns the id.
 */
int StatCollection::addCategory(const char *category)
{
	int id = LookupCategory(category);
	if (id != -1)
		return id;
	categories.push_back(category);
	/*
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	categories.push_back(str);
	*/
	return categories.size()-1;
}

/**
* Given an owner, look up the ID. O(# owners) operation. If not found, returns -1.
 */
int StatCollection::LookupOwner(const char *owner) const
{
	for (unsigned int x = 0; x < owners.size(); x++)
		if (owners[x] == owner)
			return x;
		//if (strcmp(owner, owners[x]) == 0)
		//return x;
	return -1;
}

/**
* Add a new owner to the owner list. If the owner exists, returns the id.
 * Otherwise creates the owner and returns the id.
 */
int StatCollection::addOwner(const char *owner)
{
	int id = LookupOwner(owner);
	if (id != -1)
		return id;
	owners.push_back(owner);
	/*
	char *str = new char [strlen(owner)+1];
	strcpy(str, owner);
	owners.push_back(str);
	*/
	return owners.size()-1;
}

/**
* Find the last stat entered that matches the category and owner. Returns
 * copy of stat entry. Returns true if a stat was found and false otherwise.
 */
bool StatCollection::LookupStat(const char *category, const char *owner, statValue &v) const
{
	if (!passFilter(category))
	{
		return false;
	}
	int catID, ownerID;
	catID = LookupCategory(category);
	ownerID = LookupOwner(owner);
	for (int x = (int)stats.size()-1; x >= 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
		{
			v = stats[x].value;
			return true;
		}
	}
	return false;
}

bool StatCollection::LookupStat(unsigned int index, statValue &v) const
{
	if (index < stats.size())
	{
		v = stats[index].value;
		return true;
	}
	return false;
}

/**
* Find the last stat entered that matches the category and owner. Returns pointer
 * to entry.
 */
statValue *StatCollection::getLastStat(const char *category, const char *owner)
{
	if (!passFilter(category))
		return 0;
	int catID, ownerID;
	catID = LookupCategory(category);
	ownerID = LookupOwner(owner);
	for (int x = (int)stats.size()-1; x >= 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return &stats[x].value;
	}
	return 0;
}

/**
* Check to see if the category stats should be saved.
 */
bool StatCollection::passFilter(const char *category) const
{
	if ((includeFilters.size() == 0) && (excludeFilters.size() == 0))
		return true;

	for (unsigned int x = 0; x < excludeFilters.size(); x++)
		if (excludeFilters[x] == category)
			return false;
	//if (strcmp(excludeFilters[x], category) == 0)
	//return false;

	if (includeFilters.size() == 0)
		return true;

	for (unsigned int x = 0; x < includeFilters.size(); x++)
	{
		if (includeFilters[x] == category)
			return true;
		//if (strcmp(includeFilters[x], category) == 0)
		//return true;
	}

	return false;
}

/**
* Find the next stat entry that matches the given category and owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int StatCollection::FindNextStat(const char *category, const char *owner, int startIndex) const
{
	// Check if the category has been registered
	if (!passFilter(category))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to zero
	if (startIndex < 0 || startIndex > (int)stats.size()-1)
		startIndex = 0;
	
	int catID, ownerID;
	catID = LookupCategory(category);
	ownerID = LookupOwner(owner);
	
	for (int x = startIndex; x < (int)stats.size(); x++)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return x;
	}
	
	return -1;     
}

/**
* Find the previous stat entry that matches the given category and owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int StatCollection::FindPrevStat(const char *category, const char *owner, int startIndex) const
{
	// Check if the category has been registered
	if (!passFilter(category))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to the last
	// index
	if (startIndex < 0 || startIndex > (int)stats.size() - 1)
		startIndex = (int) stats.size() -1;
	
	int catID, ownerID;
	catID = LookupCategory(category);
	ownerID = LookupOwner(owner);
	
	for (int x = startIndex; x > 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return x;
	}
	
	return -1;
}

/**
* Find the next stat entry that matches the category if findCategory is true, otherwise
 * look to match the owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int StatCollection::FindNextStat(const char *what, bool findCategory, int startIndex) const
{
	// Check if the category has been registered
	if (findCategory && (!passFilter(what)))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to zero
	if (startIndex < 0 || startIndex > (int)stats.size()-1)
		startIndex = 0;
	
	int ID;
	if (findCategory)
		ID = LookupCategory(what);
	else
		ID = LookupOwner(what);
	
	for (int x = startIndex; x < (int)stats.size(); x++)
	{
		if ((findCategory && (stats[x].category == ID)) ||
				(!findCategory && (stats[x].owner == ID)))
			return x;
	}
	
	return -1;     
}

/**
* Find the next stat entry that matches the category if findCategory is true, otherwise
 * look to match the owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int StatCollection::FindPrevStat(const char *what, bool findCategory, int startIndex) const
{
	// Check if the category has been registered
	if (findCategory && (!passFilter(what)))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to the last
	// index
	if (startIndex < 0 || startIndex > (int)stats.size() - 1)
		startIndex = (int) stats.size() -1;
	
	int ID;
	if (findCategory)
		ID = LookupCategory(what);
	else
		ID = LookupOwner(what);
	
	for (int x = startIndex; x > 0; x--)
	{
		if ((findCategory && (stats[x].category == ID)) ||
				(!findCategory && (stats[x].owner == ID)))
			return x;
	}
	
	return -1;
}


/*
 * Print the Stats Table for debugging purpose for now.
 */
void StatCollection::PrintStatsTable() const
{
	printf("Stats Table:\n----------------------------\n");
	
	for (int x = 0; x < (int)stats.size(); x++)
		if (stats[x].sType == floatStored)
			printf("%d \t%s \t%s \t%le\n", x, categories[stats[x].category].c_str(),
						 owners[stats[x].owner].c_str(), stats[x].value.fval);
		else
			printf("%d \t%s \t%s \t%ld\n", x, categories[stats[x].category].c_str(),
						 owners[stats[x].owner].c_str(), stats[x].value.lval);
}
