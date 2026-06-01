#include <iostream>
#include <fstream>
#include "TOH.h"
#include <vector>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"
#include <fstream>
#include <math.h>
#include <cmath>




template <int patternDisks, int totalDisks>
class TOHPDBWrapper : public PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>> {
public:
    
	TOHPDBWrapper(TOH<patternDisks> *e, const TOHState<totalDisks> &s)
	:PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>(e) { this->SetGoal(s); }
	//:PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>(e) { this->SetGoal(s); }
	virtual ~TOHPDBWrapper() {}

	int offset;
	TOHPDB<patternDisks, totalDisks>* basePDB;

	void SetBasePDB(TOHPDB<patternDisks, totalDisks>* _pdb)
	{
		basePDB = _pdb;
	}

	void SetOffset(int _offset)
	{
		offset = _offset;
	}

	TOHState<totalDisks> GetStateFromAbstractState(TOHState<patternDisks> &start) const
	{
		int diff = totalDisks - patternDisks;
		TOHState<totalDisks> tmp;
		for (int x = 0; x < 4; x++)
		{
			tmp.counts[x] = start.counts[x];
			for (int y = 0; y < tmp.counts[x]; y++)
			{
				tmp.disks[x][y] = start.disks[x][y]+diff-offset;
			}
		}
		return tmp;
	}

	virtual uint64_t GetAbstractHash(const TOHState<totalDisks> &s, int threadID = 0) const
	{
		int diff = totalDisks - patternDisks;
		uint64_t hash = 0;
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			{
				// 6 total 2 pattern
				if ((s.GetDiskOnPeg(x, y) > diff-offset) && (s.GetDiskOnPeg(x, y) <= totalDisks-offset))
					hash |= (uint64_t(x)<<(2*(s.GetDiskOnPeg(x, y)-1-diff+offset)));
			}
		}
		return hash;
	}

	virtual uint64_t GetPDBSize() const
	{
		return 1ull<<(2*patternDisks);
	}
	virtual uint64_t GetPDBHash(const TOHState<patternDisks> &s, int threadID = 0) const
	{
		return this->env->GetStateHash(s);
	}
	virtual void GetStateFromPDBHash(uint64_t hash, TOHState<patternDisks> &s, int threadID = 0) const
	{
		this->env->GetStateFromHash(hash, s);
	}
	
	virtual bool Load(const char *prefix)
	{
		return false;
	}
	virtual void Save(const char *prefix)
	{
		FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
		if (f == 0)
		{
			fprintf(stderr, "Error saving");
			return;
		}
		PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>::Save(f);
		fclose(f);
	}
	
	virtual std::string GetFileName(const char *prefix)
	{
		std::string s = prefix;
		s += "TOH4+"+std::to_string(offset)+"+"+std::to_string(patternDisks)+"+"+std::to_string(totalDisks)+".pdb";
		return s;
	}

	double HCost(const TOHState<totalDisks> &a, const TOHState<totalDisks> &b) const
	{
		return basePDB->PDB.Get(GetAbstractHash(a)); //PDB[GetPDBHash(a)];
	}
};