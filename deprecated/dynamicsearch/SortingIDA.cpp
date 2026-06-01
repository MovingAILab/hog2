#include "SortingIDA.h"

#include <math.h>
#include "FPUtil.h"

bool IndexAndCostComp (IndexAndCost i,IndexAndCost j) { return (i.second < j.second); }
