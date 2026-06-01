//
//  Constraint.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/15/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef Constraint_h
#define Constraint_h

template <class state>
class Constraint {
public:
	virtual bool ShouldNotGenerate(const state &start, const state &parent, const state &current, double gCost, const state &goal) const = 0;
private:
};

#endif /* Constraint_h */
