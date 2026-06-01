/*
 *  $Id: reservationProvider.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/13/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef RESERVATIONPROVIDER_H
#define RESERVATIONPROVIDER_H

#include <stdint.h>

// template <class state, class action>
// class OccupancyInterface {
// public:
// 	virtual ~OccupancyInterface() {}
// 	virtual void SetStateOccupied(state, bool) = 0;
// 	virtual bool GetStateOccupied(state) = 0;
// 	virtual bool CanMove(state, state);
// 	virtual bool CanMove(state, state, double, uint32_t ID) = 0;
// 	virtual bool ReserveMove(state, state, double, uint32_t ID) = 0;
// 	virtual bool ClearMove(state, state, double, uint32_t ID) = 0;
// 	virtual bool ClearStates() = 0;
// };

#include "Graph.h"
class unit;

class reservationProvider {
public:
	virtual ~reservationProvider() {};
	//virtual bool tileOccupied(int x, int y) = 0;	
	virtual bool nodeOccupied(node *) = 0;

	virtual bool canMove(node *from, node *to, double startTime, unit *) = 0;
	virtual bool reserveMove(node *from, node *to, double startTime, unit *) = 0;
	virtual bool clearMove(node *from, node *to, double startTime, unit *) = 0;
	virtual void clearAllReservations() = 0;
};

#include "Unit.h"

#endif

