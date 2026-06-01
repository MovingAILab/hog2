#include "Unit.h"
#include "Map2DEnvironment.h"
#include "AbsMapUnit.h"
#include "MultilevelCopRobberGame.h"

#ifndef ROBBERUNIT_H
#define ROBBERUNIT_H

#define ROBBERUNIT_EPSILON 0.01

/*!
	Provides a basic unit that simulates the action of a robber by using precomputed
	expected reward fields for all possible states (cop and robber positions).
	This Unit is implemented for a AbsMapEnvironment but can be used for other environments
	with slight modifications
*/
class RobberUnit : public BaseAbsMapUnit { // Unit<xyLoc, tDirection, AbsMapEnvironment> {
public:
	RobberUnit( xyLoc loc, MultilevelCopRobberGame *game, const char* filename );
	RobberUnit( xyLoc loc, MultilevelCopRobberGame *game, const char* filename, std::vector<unsigned int> copunits );
	RobberUnit( xyLoc loc, MultilevelCopRobberGame *game, unsigned int maxlevel = 1, unsigned int maxiter = 0, double gamma = 0.99, double epsilon = 0.01, double precision = 0.1 );
	virtual ~RobberUnit();

	virtual void SetCopUnits( std::vector<unsigned int> copunits );

	virtual const char *GetName() { return "RobberUnit"; };

	virtual bool MakeMove(AbsMapEnvironment *, OccupancyInterface<xyLoc,tDirection> *, AbsMapSimulationInfo *info, tDirection &a);

	virtual void UpdateLocation(AbsMapEnvironment *, xyLoc &l, bool, AbsMapSimulationInfo *);
	virtual void GetLocation(xyLoc &l) { l = loc; };
	virtual void OpenGLDraw(int window, AbsMapEnvironment *, AbsMapSimulationInfo *);
	virtual void GetGoal(xyLoc &s) { s = loc; };
	virtual bool Done() { return done; };
protected:

	virtual CopRobberGame::CRAction MakeAlternatingMove( CopRobberGame::CRState s, MarkovGame<graphState, graphMove> *mgame );
	virtual CopRobberGame::CRAction MakeSimultaneousMove( CopRobberGame::CRState s, MarkovGame<graphState, graphMove> *mgame );

	GLfloat r, g, b;
	xyLoc loc;
	std::vector<unsigned int> copunits;
	MultilevelCopRobberGame *game;
	double **expected_rewards;
	unsigned int num_levels;
	bool done;
};

#endif
