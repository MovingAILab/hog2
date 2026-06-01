#include "PRAStarMapUnit.h"
#include "GLUtil.h"

template<>
tDirection PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::nomove() {
	return kStay;
};

template<>
tDirection PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::nomove() {
	return kStay;
};

template<>
graphMove PraStarMapUnit<graphState,graphMove,GraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};

template<>
graphMove PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};


template<>
void PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::OpenGLDraw( const MapEnvironment *env, const SimulationInfo<xyLoc,tDirection,MapEnvironment>* ) const {
	GLdouble xx, yy, zz, rad;
	Map *m = env->GetMap();
	if( current_pos.x >= m->GetMapWidth() || current_pos.y >= m->GetMapHeight() ) {
		fprintf( stderr, "Warning: PraStarMapUnit is out of bounds. Could not draw it.\n" );
		return;
	}
	m->GetOpenGLCoord( current_pos.x, current_pos.y, xx, yy, zz, rad );
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( xx, yy, zz, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( xx, yy, zz-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			m->GetOpenGLCoord( pathcache[i].x, pathcache[i].y, xx, yy, zz, rad );
			glVertex3f( xx, yy, zz-rad/2 );
		}
		glEnd();
	}

	return;
};

template<>
void PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::OpenGLDraw( const AbsMapEnvironment *env, const SimulationInfo<xyLoc,tDirection,AbsMapEnvironment>* ) const {
	GLdouble xx, yy, zz, rad;
	Map *m = env->GetMap();
	if( current_pos.x >= m->GetMapWidth() || current_pos.y >= m->GetMapHeight() ) {
		fprintf( stderr, "Warning: PraStarMapUnit is out of bounds. Could not draw it.\n" );
		return;
	}
	m->GetOpenGLCoord( current_pos.x, current_pos.y, xx, yy, zz, rad );
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( xx, yy, zz, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( xx, yy, zz-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			m->GetOpenGLCoord( pathcache[i].x, pathcache[i].y, xx, yy, zz, rad );
			glVertex3f( xx, yy, zz-rad/2 );
		}
		glEnd();
	}

	return;
};

template<>
void PraStarMapUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( const GraphEnvironment *env, const SimulationInfo<graphState,graphMove,GraphEnvironment>* ) const {
	/*
	// TODO: make the code below compilable
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.005;
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( x, y, z-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			n  = env->GetGraph()->GetNode( pathcache[i] );
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
			glVertex3f( x, y, z-rad/2 );
		}
		glEnd();
	}
	*/
	return;

};

template<>
void PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( const AbstractionGraphEnvironment *env, const SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) const {
	/*
	// TODO: make the code below compilable
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.001;
	x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
	y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
	z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 1., 0. ); // turn green when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( x, y, z-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			n  = env->GetGraph()->GetNode( pathcache[i] );
			x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
			y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
			z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
			glVertex3f( x, y, z-rad/2 );
		}
		glEnd();
	}
	*/
	return;

};

template<>
void PraStarMapUnit<xyLoc,tDirection,MapEnvironment>::GetPraStarPath( MapEnvironment *env, xyLoc robber_pos ) {
	node *rn = abs->GetNodeFromMap( robber_pos.x, robber_pos.y );
	node *cn = abs->GetNodeFromMap( current_pos.x, current_pos.y );

	// call PRA*
	path *prapath = pra->GetPath( abs, cn, rn );
	path *temppath = prapath;

	// follow the path maximally half the steps of the path
	unsigned int steps = prapath->length() / 2;
	if( steps < cop_speed )
		steps = cop_speed;
	else
		steps -= steps%cop_speed;

	// now put together the path for all the calls
	for( unsigned int i = 0; i < steps; i++ ) {
		if( temppath->next != NULL ) {
			node *n = temppath->next->n;
			int x, y;
			abs->GetTileFromNode( n, x, y );
			pathcache.push_back( xyLoc( x, y ) );
			temppath = temppath->next;
		} else
			break;
	}

	printf( "pathcache: " );
	for( unsigned int i = 0; i < pathcache.size(); i++ ) {
		printf( "(%u,%u) ", pathcache[i].x, pathcache[i].y );
	}
	printf( "\n" );

	// cleanup
	delete prapath;

	return;
};

template<>
void PraStarMapUnit<xyLoc,tDirection,AbsMapEnvironment>::GetPraStarPath( AbsMapEnvironment *env, xyLoc robber_pos ) {
	node *rn = abs->GetNodeFromMap( robber_pos.x, robber_pos.y );
	node *cn = abs->GetNodeFromMap( current_pos.x, current_pos.y );

	// call PRA*
	path *prapath = pra->GetPath( abs, cn, rn );
	path *temppath = prapath;

	// follow the path maximally half the steps of the path
	unsigned int steps = prapath->length() / 2;
	if( steps < cop_speed )
		steps = cop_speed;
	else
		steps -= steps%cop_speed;

	// now put together the path for all the calls
	for( unsigned int i = 0; i < steps; i++ ) {
		if( temppath->next != NULL ) {
			node *n = temppath->next->n;
			int x, y;
			abs->GetTileFromNode( n, x, y );
			pathcache.push_back( xyLoc( x, y ) );
			temppath = temppath->next;
		} else
			break;
	}

	// cleanup
	delete prapath;

	return;
};

template<>
void PraStarMapUnit<graphState,graphMove,GraphEnvironment>::GetPraStarPath( GraphEnvironment *env, graphState robber_pos ) {
	node *rn = env->GetGraph()->GetNode( robber_pos );
	node *cn = env->GetGraph()->GetNode( current_pos );

	// call PRA*
	path *prapath = pra->GetPath( abs, cn, rn );
	path *temppath = prapath;

	// follow the path maximally half the steps of the path
	unsigned int steps = prapath->length() / 2;
	if( steps < cop_speed )
		steps = cop_speed;
	else
		steps -= steps%cop_speed;

	// now put together the path for all the calls
	for( unsigned int i = 0; i < steps; i++ ) {
		if( temppath->next != NULL ) {
			pathcache.push_back( temppath->next->n->GetNum() );
			temppath = temppath->next;
		} else
			break;
	}

	// cleanup
	delete prapath;
	return;
};

template<>
void PraStarMapUnit<graphState,graphMove,AbstractionGraphEnvironment>::GetPraStarPath( AbstractionGraphEnvironment *env, graphState robber_pos ) {
	node *rn = env->GetGraph()->GetNode( robber_pos );
	node *cn = env->GetGraph()->GetNode( current_pos );

	// call PRA*
	path *prapath = pra->GetPath( abs, cn, rn );
	path *temppath = prapath;

	// follow the path maximally half the steps of the path
	unsigned int steps = prapath->length() / 2;
	if( steps < cop_speed )
		steps = cop_speed;
	else
		steps -= steps%cop_speed;

	// now put together the path for all the calls
	for( unsigned int i = 0; i < steps; i++ ) {
		if( temppath->next != NULL ) {
			pathcache.push_back( temppath->next->n->GetNum() );
			temppath = temppath->next;
		} else
			break;
	}

	// cleanup
	delete prapath;
	return;
};
