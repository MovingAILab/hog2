#include "PRAStarGraphUnit.h"
#include "GLUtil.h"

template<>
graphMove PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};

template<>
graphMove PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};


template<>
void PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( const GraphEnvironment *env, const SimulationInfo<graphState,graphMove,GraphEnvironment>* ) const {
	/*
	// TODO: make the code below compilable
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.001;
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
void PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( const AbstractionGraphEnvironment *env, const SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) const {
	/*
	// TODO: make the code below compilable
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.01; //env->Scale()/2.;
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
void PraStarGraphUnit<graphState,graphMove,GraphEnvironment>::GetPraStarPath( GraphEnvironment *env, graphState robber_pos ) {
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
void PraStarGraphUnit<graphState,graphMove,AbstractionGraphEnvironment>::GetPraStarPath( AbstractionGraphEnvironment *env, graphState robber_pos ) {
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
