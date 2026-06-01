#include "astar.h"

template<>
void SFBDSAStar<graphState,graphMove>::OpenGLDraw_RegisterClosedState( QueueNode &q ) {
	if( OpenGLDrawState == NULL ) {
		unsigned int num_states = ((GraphEnvironment*)env)->GetGraph()->GetNumNodes();
		OpenGLDrawState = new bool[num_states];
		for( unsigned int i = 0; i < num_states; i++ )
			OpenGLDrawState[i] = false;
	}

	OpenGLDrawState[q.s1] = true;
	OpenGLDrawState[q.s2] = true;
	return;
}


template<>
void SFBDSAStar<graphState,graphMove>::OpenGLDraw( GLdouble rad ) const {

	if( OpenGLDrawState == NULL ) return;

	unsigned int num_states = ((GraphEnvironment*)env)->GetGraph()->GetNumNodes();

	// draw closed list in green
	glColor3f(0., 1., 0.);

	for( unsigned int i = 0; i < num_states; i++ ) {
		if( OpenGLDrawState[i] ) {
			GLdouble x,y,z;

			// draw start state
			node* n = ((GraphEnvironment*)env)->GetGraph()->GetNode( i );
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
			DrawSphere( x, y, z, rad );
		}
	}

	return;
};

template<>
void SFBDSAStar<graphState,graphMove>::OpenGLDraw( QueueNode q, bool side, GLdouble rad ) const {

	GLdouble x,y,z;

	if( side )
		glColor3f(1.,0.,0.);
	else
		glColor3f(0.,0.,1.);
	// draw the start
	node *n = ((GraphEnvironment*)env)->GetGraph()->GetNode( q.s1 );
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	DrawSphere( x, y, z, rad );

	if( side )
		glColor3f(0.,0.,1.);
	else
		glColor3f(1.,0.,0.);
	n = ((GraphEnvironment*)env)->GetGraph()->GetNode( q.s2 );
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	DrawSphere( x, y, z, rad );

	return;
};


template<>
void SFBDSAStar<xyLoc,tDirection>::OpenGLDraw( GLdouble rad ) const {
	// todo
};

template<>
void SFBDSAStar<xyLoc,tDirection>::OpenGLDraw( QueueNode q, bool side, GLdouble rad ) const {
	// todo
};

