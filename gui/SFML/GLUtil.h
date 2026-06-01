/*
 *  $Id: glUtil.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/8/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "FPUtil.h"

// #ifdef __APPLE__
// #include "TargetConditionals.h"
// #endif

#ifdef NO_OPENGL
#include "gl.h"
#include "glut.h"
#else

//#ifdef TARGET_OS_IPHONE
//#include <OpenGLES/ES1/gl.h>
//#include <OpenGLES/ES1/glext.h>
//#define GLdouble GLfloat
//#else

#ifdef OS_MAC
#include <OpenGL/gl.h>
//#include <OpenGL/glu.h>
//#include <GLUT/glut.h>
//#include <AGL/agl.h>
#else

#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/glut.h>
#endif

//#endif
#endif

#ifndef GLUTIL_H
#define GLUTIL_H

//class point3d;

#include "Colors.h"
#include "Graphics.h"
#include "Constants.h"
#include <SFML/OpenGL.hpp>

//#pragma mark -
//#pragma mark OpenGL structures:




/** Draw a pyramid with the tip at the given location, given height, and 
* width from center to edge as width.
*/
void DrawPyramid(GLfloat x, GLfloat y, GLfloat z, GLfloat height, GLfloat width);
void DrawBox(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);
void DrawBoxFrame(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat rad);
void DrawCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, int segments = 32, float rotation = 0);
void FrameCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, GLdouble lineWidth, int segments = 32, float rotation = 0);
void DrawSphere(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawSquare(GLdouble _x, GLdouble _y, GLdouble _z, GLdouble tRadius);
void DrawCylinder(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat innerRad, GLfloat outerRad, GLfloat height);
void OutlineRect(GLdouble left, GLdouble top, GLdouble right, GLdouble bottom, double zz);

void DrawText(double x, double y, double z, double scale, const char *res);
void DrawTextCentered(double x, double y, double z, double scale, const char *res);

void SetLighting(GLfloat ambientf = 0.2f, GLfloat diffusef = 1.0f, GLfloat specularf = 1.0f);


#endif
