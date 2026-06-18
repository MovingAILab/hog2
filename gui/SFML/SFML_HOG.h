/*
 *  $Id: main.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 06/22/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef SFML_HOG_H
#define SFML_HOG_H

#include "Common.h"
#include "GLUtil.h"

typedef struct {
	GLfloat worldRotation[4];
	GLfloat cameraRotation[4];
} recRotation;

typedef struct {
	GLdouble left, right, top, bottom, near, far;
} recFrustum;

typedef struct {
	Graphics::point viewPos; // View position

	bool thirdPerson;
	
	// third-person (?) camera that can be programatically controlled using gluLookAt
	Graphics::point viewDir; // View direction vector
	Graphics::point viewUp; // View up direction
	recRotation rotations; // object/camera rotations

	// Other settings
	recFrustum frust; // set in updateProjection
	GLdouble aperture; // camera aperture
	GLint viewWidth,viewHeight; // current window/screen height and width
	GLfloat viewOriginX, viewOriginY; // always 0
} recCamera;

// single set of interaction flags and states
extern GLfloat gTrackBallRotation [4];
extern pRecContext gTrackingContextInfo;

void createMenus();
void processMenuEvents(int option);
void keyPressed(unsigned char key, int x, int y);
void mouseMovedNoButton(int x, int y);
void mouseMovedButton(int x, int y);
void mousePressedButton(int button, int state, int x, int y);
static void mousePan (int x, int y, pRecContext pContextInfo);
static void mouseDolly (int x, int y, pRecContext pContextInfo);
void renderScene(void);
void buildGL(int x, int y);
void drawGL (pRecContext pContextInfo, sf::RenderWindow &window);
void drawCStringGL (char * cstrOut, GLuint fontList);
static void drawInfo (pRecContext pContextInfo);
void resizeWindow(int x, int y);
void resizeGL(pRecContext pContextInfo, int width, int height);
//void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void updateModelView (pRecContext pContextInfo);
void pointPath();

pRecContext getCurrentContext();
//static void drawCaps (pRecContext pContextInfo) {}

void setPortCamera(pRecContext pContextInfo, int currPort);
void setViewport(pRecContext pContextInfo, int currPort);
void updateModelView(pRecContext pContextInfo, int currPort);
void cameraLookAt(GLfloat, GLfloat, GLfloat, float cameraSpeed = 0.1, int port = -1);
Graphics::point cameraLookingAt(int port = -1);
void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1, int port = -1);
void cameraOffset(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed = 0.1, int port = -1);
void resetCamera();
void resetCamera(recCamera * pCamera);

void SaveScreenshot(unsigned long windowID, const char *filename);
void SetZoom(int windowID, float amount);
//point3d GetOGLPos(pRecContext pContextInfo, int x, int y);
Graphics::point GetHeading(unsigned long windowID, int which);
void GetHeading(unsigned long windowID, int which, float &hx, float &hy, float &hz);

//void rotateObject();

#endif // SFML_HOG_H
