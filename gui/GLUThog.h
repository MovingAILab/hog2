/*
 *  $Id: main.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#ifndef OS_MAC

/* Apple CGSize */
struct CGSize {
   float width;
   float height;
};

typedef struct CGSize CGSize;


/* Apple CGPoint */
struct CGPoint {
   float x;
   float y;
};
typedef struct CGPoint CGPoint;


/* Apple CGRect */
struct CGRect {
   CGPoint origin;
   CGSize size;
};
typedef struct CGRect CGRect;

#endif

void createMenus();
void processMenuEvents(int option);
void keyPressed(unsigned char key, int x, int y);
void mouseMovedNoButton(int x, int y);
void mouseMovedButton(int x, int y);
void mousePressedButton(int button, int state, int x, int y);
static void mousePan (int x, int y, pRecContext pContextInfo);
static void mouseDolly (int x, int y, pRecContext pContextInfo);
void renderScene(void);
void buildGL(void);
void drawGL(pRecContext pContextInfo);
//void trajectoryDrawGL(pRecContext pContextInfo);  // WESHACK - for drawing trajectories at the end of our 'multipleAStarMM' simulation run
void drawCStringGL (char * cstrOut, GLuint fontList);
static void drawInfo (pRecContext pContextInfo);
void resizeWindow(int x, int y);
void resizeGL(pRecContext pContextInfo, CGRect viewRect);
//void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void updateModelView (pRecContext pContextInfo);
void pointPath();
int processFramesPerSecond(char *argument[], int maxNumArgs);

void SetLighting(GLfloat ambientf = 0.2f, GLfloat diffusef = 1.0f, GLfloat specularf = 1.0f);

pRecContext getCurrentContext();
//static void drawCaps (pRecContext pContextInfo) {}
