#include "gl.h"

void glBegin ( GLenum mode ) {};
void glCallList ( GLuint list ) {};
void glColor4f ( GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha ) {};
void glDeleteLists ( GLuint list, GLsizei range ) {};
void glEnd () {};
void glEndList () {}; 
GLuint glGenLists ( GLsizei range ) {return 0;};
void glNewList ( GLuint list, GLenum mode ) {};
void glNormal3f ( GLfloat nx, GLfloat ny, GLfloat nz ) {};
void glVertex3f ( GLfloat x, GLfloat y, GLfloat z ) {};
void glVertex3d ( GLdouble x, GLdouble y, GLdouble z ) {};
void glColor3f ( GLfloat red, GLfloat green, GLfloat blue ) {};
void glColor3fv ( const GLfloat *v ) {}; 
void glGetFloatv ( GLenum pname, GLfloat *params ) {};
void glLineWidth ( GLfloat width ) {};
void glTranslatef ( GLfloat x, GLfloat y, GLfloat z ) {};
void glDisable ( GLenum cap ) {};
void glEnable ( GLenum cap ) {};
void glBlendFunc ( GLenum sfactor, GLenum dfactor ) {};
void glGetIntegerv ( GLenum pname, GLint *params ) {};
void glLoadIdentity () {};
void glMatrixMode ( GLenum mode ) {};
void glPopMatrix () {};
void glPushMatrix () {};
void glTranslated ( GLdouble x, GLdouble y, GLdouble z ) {};
void glVertex2f ( GLfloat x, GLfloat y ) {};
void glColorMaterial ( GLenum face, GLenum mode ) {};
void glGetDoublev ( GLenum pname, GLdouble *params ) {};
void glLightModeli ( GLenum pname, GLint param ) {};
void glLightfv ( GLenum light, GLenum pname, const GLfloat *params ) {};
void glMaterialfv ( GLenum face, GLenum pname, const GLfloat *params ) {};
void glReadPixels ( GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid *pixels ) {};
int gluUnProject (GLdouble winx, GLdouble winy, GLdouble winz,
                 const GLdouble modelMatrix[16], const GLdouble projMatrix[16], const GLint viewport[4],
                 GLdouble *objx, GLdouble *objy, GLdouble *objz) { return 0;};
void glClear ( GLbitfield mask ) {};
void glClearColor ( GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha ) {};
void glFrontFace( GLenum mode ) {};
void glFrustum ( GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, 
		 GLdouble near_val, GLdouble far_val ) {};
void glGetBooleanv ( GLenum pname, GLboolean *params ) {};
const GLubyte* glGetString ( GLenum name ) { return 0; };
GLboolean glIsEnabled ( GLenum cap ) {return false;};
void glPolygonMode ( GLenum face, GLenum mode ) {};
void glPolygonOffset ( GLfloat factor, GLfloat units ) {};
void glRasterPos3d ( GLdouble x, GLdouble y, GLdouble z ) {};
void glRotatef ( GLfloat angle, GLfloat x, GLfloat y, GLfloat z ) {};
void glScalef ( GLfloat x, GLfloat y, GLfloat z ) {};
void glShadeModel ( GLenum mode ) {};
void glViewport ( GLint x, GLint y, GLsizei width, GLsizei height ) {};

void glDepthMask(GLboolean b) {};
void glGenTextures(GLsizei n, GLuint *texture) {};
void glBindTexture(GLenum target, GLuint texture) {};
GLint gluBuild2DMipmaps(GLenum target, GLint internalFormat, GLsizei width, GLsizei height, GLenum format, GLenum type, const void *data) { return 0; };
void glTexParameteri(GLenum target, GLenum pname, GLint param) {};
void glNormal3fv(const GLfloat *v) {};
void glCullFace(GLenum mode) {};
void glTexCoord2f(GLfloat s, GLfloat t) {};

void glVertex2i(GLint x, GLint y) {}
void glColor3ub(GLubyte red, GLubyte green, GLubyte blue) { }
void glNormal3d(GLdouble nx, GLdouble ny, GLdouble nz) { }
void glVertex2d(GLdouble x, GLdouble y) { }
void glOrtho(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top, GLdouble zNear, GLdouble zFar) { }

void glTexImage1D (GLenum target, GLint level, GLint internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const GLvoid *pixels) { }
void glTexImage2D (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *pixels) { }
void glTexImage3D (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid *pixels) { }

void glDisableClientState (GLenum array) { }
void glVertexPointer (GLint size, GLenum type, GLsizei stride, const GLvoid *pointer) { }
void glColorPointer( GLint size, GLenum type, GLsizei stride, const GLvoid *pointer ) {}
void glEnableClientState (GLenum array) { }

void glDrawArrays (GLenum mode, GLint first, GLsizei count) { }
void glNormalPointer (GLenum type, GLsizei stride, const GLvoid *pointer) { }
void glDrawElements (GLenum mode, GLsizei count, GLenum type, const GLvoid *indices) { }

void glMultMatrixd(	const GLdouble * m) {}
void glMultMatrixf(	const GLfloat * m) {}
