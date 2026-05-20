#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.3 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/utils/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = gui
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/utils -I$(ROOT)/gui
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS) -I/usr/local/include/SFML/
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS) -I/usr/local/include/SFML/

PROJ_DBG_LNFLAGS = -lsfml-graphics -lsfml-window -lsfml-system
PROJ_REL_LNFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

ifeq ("$(OPENGL)", "STUB")
SRC_CPP = \
  gui/Common.cpp \
  gui/STUB/STUB_HOG.cpp \
  gui/Trackball.cpp

else

SRC_CPP = \
  gui/Common.cpp \
  gui/SFML/SFML_HOG.cpp \
  gui/SFML/TextBox.cpp \
  gui/SFML/GLUtil.cpp \
  gui/Trackball.cpp

endif

