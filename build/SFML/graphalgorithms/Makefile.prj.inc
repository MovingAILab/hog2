#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/graph/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = graphalgorithms
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/graph -I$(ROOT)/abstraction -I$(ROOT)/utils -I$(ROOT)/abstractionalgorithms -I$(ROOT)/graphalgorithms -I$(ROOT)/simulation -I$(ROOT)/environments -I$(ROOT)/algorithms   -I$(ROOT)/search
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	graphalgorithms/Heap.cpp \
	graphalgorithms/Path.cpp \
	graphalgorithms/MeroB.cpp \
	graphalgorithms/Propagation.cpp \
	graphalgorithms/AStarDelay.cpp \
	graphalgorithms/FloydWarshall.cpp \
	graphalgorithms/Prim.cpp \



