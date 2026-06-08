#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/graph/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = abstractionalgorithms
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/graphalgorithms -I$(ROOT)/algorithms -I$(ROOT)/graph -I$(ROOT)/utils -I$(ROOT)/abstraction -I$(ROOT)/simulation -I$(ROOT)/search
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	abstractionalgorithms/AStar2.cpp \
	abstractionalgorithms/AStar3.cpp \
	abstractionalgorithms/AStar.cpp \
	abstractionalgorithms/CorridorAStar.cpp \
	abstractionalgorithms/CRAStar.cpp \
	abstractionalgorithms/HPAStar.cpp \
	abstractionalgorithms/PRAStar2.cpp \
	abstractionalgorithms/PRAStar.cpp \
	abstractionalgorithms/SpreadPRAStar.cpp \
	abstractionalgorithms/SearchAlgorithm.cpp \
	abstractionalgorithms/FringeSearch.cpp \
	abstractionalgorithms/CFOptimalRefinement.cpp \
	abstractionalgorithms/IRDijkstra.cpp \
	abstractionalgorithms/IRAStar.cpp 




