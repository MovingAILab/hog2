#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.1 2006/10/20 18:16:56 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/shared/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = shared
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/utils -I$(ROOT)/graph -I$(ROOT)/simulation -I$(ROOT)/abstraction -I$(ROOT)/absmapalgorithms -I$(ROOT)/abstractionalgorithms -I$(ROOT)/graphalgorithms -I$(ROOT)/algorithms -I$(ROOT)/environments  -I$(ROOT)/search
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	shared/PathGeneration.cpp \
	shared/SearchUnit.cpp \
	shared/SharedAMapGroup.cpp
	#shared/HumanUnit.cpp \
	#shared/PatrolUnit.cpp \
	#shared/PRAStarUnit.cpp \
	#shared/RewardUnit.cpp \
