#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/graph/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = environments
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/graph -I$(ROOT)/envutil -I$(ROOT)/environments -I$(ROOT)/utils -I$(ROOT)/abstraction -I$(ROOT)/simulation -I$(ROOT)/graphalgorithms  -I$(ROOT)/generic -I$(ROOT)/algorithms -I$(ROOT)/search -I$(ROOT)/gui
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	environments/Airplane.cpp \
	environments/GraphEnvironment.cpp \
	environments/GraphRefinementEnvironment.cpp \
	environments/Map2DEnvironment.cpp \
	environments/PermutationPuzzleEnvironment.cpp \
	environments/MNPuzzle.cpp \
	environments/FlipSide.cpp \
	environments/Directional2DEnvironment.cpp \
	environments/TopSpin.cpp	\
	environments/RoboticArm.cpp  \
	environments/ConfigEnvironment.cpp \
	environments/Map3DGrid.cpp \
	environments/Map2DHeading.cpp \
	environments/MinimalSectorAbstraction.cpp \
	environments/MNAgentPuzzle.cpp \
	environments/RC.cpp \
	environments/RubiksCubeEdges.cpp \
	environments/RubiksCube7Edges.cpp \
	environments/RubiksCubeCorners.cpp \
	environments/RubiksCube.cpp \
	environments/CanonicalGrid.cpp \
	environments/Hexagon.cpp \
	environments/Fling.cpp \
	environments/Voxels.cpp \
	environments/VoxelGrid.cpp \
	environments/NaryTree.cpp \
	environments/RoadMap.cpp \
	environments/SnakeBird.cpp \
	environments/Racetrack.cpp \
	environments/DynamicWeightedGrid.cpp \
#	environments/WeightedMap2DEnvironment.cpp \
#	environments/PancakePuzzle.cpp \
