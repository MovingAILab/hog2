#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/10/20 20:20:15 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/apps/sample/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = coprobber
DBG_NAME = $(NAME)
REL_NAME = $(NAME)

ROOT = ../../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/absmapalgorithms -I$(ROOT)/graphalgorithms -I$(ROOT)/shared -I$(ROOT)/abstraction -I$(ROOT)/gui -I$(ROOT)/simulation -I$(ROOT)/abstractionalgorithms -I$(ROOT)/environments -I$(ROOT)/mapalgorithms -I$(ROOT)/algorithms -I$(ROOT)/generic -I$(ROOT)/utils -I$(ROOT)/graph -I$(ROOT)/search

PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS = -L$(DBG_BINDIR)
PROJ_REL_LNFLAGS = -L$(REL_BINDIR)

PROJ_DBG_LIB = -lgsl -lgslcblas -lm -lshared -labstraction -lgraph -labstractionalgorithms -lenvironments -lmapalgorithms -lalgorithms -labsmapalgorithms -lgraphalgorithms -lgui -lutils
PROJ_REL_LIB = -lgsl -lgslcblas -lm -lshared -labstraction -lgraph -labstractionalgorithms -lenvironments -lmapalgorithms -lalgorithms -labsmapalgorithms -lgraphalgorithms -lgui -lutils


PROJ_DBG_DEP = \
  $(DBG_BINDIR)/libutils.a \
  $(DBG_BINDIR)/libgraph.a \
  $(DBG_BINDIR)/libabstraction.a \
  $(DBG_BINDIR)/libgui.a \
  $(DBG_BINDIR)/libabstractionalgorithms.a \
  $(DBG_BINDIR)/libenvironments.a \
  $(DBG_BINDIR)/libmapalgorithms.a \
  $(DBG_BINDIR)/libabsmapalgorithms.a \
  $(DBG_BINDIR)/libgraphalgorithms.a \
  $(DBG_BINDIR)/libalgorithms.a \
  $(DBG_BINDIR)/libshared.a 


PROJ_REL_DEP = \
  $(REL_BINDIR)/libutils.a \
  $(REL_BINDIR)/libgraph.a \
  $(REL_BINDIR)/libabstraction.a \
  $(REL_BINDIR)/libgui.a \
  $(REL_BINDIR)/libabstractionalgorithms.a \
  $(REL_BINDIR)/libenvironments.a \
  $(REL_BINDIR)/libmapalgorithms.a \
  $(REL_BINDIR)/libabsmapalgorithms.a \
  $(REL_BINDIR)/libgraphalgorithms.a \
  $(REL_BINDIR)/libalgorithms.a \
  $(REL_BINDIR)/libshared.a 

ifeq ("$(OPENGL)", "STUB")
PROJ_DBG_LIB += -lSTUB
PROJ_REL_LIB += -lSTUB
PROJ_DBG_DEP +=   $(DBG_BINDIR)/libSTUB.a
PROJ_REL_DEP +=   $(REL_BINDIR)/libSTUB.a
endif

default : all

SRC_CPP = \
	apps/coprobber/JRobinson.cpp \
	apps/coprobber/MyHash.cpp \
	apps/coprobber/CopRobberGame.cpp \
	apps/coprobber/MultilevelCopRobberGame.cpp \
	apps/coprobber/RobberUnit.cpp \
	apps/coprobber/MySearchUnit.cpp \
	apps/coprobber/Minimax.cpp \
	apps/coprobber/Minimax_optimized.cpp \
	apps/coprobber/TIDAStar_optimized.cpp \
	apps/coprobber/Dijkstra.cpp \
	apps/coprobber/MinimaxAStar.cpp \
	apps/coprobber/MinimaxAStar_optimized.cpp \
	apps/coprobber/TwoPlayerDijkstra.cpp \
	apps/coprobber/DSCREnvironment.cpp \
	apps/coprobber/DSDijkstra.cpp \
	apps/coprobber/DSRMAStar.cpp \
	apps/coprobber/DSDijkstra_MemOptim.cpp \
	apps/coprobber/DSIDFPN.cpp \
	apps/coprobber/DSDAM.cpp \
	apps/coprobber/DSIDAM.cpp \
	apps/coprobber/DSIDAM2.cpp \
	apps/coprobber/DSDATPDijkstra.cpp \
	apps/coprobber/DSRandomBeacons.cpp \
	apps/coprobber/DSPRAStarCop.cpp \
	apps/coprobber/dscrsimulation/TrailMaxUnit.cpp \
	apps/coprobber/dscrsimulation/PRAStarMapUnit.cpp \
	apps/coprobber/dscrsimulation/PRAStarGraphUnit.cpp \
	apps/coprobber/dscrsimulation/OptimalUnit.cpp \
	apps/coprobber/TwoCopsDijkstra.cpp \
	apps/coprobber/TwoCopsDijkstra2.cpp \
	apps/coprobber/TwoCopsRMAStar.cpp \
	apps/coprobber/TwoCopsTIDAStar.cpp \
	apps/coprobber/DSBestResponse.cpp \
	apps/coprobber/CopRobber.cpp
