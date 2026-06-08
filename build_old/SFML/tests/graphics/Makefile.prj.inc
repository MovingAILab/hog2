#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/10/20 20:20:15 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/apps/nathan/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = GraphicTest
DBG_NAME = $(NAME)
REL_NAME = $(NAME)

ROOT = ../../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
TEST_OBJDIR = $(ROOT)/objs/$(NAME)/tests
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release
TEST_BINDIR = $(REL_BINDIR)

PROJ_CXXFLAGS = -I$(ROOT)/absmapalgorithms -I$(ROOT)/graphalgorithms -I$(ROOT)/gui -I$(ROOT)/simulation -I$(ROOT)/environments -I$(ROOT)/mapalgorithms -I$(ROOT)/algorithms -I$(ROOT)/generic -I$(ROOT)/utils -I$(ROOT)/graph -I$(ROOT)/search

PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_TEST_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS = -L$(DBG_BINDIR)
PROJ_REL_LNFLAGS = -L$(REL_BINDIR)
PROJ_TEST_LNFLAGS = -L$(TEST_BINDIR)

PROJ_DBG_LIB = -lgraph -lenvironments -lenvutil -lmapalgorithms -lalgorithms -lgraphalgorithms -lgui -lutils
PROJ_REL_LIB = -lgraph -lenvironments -lenvutil -lmapalgorithms -lalgorithms -lgraphalgorithms -lgui -lutils
PROJ_TEST_LIB = -lgraph -lenvironments -lenvutil -lmapalgorithms -lalgorithms -lgraphalgorithms -lgui -lutils


PROJ_DBG_DEP = \
  $(DBG_BINDIR)/libutils.a \
  $(DBG_BINDIR)/libgraph.a \
  $(DBG_BINDIR)/libgui.a \
  $(DBG_BINDIR)/libenvironments.a \
  $(DBG_BINDIR)/libenvutil.a \
  $(DBG_BINDIR)/libmapalgorithms.a \
  $(DBG_BINDIR)/libgraphalgorithms.a \
  $(DBG_BINDIR)/libalgorithms.a \


PROJ_REL_DEP = \
  $(REL_BINDIR)/libutils.a \
  $(REL_BINDIR)/libgraph.a \
  $(REL_BINDIR)/libgui.a \
  $(REL_BINDIR)/libenvironments.a \
  $(REL_BINDIR)/libenvutil.a \
  $(REL_BINDIR)/libmapalgorithms.a \
  $(REL_BINDIR)/libgraphalgorithms.a \
  $(REL_BINDIR)/libalgorithms.a \


PROJ_TEST_DEP = \
  $(REL_BINDIR)/libutils.a \
  $(REL_BINDIR)/libgraph.a \
  $(REL_BINDIR)/libgui.a \
  $(REL_BINDIR)/libenvironments.a \
  $(REL_BINDIR)/libenvutil.a \
  $(REL_BINDIR)/libmapalgorithms.a \
  $(REL_BINDIR)/libgraphalgorithms.a \
  $(REL_BINDIR)/libalgorithms.a \


ifeq ("$(OPENGL)", "STUB")
PROJ_DBG_LIB += -lSTUB
PROJ_REL_LIB += -lSTUB
PROJ_DBG_DEP +=   $(DBG_BINDIR)/libSTUB.a
PROJ_REL_DEP +=   $(REL_BINDIR)/libSTUB.a
endif

default : all

SRC_CPP = \
  tests/graphics/Driver.cpp \
