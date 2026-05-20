#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.3 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/utils/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = utils
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/graph -I$(ROOT)/gui -I$(ROOT)/utils
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	utils/BitVector.cpp \
	utils/BitMap.cpp \
	utils/FPUtil.cpp \
	utils/FunctionApproximator.cpp \
	utils/Colors.cpp \
	utils/LinearRegression.cpp \
	utils/Map.cpp \
	utils/MapOverlay.cpp \
	utils/Plot2D.cpp \
	utils/ScenarioLoader.cpp \
	utils/StatCollection.cpp \
	utils/StatUtil.cpp \
	utils/Timer.cpp \
	utils/SwapEndian.cpp \
	utils/StringUtils.cpp \
	utils/FourBitArray.cpp \
	utils/EnvUtil.cpp \
	utils/TextOverlay.cpp \
	utils/MMapUtil.cpp \
	utils/DiskBitFile.cpp \
	utils/Bloom.cpp \
	utils/MinBloom.cpp \
	utils/MapGenerators.cpp \
	utils/MMapUtil.cpp \
	utils/NBitArray.cpp \
	utils/SVGUtil.cpp \
	utils/RangeCompression.cpp \
	utils/MR1Permutation.cpp \
	utils/Graphics.cpp \
	utils/ScreenTransition.cpp \
	utils/Json.cpp \
	utils/MonoFont.cpp \
	utils/FileUtil.cpp \
