#-----------------------------------------------------------------------------
# GNU Makefile: common part for all targets
#
# $Id: Makefile.com.inc,v 1.4 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile.com.inc,v $
#-----------------------------------------------------------------------------

SHELL = /bin/bash
CXX = em++ # clang++
COMMON_CXXFLAGS += -Wall -std=gnu++17 # -stdlib=libc++

#COMMON_CXXFLAGS += -ansi -pedantic

CC = emcc
COMMON_CFLAGS += -Wall

LN = $(CXX)
#COMMON_LNFLAGS += -stdlib=libc++

AR = emar

COMMON_CXXFLAGS += -I$(ROOT)/gui/ -I$(ROOT)/gui/STUB/GL/ -DNO_OPENGL -pthread

# if Linux need some other pthread call
ifeq ($(findstring "Linux", "$(shell uname -s)"), "Linux")
COMMON_LNFLAGS += -Lapps/libs -Ldemos/libs -L/usr/X11R6/lib64 -L/usr/X11R6/lib -pthread
else
COMMON_LNFLAGS += -Lapps/libs -Ldemos/libs -L/usr/X11R6/lib64 -L/usr/X11R6/lib -lpthread
endif

# COMMON_CXXFLAGS += -I$(ROOT)/gui/ -I$(ROOT)/gui/GL/ -DNO_OPENGL

DBG_CXXFLAGS = $(PROJ_DBG_CXXFLAGS) $(COMMON_CXXFLAGS) -g
REL_CXXFLAGS = $(PROJ_REL_CXXFLAGS) $(COMMON_CXXFLAGS) -g -O3 #-DNDEBUG

DBG_CFLAGS = $(PROJ_DBG_CFLAGS) $(COMMON_CFLAGS) -g
REL_CFLAGS = $(PROJ_REL_CFLAGS) $(COMMON_CFLAGS) -g -O3 #-DNDEBUG

DBG_LNFLAGS = $(PROJ_DBG_LNFLAGS) $(COMMON_LNFLAGS) -g -sEXPORTED_FUNCTIONS="['_InitHOG', '_InitHOGArg', '_DoFrame', '_MouseEvent', '_HitKey', '_SetCanvasSize']" -sEXPORTED_RUNTIME_METHODS="['cwrap', 'stringToNewUTF8']" -sASSERTIONS=1 -sINITIAL_MEMORY=200MB -sPTHREAD_POOL_SIZE="navigator.hardwareConcurrency"
REL_LNFLAGS = $(PROJ_REL_LNFLAGS) $(COMMON_LNFLAGS) -g -O3 -sEXPORTED_FUNCTIONS="['_InitHOG', '_InitHOGArg', '_DoFrame', '_MouseEvent', '_HitKey', '_SetCanvasSize']" -sEXPORTED_RUNTIME_METHODS="['cwrap', 'stringToNewUTF8']" -sASSERTIONS=1 -sINITIAL_MEMORY=200MB -sPTHREAD_POOL_SIZE="navigator.hardwareConcurrency"

DBG_OBJS = \
  $(patsubst %.cpp, $(DBG_OBJDIR)/%.o, $(SRC_CPP)) \
  $(patsubst %.c, $(DBG_OBJDIR)/%.o, $(SRC_C))
REL_OBJS = \
  $(patsubst %.cpp, $(REL_OBJDIR)/%.o, $(SRC_CPP)) \
  $(patsubst %.c, $(REL_OBJDIR)/%.o, $(SRC_C))

#-----------------------------------------------------------------------------

$(DBG_OBJDIR)/%.o: %.cpp
	@echo "$(notdir $@) (dbg)"
	mkdir -p $(dir $@)
	$(CXX) -c $(DBG_CXXFLAGS) -o $@ $<

$(REL_OBJDIR)/%.o: %.cpp
	@echo "$(notdir $@) (rel)"
	mkdir -p $(dir $@)
	$(CXX) -c $(REL_CXXFLAGS) -o $@ $<

$(DBG_OBJDIR)/%.o: %.c
	@echo "$(notdir $@) (dbg)"
	mkdir -p $(dir $@)
	$(CC) -c $(DBG_CFLAGS) -o $@ $<

$(REL_OBJDIR)/%.o: %.c
	@echo "$(notdir $@) (rel)"
	mkdir -p $(dir $@)
	$(CC) -c $(REL_CFLAGS) -o $@ $<

# Creation of dependency files using the compiler with -MM option and
# sed for postprocessing of its output (including the dependency file in
# the target section and adding the correct path to the object file).
# Dependency files are separated for each project, even if the
# object files are shared, since the compiler generated dependency lines
# contain relative paths.

$(DBG_OBJDIR)/%.d: %.cpp
	@echo "$(notdir $@) (dbg)"
	mkdir -p $(dir $@)
	$(CXX) -MM $(DBG_CXXFLAGS) $< | sed 's#\([-A-Za-z0-9_,]*\)\.o[ :]*#$(patsubst %.d,%.o,$@) $@ : #g' >$@

$(REL_OBJDIR)/%.d: %.cpp
	@echo "$(notdir $@) (rel)"
	mkdir -p $(dir $@)
	$(CXX) -MM $(REL_CXXFLAGS) $< | sed 's#\([-A-Za-z0-9_,]*\)\.o[ :]*#$(patsubst %.d,%.o,$@) $@ : #g' >$@

$(DBG_OBJDIR)/%.d: %.c
	@echo "$(notdir $@) (dbg)"
	mkdir -p $(dir $@)
	$(CC) -MM $(DBG_CXXFLAGS) $< | sed 's#\([-A-Za-z0-9_,]*\)\.o[ :]*#$(patsubst %.d,%.o,$@) $@ : #g' >$@

$(REL_OBJDIR)/%.d: %.c
	@echo "$(notdir $@) (rel)"
	mkdir -p $(dir $@)
	$(CC) -MM $(REL_CXXFLAGS) $< | sed 's#\([-A-Za-z0-9_,]*\)\.o[ :]*#$(patsubst %.d,%.o,$@) $@ : #g' >$@

debug: $(DBG_BINDIR)/$(DBG_NAME) debug_post_build

release: $(REL_BINDIR)/$(REL_NAME) release_post_build

all: debug release

# Define post build steps in Makefile.prj.inc if needed.

post_build:

debug_post_build: post_build

release_post_build: post_build

clean:
	@rm -f `find $(DBG_OBJDIR) -name "*.o" -o -name "*.d"`
	@rm -f `find $(REL_OBJDIR) -name "*.o" -o -name "*.d"`
	@rm -f $(DBG_BINDIR)/$(DBG_NAME) $(REL_BINDIR)/$(REL_NAME)

#-----------------------------------------------------------------------------

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(SRC_CPP)),)
-include $(patsubst %.cpp, $(DBG_OBJDIR)/%.d, $(SRC_CPP))
-include $(patsubst %.cpp, $(REL_OBJDIR)/%.d, $(SRC_CPP))
endif
ifneq ($(strip $(SRC_C)),)
-include $(patsubst %.c, $(DBG_OBJDIR)/%.d, $(SRC_C))
-include $(patsubst %.c, $(REL_OBJDIR)/%.d, $(SRC_C))
endif
endif

#-----------------------------------------------------------------------------
