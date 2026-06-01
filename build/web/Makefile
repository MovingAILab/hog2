#-----------------------------------------------------------------------------
# Main Makefile for project_hog.
# The sub-makefiles are invoked sequentially
# (see: Peter Miller: Recursive Make Considered Harmful
# http://aegis.sourceforge.net/auug97.pdf)
#
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile,v $
# $Id: Makefile,v 1.5 2006/11/01 23:34:14 nathanst Exp $
#-----------------------------------------------------------------------------

# All projects to build in this order.
PROJECTS = \
  gui/STUB \
  gui/ \
  algorithms \
  environments \
  envutil \
  graph \
  grids \
  graphalgorithms \
  gui/web \
  mapalgorithms \
  utils \
  apps/snakebird \
  apps/arm \
  demos/dynamic-phi \
  papers/DSDWA \
  demos/fastmap \
  demos/pancake \
  demos/suboptimal-map \
  demos/STP \
  demos/witness \
  demos/witness3 \
  demos/transit-routing-maps \
  demos/gridabstraction \
  demos/JPS \
  demos/bb \
  demos/reach \
  demos/dh-place \
  demos/ida \
  demos/bidirnecessary \
  demos/bidir \
  demos/GMX \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/astar-map \
  demos/inconsistency-graph \
  demos/racetrack \
  demos/multi-target \
  demos/ACA \
  papers/DWA \
  papers/FMDH \
  papers/TheWitnessEditor

# Include projects with broken compilation
PROJECTS_ALL = \
  gui/GL \
  absmapalgorithms \
  abstraction \
  abstractionalgorithms \
  algorithms \
  environments \
  envutil \
  generic \
  graph \
  grids \
  graphalgorithms \
  gui \
  mapalgorithms \
  shared \
  simulation \
  utils \
  demos/witness3 \
  demos/pancake \
  apps/arm \
  apps/airplane \
  apps/localsensingsearch \
  apps/cpdb \
  apps/pathsample \
  apps/puzzlesample \
  apps/inconsistency \
  apps/multiagent \
  apps/dynamicsearch \
  apps/directional \
  apps/mapUtils \
  apps/dragonage \
  apps/sfbds \
  apps/prediction \
  apps/rubik \
  apps/fling \
  apps/bidirectional \
  apps/voxel \
  apps/topspin \
  apps/stp \
  apps/roads \
  apps/delta \
  apps/pancake \
  apps/multiagent \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/idastar \
  demos/multi-target \
  papers/ACA \
  papers/DWA \
  papers/TheWitnessEditor

# sequentially to avoid same sub-target in sub-make invoked twice
default:
	$(MAKE) debug
	$(MAKE) release

debug:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (dbg)"; \
             $(MAKE) -C $$dir debug || exit -1; \
           fi; \
        done
	@echo ""

release:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (rel)"; \
             $(MAKE) -C $$dir release || exit -1; \
           fi; \
        done
	@echo ""

clean:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- Removing temporary files for $$dir"; \
             $(MAKE) -C $$dir clean || exit -1; \
           fi; \
         done
	@echo ""

# sequentially to avoid same sub-target in sub-make invoked twice
all: override PROJECTS = $(PROJECTS_ALL)
all:
	$(MAKE) all-debug
	$(MAKE) all-release

all-debug: override PROJECTS = $(PROJECTS_ALL)
all-debug: debug

all-release: override PROJECTS = $(PROJECTS_ALL)
all-release: release

tags:
	@echo -e "Generating tags for Emacs"
	@(ctags -e `find ../.. -name "*.h" -o -name "*.cpp"`)

.PHONY: clean debug default release tags
