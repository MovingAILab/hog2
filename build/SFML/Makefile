#-----------------------------------------------------------------------------
# Main Makefile for project_hog.
# The sub-makefiles are invoked sequentially
# (see: Peter Miller: Recursive Make Considered Harmful
# http://aegis.sourceforge.net/auug97.pdf)
#
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile,v $
# $Id: Makefile,v 1.5 2006/11/01 23:34:14 nathanst Exp $
#-----------------------------------------------------------------------------
ROOT = ../..
BINDIR = $(ROOT)/bin/release/
prefix = /usr/local/bin

# All projects to build in this order.
PROJECTS = \
  gui/STUB \
  algorithms \
  environments \
  envutil \
  graph \
  grids \
  graphalgorithms \
  gui \
  mapalgorithms \
  utils \
  apps/arm \
  apps/witness \
  apps/snakebird \
  apps/airplane \
  apps/localsensingsearch \
  apps/pathsample \
  apps/puzzlesample \
  apps/mapUtils \
  apps/prediction \
  apps/emotion \
  apps/hexagon \
  apps/3dmaps \
  apps/rubik \
  apps/bidirectional \
  apps/voxel \
  apps/delta \
  apps/fling \
  apps/stp \
  apps/PDB \
  apps/roads \
  apps/canonicalGrids \
  apps/delta \
  demos/ACA \
  demos/BOA \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/idastar \
  demos/fastmap \
  demos/pancake \
  demos/racetrack \
  demos/bidir \
  demos/multiTarget \
  demos/rc \
  demos/astar-map \
  demos/suboptimal-map \
  demos/STP \
  demos/transit-routing-maps \
  demos/gridabstraction \
  demos/JPS \
  demos/bb \
  demos/reach \
  demos/dh-place \
  demos/bidirnecessary \
  demos/GMX \
  demos/inconsistency-graph \
  papers/IBEX \
  papers/DWA \
  papers/BGS \
  papers/DSDWA \
  tests/mnpuzzle \
  tests/graphics \
  papers/FMDH \
#  generic \




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
  demos/ACA \
  demos/BOA \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/idastar \
  demos/fastmap \
  demos/pancake \
  demos/racetrack \
  demos/bidir \
  demos/multiTarget \
  demos/rc \
  demos/astar-map \
  demos/suboptimal-map \
  demos/STP \
  demos/transit-routing-maps \
  demos/gridabstraction \
  demos/JPS \
  demos/bb \
  demos/reach \
  demos/dh-place \
  demos/bidirnecessary \
  demos/GMX \
  demos/inconsistency-graph \
  apps/snakebird \
  papers/DSDWA \
#  papers/IBEX \
#	apps/coprobber

TESTS = \
	tests/mnpuzzle \


# sequentially to avoid same sub-target in sub-make invoked twice
default:
	$(MAKE) debug
	$(MAKE) release
	$(MAKE) tests

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

tests: release
	@for dir in $(TESTS); do \
					if test -f $$dir/Makefile; then \
						echo -e "\n--------------- $$dir (test)"; \
						$(MAKE) -C $$dir tests || exit -1; \
					fi; \
			done
	@echo ""

.PHONY: install
install:
	@for file in $$(ls -I '*.a' $(BINDIR)); do \
           if test -f $(BINDIR)$$file; then \
              echo "Installing $$file"; \
              mkdir -p $(prefix); \
              cp $(BINDIR)$$file $(prefix)/; \
           fi; \
        done
	@echo -e "\nDone\nAll files installed to $(prefix)"


.PHONY: uninstall
uninstall:
	@for file in $$(ls -I '*.a' $(BINDIR)); do \
           if test -f $(BINDIR)$$file; then \
              if test -f $(prefix)/$$file; then \
                 echo "Removing $(prefix)/$$file"; \
                 rm $(prefix)/$$file; \
              fi; \
           fi; \
        done
	@echo -e "\nDone"


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
