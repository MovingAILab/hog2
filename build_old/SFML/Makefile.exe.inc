#-----------------------------------------------------------------------------
# GNU Makefile: part for executables
#
# $Id: Makefile.exe.inc,v 1.1 2006/10/20 18:16:56 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile.exe.inc,v $
#-----------------------------------------------------------------------------

$(DBG_BINDIR)/$(DBG_NAME): $(DBG_OBJS) $(PROJ_DBG_DEP)
	@echo "Linking $@"
	mkdir -p $(dir $@)
	$(LN) -o $@ $(DBG_OBJS) $(PROJ_DBG_LIB) $(DBG_LNFLAGS)

$(REL_BINDIR)/$(REL_NAME): $(REL_OBJS) $(PROJ_REL_DEP)
	@echo "Linking $@"
	mkdir -p $(REL_BINDIR)
	$(LN) -o $@ $(REL_OBJS) $(PROJ_REL_LIB) $(REL_LNFLAGS)
