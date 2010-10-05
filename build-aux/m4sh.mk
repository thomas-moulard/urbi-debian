##
## m4sh.mk: This file is part of build-aux.
## Copyright (C) 2006-2009, Gostai S.A.S.
##
## This software is provided "as is" without warranty of any kind,
## either expressed or implied, including but not limited to the
## implied warranties of fitness for a particular purpose.
##
## See the LICENSE file for more information.
## For comments, bug reports and feedback: http://www.urbiforge.com
##

# Use autom4te to create our scripts, with additions stored in
# $(top_srcdir)/build-aux.  Extend m4sh_scripts.

m4sh_scripts =

## ------------- ##
## The scripts.  ##
## ------------- ##

edit =									  \
 sed									  \
  -e 's|@PACKAGE_NAME[@]|$(PACKAGE_NAME)|g'				  \
  -e 's|@EXEEXT[@]|$(EXEEXT)|g'                                           \
  -e 's|@PACKAGE_TARNAME[@]|$(PACKAGE_TARNAME)|g'			  \
  -e 's|@PERL[@]|$(PERL)|g'						  \
  -e 's|@SHELL[@]|$(SHELL)|g'						  \
  -e 's|@VERSION[@]|$(VERSION)|g'					  \
  -e 's|@abs_builddir[@]|$(abs_builddir)|g'				  \
  -e 's|@abs_srcdir[@]|$(abs_srcdir)|g'					  \
  -e 's|@abs_top_builddir[@]|$(abs_top_builddir)|g'			  \
  -e 's|@abs_top_srcdir[@]|$(abs_top_srcdir)|g'				  \
  -e 's|@bindir[@]|$(bindir)|g'						  \
  -e 's|@builddir[@]|$(builddir)|g'					  \
  -e 's|@configure_input[@]|Generated from $@.in; do not edit by hand.|g' \
  -e 's|@datadir[@]|$(pkgdatadir)|g'					  \
  -e 's|@host[@]|$(host)|g'						  \
  -e 's|@prefix[@]|$(prefix)|g'						  \
  -e 's|@srcdir[@]|$(srcdir)|g'						  \
  -e 's|@top_builddir[@]|$(top_builddir)|g'				  \
  -e 's|@top_srcdir[@]|$(top_srcdir)|g'

# Scripts written in M4sh.
m4sh_deps =					\
  $(build_aux_dir)/m4sh/base.m4sh		\
  $(build_aux_dir)/m4sh/rst.m4sh		\
  $(build_aux_dir)/m4sh/instrument.m4sh		\
  $(build_aux_dir)/m4sh/children.m4sh		\
  $(build_aux_dir)/m4sh/urbi.m4sh

EXTRA_DIST += $(m4sh_deps)

AUTOM4TE = autom4te
M4SH = $(AUTOM4TE) --language M4sh
%.in: %.m4sh $(m4sh_deps)
	$(MKDIR_P) $(dir $@)
	$(M4SH) $(M4SHFLAGS) $(m4sh_deps) $< -o $@

## All the scripts depend on Makefile so that they are rebuilt when the
## prefix etc. changes.  It took quite a while to have the rule correct,
## don't break it!
## Use chmod -w to prevent people from editing the wrong file by accident.
$(m4sh_scripts): %: %.in Makefile
	rm -f $@ $@.tmp
	$(edit) `test -f ./$@.in || echo $(srcdir)/`$@.in >$@.tmp
	chmod +x $@.tmp
	chmod a-w $@.tmp
	mv $@.tmp $@

CLEANFILES += $(m4sh_scripts:=.in)
