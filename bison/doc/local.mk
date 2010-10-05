## Copyright (C) 2001, 2002, 2003, 2005, 2006, 2007, 2008, 2009, 2010
## Free Software Foundation, Inc.

## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

AM_MAKEINFOFLAGS = --no-split
info_TEXINFOS = doc/bison.texinfo
doc_bison_TEXINFOS =				\
  $(CROSS_OPTIONS_TEXI)				\
  doc/fdl.texi					\
  doc/gpl-3.0.texi

CLEANFILES = doc/bison.fns
CLEANDIRS = doc/*.t2d
clean-local:
	rm -rf $(CLEANDIRS)

MOSTLYCLEANFILES += $(top_srcdir)/doc/*.t

CROSS_OPTIONS_PL = $(top_srcdir)/build-aux/cross-options.pl
CROSS_OPTIONS_TEXI = $(top_srcdir)/doc/cross-options.texi
$(CROSS_OPTIONS_TEXI): doc/bison.help $(CROSS_OPTIONS_PL)
# Create $@~ which is the previous contents.  Don't use `mv' here so
# that even if we are interrupted, the file is still available for
# diff in the next run.  Note that $@ might not exist yet.
	{ test ! -f $@ || cat $@; } >$@~
	test ! -f $@.tmp || rm -f $@.tmp
	src/bison$(EXEEXT) --help |					 \
	  perl $(CROSS_OPTIONS_PL) $(top_srcdir)/src/scan-gram.l >$@.tmp
	diff -u $@~ $@.tmp || true
	mv $@.tmp $@
MAINTAINERCLEANFILES = $(CROSS_OPTIONS_TEXI)

## ---------- ##
## Ref card.  ##
## ---------- ##

EXTRA_DIST += doc/refcard.tex
CLEANFILES += doc/refcard.dvi doc/refcard.log doc/refcard.ps

doc/refcard.dvi: doc/refcard.tex
	cd doc && tex refcard.tex

doc/refcard.ps: doc/refcard.dvi


## ---------------- ##
## doc/bison.help.  ##
## ---------------- ##

# Some of our targets (cross-option.texi, bison.1) use "bison --help".
# Since we want to ship the generated file to avoid additional
# requirements over the user environment, we used not depend on
# src/bison itself, but on src/getargs.c and other files.  Yet, we
# need "bison --help" to work to make help2man happy, so we used to
# include "make src/bison" in the commands.  Then we may have a
# problem with concurrent builds, since one make might be aiming one
# of its jobs at compiling src/bison, and another job at generating
# the man page.  If the latter is faster than the former, then we have
# two makes that concurrently try to compile src/bison.  Doomed to
# failure.
#
# As a simple scheme to get our way out, make a stamp file,
# bison.help, which contains --version then --help.  This file can
# depend on bison, which ensures its correctness.  But update it
# *only* if needed (content changes).  This way, we avoid useless
# compilations of cross-option.texi and bison.1.  At the cost of
# repeated builds of bison.help.

EXTRA_DIST += $(top_srcdir)/doc/bison.help
MAINTAINERCLEANFILES += $(top_srcdir)/doc/bison.help
$(top_srcdir)/doc/bison.help: src/bison$(EXEEXT)
	$< --version >doc/bison.help.t
	$< --help   >>doc/bison.help.t
	$(top_srcdir)/build-aux/move-if-change doc/bison.help.t $@


## ----------- ##
## Man Pages.  ##
## ----------- ##

dist_man_MANS = $(top_srcdir)/doc/bison.1

EXTRA_DIST += $(dist_man_MANS:.1=.x)
MAINTAINERCLEANFILES += $(dist_man_MANS)

# Differences to ignore when comparing the man page (the date).
remove_time_stamp = \
  sed 's/^\(\.TH[^"]*"[^"]*"[^"]*\)"[^"]*"/\1/'

# Depend on configure to get version number changes.
$(top_srcdir)/doc/bison.1: doc/bison.help doc/bison.x $(top_srcdir)/configure
	@echo "Updating man page $@"
	$(HELP2MAN)					\
	    --include=$(top_srcdir)/doc/bison.x		\
	    --output=$@.t src/bison$(EXEEXT)
	if $(remove_time_stamp) $@ >$@a.t 2>/dev/null &&		 \
	   $(remove_time_stamp) $@.t | cmp $@a.t - >/dev/null 2>&1; then \
	  touch $@;							 \
	else								 \
	  mv $@.t $@;							 \
	fi
	rm -f $@*.t

nodist_man_MANS = doc/yacc.1

## -------------- ##
## Doxygenation.  ##
## -------------- ##

DOXYGEN = doxygen

.PHONY: doc html

doc: html

html-local: doc/Doxyfile
	cd doc && $(DOXYGEN)

edit = sed -e 's,@PACKAGE_NAME\@,$(PACKAGE_NAME),g' \
	   -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' \
	   -e 's,@top_builddir\@,$(top_builddir),g' \
	   -e 's,@top_srcdir\@,$(top_srcdir),g'

EXTRA_DIST += doc/Doxyfile.in
CLEANFILES += doc/Doxyfile
# Sed is used to generate Doxyfile from Doxyfile.in instead of
# configure, because the former is way faster than the latter.
doc/Doxyfile: $(top_srcdir)/doc/Doxyfile.in
	$(edit) $(top_srcdir)/doc/Doxyfile.in >doc/Doxyfile

CLEANDIRS += html latex
