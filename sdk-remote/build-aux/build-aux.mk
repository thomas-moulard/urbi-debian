##
## build-aux.mk: This file is part of build-aux.
## Copyright (C) 2006-2010, Gostai S.A.S.
##
## This software is provided "as is" without warranty of any kind,
## either expressed or implied, including but not limited to the
## implied warranties of fitness for a particular purpose.
##
## See the LICENSE file for more information.
## For comments, bug reports and feedback: http://www.urbiforge.com
##

# This file is meant to be included only by the top-level Makefile.am.

EXTRA_DIST +=					\
  $(build_aux_srcdir)/bin/bootstrap		\
  $(build_aux_srcdir)/bin/instrument		\
  $(build_aux_srcdir)/bin/instrument.supp	\
  $(build_aux_srcdir)/bin/print-env		\
  $(build_aux_srcdir)/bin/semaphore-clean.sh

EXTRA_DIST +=					\
    $(call ls_files,@{build-aux}/perl/*)

## ------ ##
## Help.  ##
## ------ ##

.PHONY: help help-first

help: help-first
help-first:
	@echo "Some make targets:"


## -------- ##
## reconf.  ##
## -------- ##

# It is often helpful to rerun configure (well, config.status).
# This is a convenient shorthand.
.PHONY: reconf reconf-help
help: reconf-help
reconf-help:
	@echo "reconf:  rerun configure"

reconf:
	cd $(top_builddir) && ./config.status --recheck
	cd $(top_builddir) && ./config.status
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS)



## -------- ##
## boost.m4 ##
## -------- ##

FETCH_BM4 = wget -O $(build_aux_dir)/boost.m4
boost-m4-up:
	$(FETCH_BM4) 'http://repo.or.cz/w/boost.m4.git?a=blob_plain;f=build-aux/boost.m4;hb=HEAD'
