##
## make/doxygen.mk: This file is part of build-aux.
## Copyright (C) 2010, Gostai S.A.S.
##
## This software is provided "as is" without warranty of any kind,
## either expressed or implied, including but not limited to the
## implied warranties of fitness for a particular purpose.
##
## See the LICENSE file for more information.
## For comments, bug reports and feedback: http://www.urbiforge.com
##

## ----------------------- ##
## Doxygen documentation.  ##
## ----------------------- ##

# Make doxygen silent by default, unless V=1.

# Use tex4ht.
%.htmldir: %.dox
	-rm -rf $@ $@.tmp
	mkdir -p $@.tmp
	{					\
	  cat $<;				\
	  echo "OUTPUT_DIRECTORY = $@.tmp";	\
	  if test 'x$(V)' = x1; then		\
	    echo "QUIET = NO";			\
	  else					\
	    echo "QUIET = YES";			\
	  fi					\
	} |					\
	  $(DOXYGEN) $(AM_DOXYGENFLAGS) -
	mv $@.tmp/html $@
	rm -rf $@.tmp
