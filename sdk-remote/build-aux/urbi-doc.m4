#
# urbi-doc.m4: This file is part of build-aux.
# Copyright (C) 2006-2010, Gostai S.A.S.
#
# This software is provided "as is" without warranty of any kind,
# either expressed or implied, including but not limited to the
# implied warranties of fitness for a particular purpose.
#
# See the LICENSE file for more information.
# For comments, bug reports and feedback: http://www.urbiforge.com
#

m4_pattern_forbid([^URBI_])dnl

AC_PREREQ([2.60])

# URBI_ARG_ENABLE_DOC
# -------------------
AC_DEFUN([URBI_ARG_ENABLE_DOC],
[URBI_ARGLIST_ENABLE([enable-doc=FORMATS],
                 [generate the documentation],
                 [all|doxygen|html|no|none|pdf|yes],
                 [pdf],
                 [
      FORMATS: comma-separated list of:
        - all: build them all
        - doxygen: generate the HTML documentation from Doxygen
        - html: generate the HTML documentation from LaTeX
        - no: alias for none
        - none: build none of them
        - pdf: generate the PDF documentation from LaTeX
        - yes: alias for all
])

urbi_enable_doc_format ()
{
  local format
  for format
  do
    case $format in
      (all|yes)  urbi_enable_doc_format doxygen html pdf;;
      (doxygen)  urbi_doc_formats="$urbi_doc_formats $format"
                 ENABLE_DOC_DOXYGEN=true;;
      (html)     urbi_doc_formats="$urbi_doc_formats $format"
                 ENABLE_DOC_HTML=true;;
      (none|no)  urbi_doc_formats=
                 ENABLE_DOC_DOXYGEN=false
                 ENABLE_DOC_HTML=false
                 ENABLE_DOC_PDF=false;;
      (pdf)      urbi_doc_formats="$urbi_doc_formats $format"
                 ENABLE_DOC_PDF=true;;
    esac
  done
}

urbi_enable_doc_format none $(echo $enable_doc | tr ',' ' ')
])


## --------- ##
## Doxygen.  ##
## --------- ##
AC_DEFUN([URBI_DOXYGEN],
[# Even if the user didn't say it at configure time, be ready to use
# DOXYGEN.
AC_SUBST([DOXYGEN], [doxygen])
if $ENABLE_DOC_DOXYGEN; then
  URBI_ARG_PROGS([doxygen], [the Doxygen documentation generation program])
fi

AM_CONDITIONAL([ENABLE_DOC_DOXYGEN], [$ENABLE_DOC_DOXYGEN])
# Backward compatibility, to remove eventually.
AM_CONDITIONAL([ENABLE_DOC], [$ENABLE_DOC_DOXYGEN])
])


# URBI_DOC
# --------
AC_DEFUN([URBI_DOC],
[AC_MSG_CHECKING([for documentation formats])
URBI_ARG_ENABLE_DOC
AC_MSG_RESULT([$urbi_doc_formats])

URBI_DOXYGEN

## ------ ##
## HTML.  ##
## ------ ##
AM_CONDITIONAL([ENABLE_DOC_HTML], [$ENABLE_DOC_HTML])
AC_SUBST([TEX4HT], [tex4ht])
if $ENABLE_DOC_HTML; then
  URBI_REQUIRE_PROGS([TEX4HT],  [tex4ht],  [TeX4HT])
fi

## ----- ##
## PDF.  ##
## ----- ##
AM_CONDITIONAL([ENABLE_DOC_PDF], [$ENABLE_DOC_PDF])

## ------------------------ ##
## LaTeX and dependencies.  ##
## ------------------------ ##
AC_SUBST([CONVERT], [convert])
AC_SUBST([DOT],     [dot])
AC_SUBST([PDFCROP], [pdfcrop])
if $ENABLE_DOC_HTML || $ENABLE_DOC_PDF; then
  URBI_PROG_PDFLATEX_REQUIRED
  URBI_REQUIRE_PROGS([CONVERT], [convert], [ImageMagick])
  URBI_REQUIRE_PROGS([DOT],     [dot],     [GraphViz])
  URBI_REQUIRE_PROGS([PDFCROP], [pdfcrop], [TeXlive])
fi
])


## Local Variables:
## mode: autoconf
## End:
