# Makefile for Independent JPEG Group's software
CLEANFILES =
EXTRA_DIST =

# Work around libtool issues.
build_aux_dir = $(top_srcdir)/build-aux
include build-aux/fix-libtool-la.mk

# That's where aclocal extensions are.
ACLOCAL_AMFLAGS = -I build-aux

# source files: JPEG library proper
if INSTALL_LIBRARY
lib_LTLIBRARIES = lib@LIBJPEG_NAME@@LIBSFX@.la
else
noinst_LTLIBRARIES = lib@LIBJPEG_NAME@@LIBSFX@.la
endif

lib@LIBJPEG_NAME@@LIBSFX@_la_SOURCES =					\
  jcapimin.c jcapistd.c jccoefct.c jccolor.c jcdctmgr.c jchuff.c	\
  jcinit.c jcmainct.c jcmarker.c jcmaster.c jcomapi.c jcparam.c		\
  jcphuff.c jcprepct.c jcsample.c jctrans.c jdapimin.c jdapistd.c	\
  jdatadst.c jdatasrc.c jdcoefct.c jdcolor.c jddctmgr.c jdhuff.c	\
  jdinput.c jdmainct.c jdmarker.c jdmaster.c jdmerge.c jdphuff.c	\
  jdpostct.c jdsample.c jdtrans.c jerror.c jfdctflt.c jfdctfst.c	\
  jfdctint.c jidctflt.c jidctfst.c jidctint.c jidctred.c jquant1.c	\
  jquant2.c jutils.c jmemmgr.c
lib@LIBJPEG_NAME@@LIBSFX@_la_SOURCES +=					\
  jchuff.h jdhuff.h jdct.h jerror.h jinclude.h jmemsys.h jmorecfg.h	\
  jpegint.h jpeglib.h jversion.h cdjpeg.h cderror.h transupp.h
# Export the shared library symbols.
lib@LIBJPEG_NAME@@LIBSFX@_la_CPPFLAGS = -DBUILDING_JPEG
lib@LIBJPEG_NAME@@LIBSFX@_la_LDFLAGS = -no-undefined -avoid-version
lib@LIBJPEG_NAME@@LIBSFX@_la_LIBADD =			\
  $(SYSDEPMEM)
lib@LIBJPEG_NAME@@LIBSFX@_la_DEPENDENCIES =		\
  $(SYSDEPMEM)

# memmgr back ends: compile only one of these into a working library
SYSDEPSOURCES = jmemansi.c jmemname.c jmemnobs.c jmemdos.c jmemmac.c
EXTRA_lib@LIBJPEG_NAME@@LIBSFX@_la_SOURCES = $(SYSDEPSOURCES)

# source files: cjpeg/djpeg/jpegtran applications, also rdjpgcom/wrjpgcom
APPSOURCES = cjpeg.c djpeg.c jpegtran.c rdjpgcom.c wrjpgcom.c cdjpeg.c	\
  rdcolmap.c rdswitch.c transupp.c rdppm.c wrppm.c rdgif.c wrgif.c	\
  rdtarga.c wrtarga.c rdbmp.c wrbmp.c rdrle.c wrrle.c
SOURCES = $(LIBSOURCES) $(SYSDEPSOURCES) $(APPSOURCES)

# files included by source files
dist_include_HEADERS = jpeglib.h jmorecfg.h jerror.h
nodist_include_HEADERS = jconfig.h
BUILT_SOURCES = $(nodist_include_HEADERS)

# jconfig.h.  No need for stamps: Automake already uses one for config.h.
jconfig.h: config.h
	@rm -f $@
	cd $(top_builddir) && $(SHELL) ./config.status jconfig.h
	@touch $@

CLEANFILES += _configs.sed jconfig.h
# Files created by VC++.
CLEANFILES += vc80.pdb *.exe.manifest *.exe.intermediate.manifest


# documentation, test, and support files
DOCS = README install.doc usage.doc			\
  wizard.doc example.c libjpeg.doc structure.doc	\
  coderules.doc filelist.doc change.log

# Makefile templates.
EXTRA_DIST +=								  \
  templates/makefile.cfg  templates/makefile.ansi templates/makefile.unix \
  templates/makefile.bcc  templates/makefile.mc6  templates/makefile.dj	  \
  templates/makefile.wat  templates/makefile.vc   templates/makelib.ds	  \
  templates/makeapps.ds   templates/makeproj.mac  templates/makcjpeg.st	  \
  templates/makdjpeg.st   templates/makljpeg.st   templates/maktjpeg.st	  \
  templates/makefile.manx templates/makefile.sas  templates/makefile.mms  \
  templates/makefile.vms  templates/makvms.opt

# Configuration file templates.
EXTRA_DIST +=								\
  templates/jconfig.bcc templates/jconfig.mc6  templates/jconfig.dj	\
  templates/jconfig.wat templates/jconfig.vc   templates/jconfig.mac	\
  templates/jconfig.st  templates/jconfig.manx templates/jconfig.sas	\
  templates/jconfig.vms templates/jconfig.doc

OTHERFILES = ckconfig.c jmemdosa.asm

TESTFILES = testorig.jpg testimg.ppm testimg.bmp testimg.jpg testprog.jpg \
 testimgp.jpg

EXTRA_DIST += $(DOCS) $(SOURCES) $(OTHERFILES) $(TESTFILES)


## ------------------------------------- ##
## Programs, man pages, and test suite.  ##
## ------------------------------------- ##

AM_LDADD = lib@LIBJPEG_NAME@@LIBSFX@.la
LDADD = $(AM_LDADD)
jpeg_programs = cjpeg djpeg jpegtran rdjpgcom wrjpgcom
man_mans = cjpeg.1 djpeg.1 jpegtran.1 rdjpgcom.1 wrjpgcom.1

cjpeg_SOURCES = cjpeg.c rdppm.c rdgif.c rdtarga.c rdrle.c rdbmp.c rdswitch.c \
  cdjpeg.c
djpeg_SOURCES = djpeg.c wrppm.c wrgif.c wrtarga.c wrrle.c wrbmp.c rdcolmap.c \
  cdjpeg.c
jpegtran_SOURCES = jpegtran.c rdswitch.c cdjpeg.c transupp.c


if PROGRAMS
bin_PROGRAMS = $(jpeg_programs)
dist_man1_MANS = $(man_mans)

## Don't run these tests, since they depend on features explicitly
## disabled in configure.ac.  FIXME: define some AM_CONDITIONAL to run
## these tests when applicable.
##
## check-local: check-local-cjpeg check-local-djpeg
check-local: check-local-jpegtran

check-local-cjpeg: cjpeg
	./cjpeg -dct int -outfile testout.jpg $(srcdir)/testimg.ppm
	cmp $(srcdir)/testimg.jpg testout.jpg
	./cjpeg -dct int -progressive -opt -outfile testoutp.jpg $(srcdir)/testimg.ppm
	cmp $(srcdir)/testimgp.jpg testoutp.jpg

check-local-djpeg: djpeg
	./djpeg -dct int -ppm -outfile testout.ppm $(srcdir)/testorig.jpg
	cmp $(srcdir)/testimg.ppm testout.ppm
	./djpeg -dct int -bmp -colors 256 -outfile testout.bmp $(srcdir)/testorig.jpg
	cmp $(srcdir)/testimg.bmp testout.bmp
	./djpeg -dct int -ppm -outfile testoutp.ppm $(srcdir)/testprog.jpg
	cmp $(srcdir)/testimg.ppm testoutp.ppm

check-local-jpegtran: jpegtran
	./jpegtran -outfile testoutt.jpg $(srcdir)/testprog.jpg
	cmp $(srcdir)/testorig.jpg testoutt.jpg
CLEANFILES += testout*
else
EXTRA_PROGRAMS = $(jpeg_programs)
# dist_noinst_MANS does not work with Automake 1.11.1 (and before).
EXTRA_DIST += $(man_mans)
endif
