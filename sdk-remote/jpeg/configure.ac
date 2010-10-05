m4_pattern_forbid([^AX_])

AC_INIT([libjpeg], [6b])
AC_CONFIG_AUX_DIR([build-aux])

AM_INIT_AUTOMAKE([1.11.1 foreign build-aux/ansi2knr silent-rules])
AM_SILENT_RULES([yes])

AC_ARG_ENABLE([cflags],
              [AC_HELP_STRING([--with-cflags=FLAGS],
                              [additional compiler flags])],
              [CFLAGS="$CFLAGS $enable_cflags"])

AC_ARG_ENABLE([ldflags],
              [AC_HELP_STRING([--with-ldflags=FLAGS],
                              [additional linker flags])],
              [LDFLAGS="$LDFLAGS $enable_ldflags"])

AC_ARG_ENABLE([library],
	      [AC_HELP_STRING([--disable-library],
			      [just install the programs, not the library])],
	      [], [enable_library=yes])
AM_CONDITIONAL([INSTALL_LIBRARY], [test x"$enable_library" == xyes])

AC_ARG_ENABLE([programs],
	      [AC_HELP_STRING([--disable-programs],
			      [just build the library, not the programs])],
	      [], [enable_programs=yes])
AM_CONDITIONAL([PROGRAMS], [test x"$enable_programs" == xyes])

AC_PROG_CC
AC_CHECK_TYPES([unsigned char, unsigned short])
AM_C_PROTOTYPES

AC_CHECK_HEADERS([stddef.h stdlib.h])
AH_TEMPLATE([NEED_SYS_TYPES_H],
  [Define this if your system does not provide typedef size_t in any of the
   ANSI-standard places (stddef.h, stdlib.h, or stdio.h), but places it in
   <sys/types.h> instead.])

AH_TEMPLATE([NEED_FAR_POINTERS],
  [For 80x86 machines, you need to define NEED_FAR_POINTERS,
   unless you are using a large-data memory model or 80386 flat-memory mode.
   On less brain-damaged CPUs this symbol must not be defined.
   (Defining this symbol causes large data structures to be referenced through
   "far" pointers and to be allocated with a special version of malloc.)])

AH_TEMPLATE([NEED_SHORT_EXTERNAL_NAMES],
  [Define this if your linker needs global names to be unique in less
   than the first 15 characters.])

AH_TEMPLATE([INCOMPLETE_TYPES_BROKEN],
  [Although a real ANSI C compiler can deal perfectly well with pointers to
   unspecified structures (see "incomplete types" in the spec), a few pre-ANSI
   and pseudo-ANSI compilers get confused.  To keep one of these bozos happy,
   define INCOMPLETE_TYPES_BROKEN.  This is not recommended unless you
   actually get "missing structure definition" warnings or errors while
   compiling the JPEG code.])

AH_BOTTOM([
/*
 * The following options affect code selection within the JPEG library,
 * but they don't need to be visible to applications using the library.
 * To minimize application namespace pollution, the symbols won't be
 * defined unless JPEG_INTERNALS has been defined.
 */

#ifdef JPEG_INTERNALS

/* Define this if your compiler implements ">>" on signed values as a logical
 * (unsigned) shift; leave it undefined if ">>" is a signed (arithmetic) shift,
 * which is the normal and rational definition.
 */
# undef RIGHT_SHIFT_IS_UNSIGNED


#endif /* JPEG_INTERNALS */


/*
 * The remaining options do not affect the JPEG library proper,
 * but only the sample applications cjpeg/djpeg (see cjpeg.c, djpeg.c).
 * Other applications can ignore these.
 */

#ifdef JPEG_CJPEG_DJPEG

/* These defines indicate which image (non-JPEG) file formats are allowed. */

/* Define if BMP image file format is supported. */
# undef BMP_SUPPORTED

/* Define if GIF image file format is supported. */
# undef GIF_SUPPORTED

/* Define if PBMPLUS PPM/PGM image file format is supported. */
# undef PPM_SUPPORTED

/* Define if Utah RLE image file format is supported. */
# undef RLE_SUPPORTED

/* Define if Targa image file format is supported. */
# undef TARGA_SUPPORTED



/* Define this if you want to name both input and output files on the command
 * line, rather than using stdout and optionally stdin.  You MUST do this if
 * your system can't cope with binary I/O to stdin/stdout.  See comments at
 * head of cjpeg.c or djpeg.c.
 */
# undef TWO_FILE_COMMANDLINE

/* Define this if your system needs explicit cleanup of temporary files.
 * This is crucial under MS-DOS, where the temporary "files" may be areas
 * of extended memory; on most other systems it's not as important.
 */
# undef NEED_SIGNAL_CATCHER

/* By default, we open image files with fopen(...,"rb") or fopen(...,"wb").
 * This is necessary on systems that distinguish text files from binary files,
 * and is harmless on most systems that don't.  If you have one of the rare
 * systems that complains about the "b" spec, define this symbol.
 */
# undef DONT_USE_B_MODE

/* Define this if you want percent-done progress reports from cjpeg/djpeg.
 */
# undef PROGRESS_REPORT

#endif /* JPEG_CJPEG_DJPEG */

/*
 * Local] [Variables:
 * mode: C
 * End:
 */
])

# Put here the object file name for the correct system-dependent memory
# manager file.  For Unix this is usually jmemnobs.c, but you may want
# to use jmemansi.c or jmemname.c if you have limited swap space.
# Possible values: see SYSDEPSOURCES in Makefile.am.
AC_CANONICAL_HOST
case $host in
  *) SYSDEPMEM='jmemnobs.lo';;
esac
AC_SUBST([SYSDEPMEM])

# The name of the library to build (not including the "lib" prefix).
# Defaults to its original name, but can be changed to another name to
# avoid conflicts.
: ${LIBJPEG_NAME=jpeg}
AC_SUBST([LIBJPEG_NAME])

# An extension to the library.  Can be empty, or, for instance, "-debug".
AC_SUBST([LIBSFX])


AC_DISABLE_STATIC
AC_PROG_LIBTOOL
URBI_LIBTOOL

AC_CONFIG_HEADERS([config.h:config.hin])
AX_PREFIX_CONFIG_H([jconfig.h])
AC_CONFIG_FILES([Makefile])

AC_OUTPUT
