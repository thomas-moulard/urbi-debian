/* jconfig.st --- jconfig.h for Atari ST/STE/TT using Pure C or Turbo C. */
/* see jconfig.doc for explanations */

#define LIBJPEG_HAVE_PROTOTYPES
#define LIBJPEG_HAVE_UNSIGNED_CHAR
#define LIBJPEG_HAVE_UNSIGNED_SHORT
/* #define void char */
/* #define const */
#undef CHAR_IS_UNSIGNED
#define LIBJPEG_HAVE_STDDEF_H
#define LIBJPEG_HAVE_STDLIB_H
#undef LIBJPEG_NEED_BSD_STRINGS
#undef LIBJPEG_NEED_SYS_TYPES_H
#undef LIBJPEG_NEED_FAR_POINTERS
#undef LIBJPEG_NEED_SHORT_EXTERNAL_NAMES
#define INCOMPLETE_TYPES_BROKEN	/* suppress undefined-structure warnings */

#ifdef JPEG_INTERNALS

#undef LIBJPEG_RIGHT_SHIFT_IS_UNSIGNED

#define ALIGN_TYPE  long	/* apparently double is a weird size? */

#endif /* JPEG_INTERNALS */

#ifdef JPEG_CJPEG_DJPEG

#define LIBJPEG_BMP_SUPPORTED		/* BMP image file format */
#define LIBJPEG_GIF_SUPPORTED		/* GIF image file format */
#define LIBJPEG_PPM_SUPPORTED		/* PBMPLUS PPM/PGM image file format */
#undef LIBJPEG_RLE_SUPPORTED		/* Utah RLE image file format */
#define LIBJPEG_TARGA_SUPPORTED		/* Targa image file format */

#define LIBJPEG_TWO_FILE_COMMANDLINE	/* optional -- undef if you like Unix style */
/* Note: if you undef LIBJPEG_TWO_FILE_COMMANDLINE, you may need to define
 * USE_SETMODE.  Some Atari compilers require it, some do not.
 */
#define LIBJPEG_NEED_SIGNAL_CATCHER	/* needed if you use jmemname.c */
#undef LIBJPEG_DONT_USE_B_MODE
#undef LIBJPEG_PROGRESS_REPORT		/* optional */

#endif /* JPEG_CJPEG_DJPEG */
