/* jconfig.wat --- jconfig.h for Watcom C/C++ on MS-DOS or OS/2. */
/* see jconfig.doc for explanations */

#define LIBJPEG_HAVE_PROTOTYPES
#define LIBJPEG_HAVE_UNSIGNED_CHAR
#define LIBJPEG_HAVE_UNSIGNED_SHORT
/* #define void char */
/* #define const */
#define CHAR_IS_UNSIGNED
#define LIBJPEG_HAVE_STDDEF_H
#define LIBJPEG_HAVE_STDLIB_H
#undef LIBJPEG_NEED_BSD_STRINGS
#undef LIBJPEG_NEED_SYS_TYPES_H
#undef LIBJPEG_NEED_FAR_POINTERS	/* Watcom uses flat 32-bit addressing */
#undef LIBJPEG_NEED_SHORT_EXTERNAL_NAMES
#undef INCOMPLETE_TYPES_BROKEN

#ifdef JPEG_INTERNALS

#undef LIBJPEG_RIGHT_SHIFT_IS_UNSIGNED

#endif /* JPEG_INTERNALS */

#ifdef JPEG_CJPEG_DJPEG

#define LIBJPEG_BMP_SUPPORTED		/* BMP image file format */
#define LIBJPEG_GIF_SUPPORTED		/* GIF image file format */
#define LIBJPEG_PPM_SUPPORTED		/* PBMPLUS PPM/PGM image file format */
#undef LIBJPEG_RLE_SUPPORTED		/* Utah RLE image file format */
#define LIBJPEG_TARGA_SUPPORTED		/* Targa image file format */

#undef LIBJPEG_TWO_FILE_COMMANDLINE	/* optional */
#define USE_SETMODE		/* Needed to make one-file style work in Watcom */
#undef LIBJPEG_NEED_SIGNAL_CATCHER	/* Define this if you use jmemname.c */
#undef LIBJPEG_DONT_USE_B_MODE
#undef LIBJPEG_PROGRESS_REPORT		/* optional */

#endif /* JPEG_CJPEG_DJPEG */