#
# urbi-libport.m4: This file is part of build-aux.
# Copyright (C) 2006-2010, Gostai S.A.S.
#
# This software is provided "as is" without warranty of any kind,
# either expressed or implied, including but not limited to the
# implied warranties of fitness for a particular purpose.
#
# See the LICENSE file for more information.
# For comments, bug reports and feedback: http://www.urbiforge.com
#


# _URBI_LIBPORT_BOOST
# -------------------
# Check for libport dependencies on Boost.
AC_DEFUN([_URBI_LIBPORT_BOOST],
[# Check for Boost headers
AC_REQUIRE([URBI_BOOST_REQUIRE])dnl
URBI_ISYSTEM([BOOST_CPPFLAGS])
URBI_PRAGMA_GCC_DIAGNOSTIC
])


# _URBI_LIBPORT_SCHED
# -------------------
# Check for what we need to use lib/sched.
AC_DEFUN([_URBI_LIBPORT_SCHED],
[AC_CHECK_HEADERS([sched.h sys/resource.h sys/mman.h \
                   sys/times.h sys/param.h direct.h])
AC_CHECK_FUNCS([sched_setscheduler setpriority mlockall times])
URBI_PTHREAD_SOURCES
])


# _URBI_LIBPORT_SERIALIZE
# -----------------------
# Check whether we use libserialize, and what it needs.
AC_DEFUN([_URBI_LIBPORT_SERIALIZE],
[
# --enable-serialization.
URBI_ARG_ENABLE([enable-serialization],
                [Build with support for serialization], [yes|no], [yes])
AM_CONDITIONAL([ENABLE_SERIALIZATION],
               [test x$enable_serialization = xyes])
if test x$enable_serialization = xyes; then
  AC_DEFINE([ENABLE_SERIALIZATION], [1],
            [Define to 1 if serialization is enabled.])
fi
])


# _URBI_LIBPORT_COMMON
# --------------------
# Code common to the use of an installed or shipped libport.
AC_DEFUN([_URBI_LIBPORT_COMMON],
[AC_REQUIRE([URBI_PTHREAD])dnl
AC_REQUIRE([URBI_FLOAT_CHECK])dnl
AC_REQUIRE([_URBI_LIBPORT_BOOST])dnl
AC_REQUIRE([_URBI_LIBPORT_SCHED])dnl
AC_REQUIRE([_URBI_LIBPORT_SERIALIZE])dnl
AC_CHECK_FUNCS([semget])
])


# URBI_LIBPORT_INSTALLED
# ----------------------
# We use an installed libport, most probably that of the kernel we
# use.  We don't run URBI_UFLOAT since, of course, we use the same
# kind of ufloat as the kernel does.
#
# Cores use an installed libport (in kernelincludedir), *and* install it
# (in sdkincludedir).
AC_DEFUN([URBI_LIBPORT_INSTALLED],
[AC_REQUIRE([_URBI_LIBPORT_COMMON])dnl
AC_AFTER([$0], [URBI_DIRS])dnl

# Check that we can find libport files.
libport_config_hh=$(URBI_RESOLVE_DIR([$kernelincludedir/libport/config.h]))
if test ! -f $libport_config_h; then
  AC_MSG_ERROR([--with-urbi-kernel: cannot find $libport_config_h])
fi
])


# URBI_LIBPORT(DIRECTORY)
# -----------------------
# Invoke the macros needed by a Libport shipped in the DIRECTORY
# (i.e., a non-installed copy in this source tree).  Define
# convenience variables to find its various components.
AC_DEFUN([URBI_LIBPORT],
[AC_REQUIRE([_URBI_LIBPORT_COMMON])dnl
AC_REQUIRE([URBI_UFLOAT])dnl

# Define Make macros to find the various libport componenents.
# LIBPORT_BUILDPREFIX must be empty, or end with a /.
AC_SUBST([LIBPORT_SRCDIR],
         ['m4_if([$1], [], [$(top_srcdir)], [$(top_srcdir)/$1])'])
AC_SUBST([LIBPORT_BUILDPREFIX],
         [m4_if([$1], [], [],              ['$(top_builddir)/$1/'])])
])

## Local Variables:
## mode: autoconf
## End:
