#! /bin/sh

set -e
case $VERBOSE in (x)
  set -x;;
esac

# Basic test script.
# Copyright (C) 2006, 2007, 2008, Gostai S.A.S.
#
# This software is provided "as is" without warranty of any kind,
# either expressed or implied, including but not limited to the
# implied warranties of fitness for a particular purpose.
#
# See the LICENSE file for more information.
# For comments, bug reports and feedback: http://www.urbiforge.com

  # ------------- #
  # Documentation #
  # ------------- #

# This test script provides a simple way to generate test suites using
# GNU Automake. Note that once the test suite is generated and distributed,
# only standard POSIX make is required.
#
# This script will check the output (on stdout and/or stderr) of a program
# against a reference output (if available) whereas Automake's builtin test
# feature only checks the return value (without using DejaGnu).

# ----------- #
# Quick Setup #
# ----------- #

# Better than a long explanation, here is a sample test suite to put in your
# tests/Makefile.am:
#
# check_PROGRAMS = foo bar
# foo_SOURCES = foo.c
# bar_SOURCES = bar.c aux.c
# TESTS = foo.test bar.test
#
# SUFFIXES = .test
# .c.test:
#	$(LN_S) -f $(srcdir)/test.sh $@
			   # ^^^^^^^ where test.sh is this script.
#
# EXTRA_DIST = test.sh
# CLEANFILES = *.my_stdout *.my_stderr *.valgrind.log
# TESTS_ENVIRONMENT = SRCDIR=$(srcdir)

# Alternatively, you can generate the .test files like this:
# $(TESTS): Makefile.am
#	for i in $(TESTS); do $(LN_S) -f $(srcdir)/test.sh $$i; done
		  # where test.sh is this script.  ^^^^^^^
# If all your tests use the script, you don't have to bother with TESTS:
# TESTS = $(check_PROGRAMS:=.test)

# ---------- #
# How to use #
# ---------- #

# Simply run `make check' or better, `make distcheck' :)
# By default, only the return value will be checked -- which is just what
# Automake does by default. If you want to check stderr/stdout, you need to
# create (or generate) reference output files. Eg: foo.stdout, foo.stderr or
# foo.ret (for the return value).
#
# You can do this by hand or use this script to generate the output:
# $ make check GEN=stdout
# will run all your tests and save their output in `test-name.stdout'. The
# next time you run `make check', stdout will be checked against the saved
# (reference) output. Running this again will simply update (overwrite)
# reference files.
#
# You can also use `GEN=stderr' or `GEN=ret'.
# If you encounter any problem, try `make check DEBUG=1'
# You can also `make check VERBOSE=1' in order to get the output on stdout and
# stderr as the tests run. Note that when doing this, stdout will always
# appear first, then stderr. Messages on stdout/stderr will not be interleaved
# as they might originally be. This is because stdout/stderr are buffered.
#
# When you generate reference output files, you might want to version them.
# When you run `make check GEN=something', files are generated in the build
# tree. You might want to move them to the source tree and `svn add' them. You
# might also want to distribute them (using EXTRA_DIST = $(TESTS:.test=.stdout)
# for instance, or list the .stdout/.stderr/.ret manually).
#
# Note: this script requires SRCDIR to be set. This implies that you must set
# the TESTS_ENVIRONMENT variable in your Makefile.am and that you *cannot* run
# ./some.test. If you want to be able to do that (recommended) you must add the
# following to your configure.ac:
# AC_CONFIG_FILES([build-aux/test.sh], [chmod a=rx build-aux/test.sh])
# Warning: this entails that test.sh will be accessible from
#          $(top_builddir)/build-aux.

# ------------- #
# With Valgrind #
# ------------- #

# Simply make sure that the environment variable USE_VALGRIND is not empty,
# eg: make distcheck USE_VALGRIND=1 and valgrind will be used if installed.
# If valgrind is not in your path or has a different name, use the VALGRIND
# environment variable, eg:
#   make distcheck USE_VALGRIND=1 VALGRIND='/path/to/valgrind'
# If you want to pass additional arguments to valgrind, use VALGRIND_ARGS eg:
#   make distcheck USE_VALGRIND=1 VALGRIND_ARGS='--tool=cachegrind'
# Note that valgrind also reads its arguments from VALGRIND_OPTS or
# ~/.valgrindrc or ./.valgrindrc (read valgrind's manual).
# By default valgrind errors are not fatal, however if want them to be fatal
# set FATAL_VALGRIND to a non-empty value.

me=$(basename "$0")

stderr ()
{
  local i
  for i
  do
    echo >&2 "$me: $i"
  done
}

error ()
{
  local exit="$1"
  shift
  stderr "$@"
  exit $exit
}

fatal ()
{
  error 1 "$@"
}

# Return whether the host is running windows or not.
host_windows ()
{
  local host_os='linux-gnu'
  case $host_os:$(uname -s) in
    (cygwin* | mingw* | pw32* | interix* | \
     *:CYGWIN* | *:MINGW* | *:[wW]indows* | *:PW* | *:Interix* | *:UWIN*)
      return 0;;

    (*)
      return 1;;
  esac
}

# new_reference_output_file [ret|stdout|stderr] <ref-file>
new_reference_output_file()
{
  # Simply update existing reference output file.
  if test x"$2" != x; then
    cp -f $bprog.my_$1 $2
  else # Create new reference output file (try first in SRCDIR).
    cp -f $bprog.my_$1 $SRCDIR/$bprog.$1 \
    || cp -f $bprog.my_$1 $bprog.$1 || {
      stderr "cannot generate $bprog.$1"
      script_exit=1
    }
  fi
}

# Program to test
prog=$(echo "$0" | sed 's/\.test$//;y/ /_/')
# Basename of the program to test
bprog=$(basename $prog)

# If we are on Windows we must remove the trailing carriage returns.
remove_trailing_cr=false
exeext=
if host_windows; then
  remove_trailing_cr=true
  exeext=.exe
fi

progexe=$prog$exeext

test -f $progexe ||
  fatal "$progexe: No such file or directory"

test -x $progexe ||
  fatal "$progexe: Not executable."

# We NEED the env var SRCDIR
: ${SRCDIR=../../../../source/sdk-remote/libport/build-aux}
test x"${SRCDIR%@}" != 'x@srcdir' ||
  error 42 '$SRCDIR is empty'

use_valgrind='no'
valgrind_libtool='no' # Are we trying to valgrind a libtool wrapper?
if [ x"$USE_VALGRIND" != x ] || [ x"$WITH_VALGRIND" != x ]; then
  : ${VALGRIND=valgrind}
  if ($VALGRIND --version) >/dev/null 2>/dev/null; then
    valgrind="$VALGRIND"
    valgrind_args="$VALGRIND_ARGS --log-file-exactly=$bprog.valgrind.log"
    use_valgrind='yes'
    # Are we about to run a script?
    if [ x"$(sed '1s/^\(..\).*/\1/;q' $progexe)" = 'x#!' ]; then
      # Libtool often generates a script instead of an executable, let's see if
      # it's the case
      if (grep -i 'generated by ltmain' $progexe) >/dev/null 2>/dev/null; then
	valgrind_libtool='yes'
	sed "/	*exec .*progdir.*program/ s/exec/& $VALGRIND $valgrind_args/" \
	  $progexe >$prog.run_valgrind ||
            fatal "Internal error"
	chmod a+x $prog.run_valgrind
      fi
    fi
  fi
fi

# default values
ref_ret=0;     check_ret='no'
ref_stdout=''; check_stdout='no'
ref_stderr=''; check_stderr='no'

# Find reference output files (if they are provided).
for i in ret stdout stderr; do
  # Search first in build dir, then where the prog is (should be build dir
  # too, but we never know) then finally in the SRCDIR.
  for f in ./$bprog.$i $prog.$i $SRCDIR/$bprog.$i; do
    if [ -f "$f" ]; then
      if [ -r "$f" ]; then
	# Meta-programming in Sh \o/
	eval ref_$i="$f"
	eval check_$i='yes'
	break
      else
	stderr "warning: \"$f\" isn't readable"
      fi
    fi
  done
done

# ------------------------------------------ #
# Run the program to check and save outputs. #
# ------------------------------------------ #
if [ $use_valgrind = yes ] && [ $valgrind_libtool = no ]; then
  $valgrind $valgrind_args -- $progexe >$bprog.my_stdout 2>$bprog.my_stderr
  my_ret=$?
elif [ $use_valgrind = yes ] && [ $valgrind_libtool = yes ]; then
  $prog.run_valgrind >$bprog.my_stdout 2>$bprog.my_stderr
  my_ret=$?
  rm -f $prog.run_valgrind
else
  $progexe >$bprog.my_stdout 2>$bprog.my_stderr
  my_ret=$?
fi

# Return value of this script.
script_exit=0

# Are we trying to generate reference output files?
if test "x$GEN" = xret; then
  echo "$my_ret" >$bprog.my_ret
  new_reference_output_file ret "$ref_var"
  rm -f $bprog.my_ret
fi

if [ $check_ret = 'no' ]; then # Check that the test returned 0 anyway.
  if [ $my_ret -ne 0 ]; then
    script_exit=1
    stderr "bad return value, got $my_ret, expected 0"
  fi
else
  ref_ret_val=$(cat "$ref_ret")
  ref_ret_without_digits=$(echo "$ref_ret_val" | sed 's/[0-9]//g')
  if [ x"$ref_ret_without_digits" != x ]; then
    script_exit=1
    echo "$0: invalid content for $ref_ret, maybe run \`make check GEN=ret'?"
  fi
  if [ $my_ret -ne "$ref_ret_val" ]; then
    script_exit=1
    stderr "bad return value, got $my_ret, expected $ref_ret_val"
  fi
fi

# Check stdout and stderr.
for i in stdout stderr; do
  check_var=check_$i
  check_var=$(eval echo \\\$$check_var)
  ref_var=ref_$i
  ref_var=$(eval echo \\\$$ref_var)

  # Display their output (a bit late...).
  if test x"$VERBOSE" != x; then
    stderr "========== $bprog.my_$i"
    cat >&2 $bprog.my_$i
  fi

  # Are we trying to generate reference output files?
  if test "x$GEN" = x$i; then
    new_reference_output_file $i "$ref_var"
  fi

  if [ $check_var = 'yes' ]; then
    $remove_trailing_cr &&
      perl -e 'while(<>) { s/\r$//; print; }' \
	      $bprog.my_$i > $bprog.nocr.my_$i &&
      mv $bprog.nocr.my_$i $bprog.my_$i
    if cmp -s $bprog.my_$i $ref_var; then
      rm -f $bprog.my_$i # Good output, remove temporary file.
    else
      script_exit=1
      stderr "wrong output on $i"
      diff -u $ref_var $bprog.my_$i
    fi
  else
    rm -f $bprog.my_$i # No reference output => remove temporary file.
  fi
done

if [ $use_valgrind = yes ]; then
  if [ -r $bprog.valgrind.log ]; then :; else
    echo "$0: Cannot find/read valgrind's output." >&2
    script_exit=1
  fi

  if (grep -i error $bprog.valgrind.log) >/dev/null 2>/dev/null; then
    stderr "Valgrind reported errors:"
    cat $bprog.valgrind.log >&2
    test x"$FATAL_VALGRIND" = x || script_exit=1
  else
    rm -f $bprog.valgrind.log
  fi
fi

exit $script_exit
