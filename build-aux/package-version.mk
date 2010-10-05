## Create the .version file.
EXTRA_DIST +=					\
  build-aux/bin/git-version-gen

# .version, the cache file.
VERSION_FILE = $(top_srcdir)/.version

# Ship it, so that when we are detached from a repository, we still
# have it available.  Besides, since "dist" depends on "EXTRA_DIST",
# it ensures that each time we run "make dist", ".version" is updated,
# which is what we want.  Having "all" depend on it would recreate it
# too often.
EXTRA_DIST += $(VERSION_FILE) $(VERSION_FILE).stamp

# all: $(VERSION_FILE) $(VERSION_FILE).stamp

GIT_VERSION_GEN = $(build_aux_dir)/bin/git-version-gen

# Update $(VERSION_FILE).stamp.
#
# We have a nasty problem here: it seems that GNU Make does not behave
# the same way on all hosts (especially GNU/Linux vs. OSX).  As a rule
# of thumb, files once qualified with $(top_srcdir) should always be.
# But in my case (Akim), VPATH is ../.., and it turns out I also have
# a package in ../../$(top_srcdir), so it actually looks in
# ../../../..  and finds a .version.stamp there :( I don't know what
# to do.
#
# The BF seems to prefer the solution below.
$(VERSION_FILE).stamp: $(GIT_VERSION_GEN)
	@rm -f $@.tmp
	@$(mkdir_p) $(dir $@)
	@touch $@.tmp
	if test ! -f $(VERSION_FILE)			\
	   || test -n "$$BUILDBOT_ROOT"; then		\
	  $(GIT_VERSION_GEN)				\
		--srcdir=$(top_srcdir)			\
		--cache=$(VERSION_FILE);		\
	fi
	@mv -f $@.tmp $@

$(VERSION_FILE): $(VERSION_FILE).stamp
	@if test -f $(VERSION_FILE); then :; else	\
	  rm -f $(VERSION_FILE).stamp;			\
	  $(MAKE) $(AM_MAKEFLAGS) $<;			\
	fi

# Force the update of the file if its value differs.
.PHONY: update-version
update-version:
	rm -f $(VERSION_FILE).stamp

.PHONY: debug debug-version
debug: debug-version
debug-version:
	@echo "\$$(VERSION) = $(VERSION)"
