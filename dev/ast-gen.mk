## Before loading this file, define ast_basedir.
##
## The base directory to prepend to all the file names generated in the
## generated Makefile snippet.  If this ast-nodes.mk is to be included
## by ast/Makefile.am, then define to empty.  If it is included by its
## parent Makefile.am, define to "ast/".
##
## Likewise, define ast_nodes to the list of all C++ files containing
## the definitions of the AST nodes to be generated.
##
## Using $(ast_srcdir)/foo or $(ast_basedir)foo depends on how the file
## is used.  Makes, including GNU Make although it is better at this
## than the others, do bad things when a file is sometimes referred to
## as "ast/foo", and sometimes "$(srcdir)/ast/foo", even if $(srcdir)
## is ".".  So stick to a single naming scheme.
##
## Below, we always avoid using $(srcdir) when we can.  We need it on
## the *.stamp targets, otherwise they are created in the build tree.
## We need it on "ast-nodes.mk" because it is "included" in which case
## Automake always prepends "$(srcdir)/".


## -------------------------- ##
## Using the AST generators.  ##
## -------------------------- ##

## We use GNU Make extensions.
AUTOMAKE_OPTIONS += -Wno-portability

## Generate the stamp file, i.e., run the generator.
$(ast_srcdir)/%.stamp: $(gen_dir)/ast-%-gen $(ast_gen_deps)
	@rm -f $@ $@.tmp
	@touch $@.tmp
	$(gen_dir)/ast-$*-gen $(ast_srcdir) < $(ast_srcdir)/ast.yml
	@mv -f $@.tmp $@

## If a generated file is missing, rerun the generation by removing
## the stamp file.
AST_REGENERATE_IF_NEEDED =				\
  if test ! -f $@ && test ! -f $(srcdir)/$@; then	\
    rm -f $<;						\
    $(MAKE) $(AM_MAKEFLAGS) $<;				\
  fi


## ast-nodes.mk
EXTRA_DIST += $(ast_basedir)nodes-mk.stamp
$(ast_srcdir)/ast-nodes.mk: $(ast_srcdir)/nodes-mk.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## all.hh.
EXTRA_DIST += $(ast_basedir)all.stamp
$(ast_basedir)all.hh: $(ast_srcdir)/all.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## cloner.hh etc.
EXTRA_DIST += $(ast_basedir)cloner.stamp
$(ast_basedir)cloner.hh $(ast_basedir)cloner.cc: $(ast_srcdir)/cloner.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## transformer.hh etc.
EXTRA_DIST += $(ast_basedir)transformer.stamp
$(ast_basedir)transformer.hh $(ast_basedir)transformer.cc: $(ast_srcdir)/transformer.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## fwd.hh.
EXTRA_DIST += $(ast_basedir)fwd.stamp
$(ast_basedir)fwd.hh: $(ast_srcdir)/fwd.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## ignores.
EXTRA_DIST += $(ast_basedir)ignores $(ast_basedir)ignores.stamp
$(ast_srcdir)/ignores.stamp: $(gen_dir)/ast-ignores-gen $(ast_gen_deps)
	@rm -f $@ $@.tmp
	@touch $@.tmp
	$(gen_dir)/ast-ignores-gen $(ast_srcdir) < $(ast_srcdir)/ast.yml
	if test -d $(ast_srcdir)/.svn; then \
          svn propset svn:ignore $(ast_srcdir) -F $(ast_srcdir)/ignores; \
	elif test -f $(ast_srcdir)/.gitignore; then \
	  cp $(ast_srcdir)/ignores $(ast_srcdir)/.gitignore; \
        fi
	@mv -f $@.tmp $@
$(ast_basedir)ignores: $(ast_srcdir)/ignores.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## AST itself.
EXTRA_DIST += $(ast_basedir)nodes.stamp
$(AST_NODES): $(ast_srcdir)/nodes.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## ast.dot
EXTRA_DIST += $(ast_basedir)graph.stamp
$(ast_basedir)ast.dot: $(ast_srcdir)/graph.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## DefaultVisitor
EXTRA_DIST += $(ast_basedir)default-visitor.stamp
$(ast_basedir)default-visitor.hh $(ast_basedir)default-visitor.hxx: $(ast_srcdir)/default-visitor.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## Visitor.
EXTRA_DIST += $(ast_basedir)visitor.stamp
$(ast_basedir)visitor.hh: $(ast_srcdir)/visitor.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## PrettyPrinter.
EXTRA_DIST += $(ast_basedir)pretty-printer.stamp
$(ast_basedir)pretty-printer.hh $(ast_basedir)pretty-printer.hxx $(ast_basedir)pretty-printer.cc: $(ast_srcdir)/pretty-printer.stamp
	@$(AST_REGENERATE_IF_NEEDED)

## Dot
EXTRA_DIST += $(ast_basedir)dot-printer.stamp
$(ast_basedir)dot-printer.hh $(ast_basedir)dot-printer.cc: $(ast_srcdir)/dot-printer.stamp
	@$(AST_REGENERATE_IF_NEEDED)

# Serialize
EXTRA_DIST += $(ast_basedir)serializer.stamp
$(ast_basedir)serializer.hh $(ast_basedir)serializer.cc: $(ast_srcdir)/serializer.stamp
	@$(AST_REGENERATE_IF_NEEDED)

MAINTAINERCLEANFILES += $(BUILT_SOURCES_ast)
