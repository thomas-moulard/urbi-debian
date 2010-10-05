## ----------------------------------------------------------------------------
## Tools for python generators
## ----------------------------------------------------------------------------

import string, re, sys
import os, stat, filecmp, shutil

def warning (msg):
  "Display a warning."
  print >>sys.stderr, "Warning: " + msg

def error (msg):
  "Display an error message and exit."
  print >>sys.stderr, "Error: " + msg
  sys.exit (1)

bols = re.compile("^(?=.)", re.M);
def indent(tab, text):
  "Add tab spaces in front of text."
  return bols.sub (" " * tab, text);

def lazy_overwrite (old, new):
  """Overwrite old with new if different, or nonexistant.
  Remove the write permission on the result to avoid accidental edition
  of generated files."""
  verbose = os.getenv(key='BUILDFARM') is None
  if not os.path.isfile (old):
    if verbose:
      print "> Create: " + old
    shutil.move (new, old)
    if verbose:
      os.system("colordiff -uw /dev/null " + old)
  elif not filecmp.cmp (old, new):
    if verbose:
      print "> Overwrite: " + old
    # Change the file modes to write the file
    file_modes = os.stat (old) [stat.ST_MODE]
    os.chmod (old, file_modes | 0666);
    shutil.move (old, old + "~")
    shutil.move (new, old)
    if verbose:
      os.system("colordiff -uw " + old + "~ " + old)
  else:
    os.remove (new)
  # Prevent generated file modifications
  file_modes = os.stat (old) [stat.ST_MODE]
  os.chmod(old, file_modes & 0555);

def lazy_install (srcdir, name):
   """Install name.tmp as srcdir/name."""
   lazy_overwrite (os.path.join (srcdir, name), name + ".tmp")

## String helpers -------------------------------------------------------------

def define_id (s):
  """Return a conventional macro identifier from a pseudo-file name.
  (ast/FooBar.hh -> AST_FOO_BAR_HH)."""
  return re.sub ("[^A-Z]", "_", file_id(s).upper())

def file_id (s):
  """Return a conventional file name from a pseudo-file name.
  (ast/FooBar.hh -> ast/foo-bar.hh)."""
  s = re.sub("^r(Const)?([A-Z])", "\\2", s) # remove the refcounting mark
  return re.sub ("([a-z])([A-Z])", "\\1-\\2", s).lower ()

# FIXME: Improve this generator
# (see http://en.wikipedia.org/wiki/A_and_an for instance).
# Reported by Nicolas Pierron.
## Return the indefinite article to be put before NOUN.
def indef_article (noun):
  if re.match ("[aeiouAEIOU]", noun):
    return "an"
  else:
    return "a"

## Wrap a function prototype.
## This is simplistic, but enough to process our generated code.
def wrap_proto (fundec, width):
  ## Look for the first parenthesis to get the level of indentation.
  indent = fundec.find ("(")
  pieces = fundec.split(",")
  output = ""
  line = ""
  while pieces:
    if len (pieces) == 1:
      sep = ""
    else:
      sep = ","
    piece = pieces.pop (0)
    if len (line) + len (piece) + len (sep) > width:
      # "Flush" the current line.
      output += line + "\n"
      line = " " * indent + piece + sep
    else:
      line += piece + sep
  output += line
  return output

def banner(ast_params, file, brief):
  '''Given a name and description, return the file's banner, including
  its CPP guard when needed.'''
  res = ast_params['file_prologue'] % { 'file': file, 'brief': brief }
  # Header and inline implementation files want guards.
  if re.match(".*\\.(hh|hxx)", file):
    res += "\n"
    res += "#ifndef " + define_id(file) + "\n"
    res += "# define " + define_id(file) + "\n"
  return res
