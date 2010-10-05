/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef PARSER_PRESCAN_HH
# define PARSER_PRESCAN_HH

# include <libport/cstdlib>

/* The scanner is used for two different purposes: "regular": as a
   regular scanner that feeds its parser, and "prescanner" as a
   pre-scanner that tries to find a complete sentence to feed the
   parser with.

   In the prescanner mode, the scanner looks for the complete command,
   i.e., up to the next terminator (";" or ","), taking care of
   tracking braces, and, of course, occurrences of the terminators in
   the strings or comments etc.

   Flex is not meant to embed two sets of actions in a single scanner,
   so we have to play tricks.  yylex has three optional arguments:
   valp, locp, and up.  In prescanner mode, up = 0, and we are valp
   (which is a pointer) to store a size_t: the length of the full
   sentence.

   There is a number of macros to implement the prescanner mode, and
   to skip the rest of the action in that case: they are all prefixed
   with PRE_*. */

namespace parser
{
  // Prescanner return values.
  enum prescan_type
  {
    /// End of file.
    pre_eof = -1,
    /// No terminator found.
    pre_wants_more = 0,
    /// A complete sentence was found.
    pre_complete = 1,
  };

  /// Scan \a buf, and return the number of bytes to read to get the
  /// next top-level "," or ";" (included).  Return 0 if there is none
  /// (maybe the buffer is not complete enough).
  ///
  /// On a "parse-error" (braces do not match), return the length up to
  /// (including) the invalid brace, so that the parser will raise a
  /// parser error on it.
  /// If a hint on the minimum size required to hit a terminator is available,
  /// return it multiplied by -1.
  long prescan(const char* buf, size_t length);
}

#endif // !PARSER_PRESCAN_HH
