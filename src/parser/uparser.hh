/*
 * Copyright (C) 2006-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/// \file parser/uparser.hh

#ifndef PARSER_UPARSER_HH
# define PARSER_UPARSER_HH

# include <memory>
# include <string>

# include <parser/fwd.hh>
# include <parser/location.hh>

namespace parser
{

  /// Persistent parsing.
  ///
  /// The main point of this class is to allow parsing of a pseudo
  /// stream while maintaining a stack of locations.  It is actually a
  /// facade around a ParserImpl in order to avoid useless
  /// recompilations.
  class UParser
  {
  public:
    UParser();
    UParser(const UParser&);
    ~UParser();

    /// Enable/disable meta variables support.
    /// Disabled by default.
    void meta(bool b);

    /// Parse the command from a buffer.
    /// \return yyparse's result (0 on success).
    /// If \a loc is defined, use it to parse \a code, otherwise
    /// use the current location and update it.
    parse_result_type parse(const std::string& code,
                            const yy::location* loc = 0);

    /// Parse a file.
    /// \return yyparse's result (0 on success).
    parse_result_type parse_file(const std::string& fn);

  private:
    std::auto_ptr<ParserImpl> pimpl_;
  };

}

#endif // !PARSER_UPARSER_HH
