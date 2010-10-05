/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef PARSER_UTILS_HH
# define PARSER_UTILS_HH

# include <string>

# include <parser/ugrammar.hh>

namespace parser
{

  /// Format an error message.
  /// \param l The location.
  /// \param msg The error message.
  /// \return The formatted error message containing the location.
  std::string message_format
    (const yy::parser::location_type& l, const std::string& msg);

} // namespace parser

#endif // PARSER_UTILS_HH
