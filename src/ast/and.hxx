/*
 * Copyright (C) 2007-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

// Generated, do not edit by hand.

/**
 ** \file ast/and.hxx
 ** \brief Inline methods of ast::And.
 */

#ifndef AST_AND_HXX
# define AST_AND_HXX

# include <ast/and.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  And::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Composite::serialize(ser);
  }

  template <typename T>
  And::And(libport::serialize::ISerializer<T>& ser)
    : Composite(ser)
  {
    (void)ser;
  }
#endif


} // namespace ast

#endif // !AST_AND_HXX
