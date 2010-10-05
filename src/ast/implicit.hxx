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
 ** \file ast/implicit.hxx
 ** \brief Inline methods of ast::Implicit.
 */

#ifndef AST_IMPLICIT_HXX
# define AST_IMPLICIT_HXX

# include <ast/implicit.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Implicit::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
  }

  template <typename T>
  Implicit::Implicit(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
  }
#endif


} // namespace ast

#endif // !AST_IMPLICIT_HXX
