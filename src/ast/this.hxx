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
 ** \file ast/this.hxx
 ** \brief Inline methods of ast::This.
 */

#ifndef AST_THIS_HXX
# define AST_THIS_HXX

# include <ast/this.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  This::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
  }

  template <typename T>
  This::This(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
  }
#endif


} // namespace ast

#endif // !AST_THIS_HXX
