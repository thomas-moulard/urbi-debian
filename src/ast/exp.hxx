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
 ** \file ast/exp.hxx
 ** \brief Inline methods of ast::Exp.
 */

#ifndef AST_EXP_HXX
# define AST_EXP_HXX

# include <ast/exp.hh>

namespace ast
{

#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Exp::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Ast::serialize(ser);
  }

  template <typename T>
  Exp::Exp(libport::serialize::ISerializer<T>& ser)
    : Ast(ser)
  {
    (void)ser;
  }
#endif


} // namespace ast

#endif // !AST_EXP_HXX
