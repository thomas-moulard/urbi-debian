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
 ** \file ast/float.hxx
 ** \brief Inline methods of ast::Float.
 */

#ifndef AST_FLOAT_HXX
# define AST_FLOAT_HXX

# include <ast/float.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Float::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<ufloat>("value", value_);
  }

  template <typename T>
  Float::Float(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    value_ = ser.template unserialize<ufloat>("value");
  }
#endif

  inline const ufloat&
  Float::value_get () const
  {
    return value_;
  }


} // namespace ast

#endif // !AST_FLOAT_HXX
