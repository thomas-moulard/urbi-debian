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
 ** \file ast/assign.hxx
 ** \brief Inline methods of ast::Assign.
 */

#ifndef AST_ASSIGN_HXX
# define AST_ASSIGN_HXX

# include <ast/assign.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Assign::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<rExp>("what", what_);
    ser.template serialize<rExp>("value", value_);
    ser.template serialize<modifiers_type*>("modifiers", modifiers_);
  }

  template <typename T>
  Assign::Assign(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    what_ = ser.template unserialize<rExp>("what");
    value_ = ser.template unserialize<rExp>("value");
    modifiers_ = ser.template unserialize<modifiers_type*>("modifiers");
  }
#endif

  inline const rExp&
  Assign::what_get () const
  {
    return what_;
  }
  inline rExp&
  Assign::what_get ()
  {
    return what_;
  }

  inline const rExp&
  Assign::value_get () const
  {
    return value_;
  }
  inline rExp&
  Assign::value_get ()
  {
    return value_;
  }

  inline const modifiers_type*
  Assign::modifiers_get () const
  {
    return modifiers_;
  }
  inline modifiers_type*
  Assign::modifiers_get ()
  {
    return modifiers_;
  }
  inline void
  Assign::modifiers_set (modifiers_type* modifiers)
  {
    delete modifiers_;
    modifiers_ = modifiers;
  }


} // namespace ast

#endif // !AST_ASSIGN_HXX
