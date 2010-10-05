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
 ** \file ast/dictionary.hxx
 ** \brief Inline methods of ast::Dictionary.
 */

#ifndef AST_DICTIONARY_HXX
# define AST_DICTIONARY_HXX

# include <ast/dictionary.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Dictionary::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<rExp>("base", base_);
    ser.template serialize<modifiers_type>("value", value_);
  }

  template <typename T>
  Dictionary::Dictionary(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    base_ = ser.template unserialize<rExp>("base");
    value_ = ser.template unserialize<modifiers_type>("value");
  }
#endif

  inline const rExp&
  Dictionary::base_get () const
  {
    return base_;
  }
  inline rExp&
  Dictionary::base_get ()
  {
    return base_;
  }

  inline const modifiers_type&
  Dictionary::value_get () const
  {
    return value_;
  }
  inline modifiers_type&
  Dictionary::value_get ()
  {
    return value_;
  }


} // namespace ast

#endif // !AST_DICTIONARY_HXX
