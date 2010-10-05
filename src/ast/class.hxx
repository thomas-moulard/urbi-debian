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
 ** \file ast/class.hxx
 ** \brief Inline methods of ast::Class.
 */

#ifndef AST_CLASS_HXX
# define AST_CLASS_HXX

# include <ast/class.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Class::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<rLValue>("what", what_);
    ser.template serialize<exps_type*>("protos", protos_);
    ser.template serialize<rExp>("content", content_);
  }

  template <typename T>
  Class::Class(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    what_ = ser.template unserialize<rLValue>("what");
    protos_ = ser.template unserialize<exps_type*>("protos");
    content_ = ser.template unserialize<rExp>("content");
  }
#endif

  inline const rLValue&
  Class::what_get () const
  {
    return what_;
  }
  inline rLValue&
  Class::what_get ()
  {
    return what_;
  }

  inline const exps_type*
  Class::protos_get () const
  {
    return protos_;
  }
  inline exps_type*
  Class::protos_get ()
  {
    return protos_;
  }

  inline const rExp&
  Class::content_get () const
  {
    return content_;
  }
  inline rExp&
  Class::content_get ()
  {
    return content_;
  }


} // namespace ast

#endif // !AST_CLASS_HXX
