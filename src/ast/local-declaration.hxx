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
 ** \file ast/local-declaration.hxx
 ** \brief Inline methods of ast::LocalDeclaration.
 */

#ifndef AST_LOCAL_DECLARATION_HXX
# define AST_LOCAL_DECLARATION_HXX

# include <ast/local-declaration.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  LocalDeclaration::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    LocalWrite::serialize(ser);
    ser.template serialize<bool>("constant", constant_);
  }

  template <typename T>
  LocalDeclaration::LocalDeclaration(libport::serialize::ISerializer<T>& ser)
    : LocalWrite(ser)
  {
    (void)ser;
    constant_ = ser.template unserialize<bool>("constant");
  }
#endif

  inline const bool&
  LocalDeclaration::constant_get () const
  {
    return constant_;
  }
  inline void
  LocalDeclaration::constant_set (bool constant)
  {
    constant_ = constant;
  }


} // namespace ast

#endif // !AST_LOCAL_DECLARATION_HXX
