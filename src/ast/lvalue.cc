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
 ** \file ast/lvalue.cc
 ** \brief Implementation of ast::LValue.
 */

#include <ast/visitor.hh>
#include <ast/lvalue.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif
# include <ast/call.hh>
# include <ast/print.hh>


namespace ast
{

  rCall LValue::call()
  {
    rCall res = dynamic_cast<Call*>(this);
    passert("Invalid call: " << *this, res);
    return res;
  }

  LValue::LValue (const loc& location)
    : Exp (location)
  { }

  LValue::~LValue ()
  {
  }


} // namespace ast

