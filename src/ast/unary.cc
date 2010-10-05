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
 ** \file ast/unary.cc
 ** \brief Implementation of ast::Unary.
 */

#include <ast/visitor.hh>
#include <ast/unary.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Unary::Unary (const loc& location, const rLValue& exp)
    : Exp (location),
      exp_ (exp)
  { }

  Unary::~Unary ()
  {
  }


} // namespace ast

