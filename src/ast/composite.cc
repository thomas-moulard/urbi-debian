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
 ** \file ast/composite.cc
 ** \brief Implementation of ast::Composite.
 */

#include <ast/visitor.hh>
#include <ast/composite.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Composite::Composite (const loc& location, const exps_type& children)
    : Exp (location),
      children_ (children)
  { }

  Composite::~Composite ()
  {
  }


} // namespace ast

