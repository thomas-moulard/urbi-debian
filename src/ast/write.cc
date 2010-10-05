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
 ** \file ast/write.cc
 ** \brief Implementation of ast::Write.
 */

#include <ast/visitor.hh>
#include <ast/write.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Write::Write (const loc& location, const rLValue& what,
                const rExp& value)
    : Exp (location),
      what_ (what),
      value_ (value)
  { }

  Write::~Write ()
  {
  }


} // namespace ast

