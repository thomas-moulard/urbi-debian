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
 ** \file ast/local-write.cc
 ** \brief Implementation of ast::LocalWrite.
 */

#include <ast/visitor.hh>
#include <ast/local-write.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  LocalWrite::LocalWrite (const loc& location, const libport::Symbol& what,
                          const rExp& value)
    : Exp (location),
      what_ (what),
      value_ (value),
      local_index_ (0)
  { }

  LocalWrite::~LocalWrite ()
  {
  }


} // namespace ast

