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
 ** \file ast/lvalue-args.cc
 ** \brief Implementation of ast::LValueArgs.
 */

#include <ast/visitor.hh>
#include <ast/lvalue-args.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  LValueArgs::LValueArgs (const loc& location, exps_type* arguments)
    : LValue (location),
      arguments_ (arguments)
  { }

  LValueArgs::~LValueArgs ()
  {
    delete arguments_;
  }


} // namespace ast

