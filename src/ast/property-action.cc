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
 ** \file ast/property-action.cc
 ** \brief Implementation of ast::PropertyAction.
 */

#include <ast/visitor.hh>
#include <ast/property-action.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  PropertyAction::PropertyAction (const loc& location, const rExp& owner,
                                  const libport::Symbol& name)
    : LValue (location),
      owner_ (owner),
      name_ (name)
  { }

  PropertyAction::~PropertyAction ()
  {
  }


} // namespace ast

