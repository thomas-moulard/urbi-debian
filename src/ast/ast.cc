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
 ** \file ast/ast.cc
 ** \brief Implementation of ast::Ast.
 */

#include <ast/visitor.hh>
#include <ast/ast.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

    void Ast::original_set(const rConstAst& original)
    {
      // Try to keep the very first original node if it exists.
      original_ =
       (original && original->original_) ?
       original->original_ : original;
    }

  Ast::Ast (const loc& location)
    : location_ (location),
      original_ (0)
  { }

  Ast::~Ast ()
  {
  }


} // namespace ast

