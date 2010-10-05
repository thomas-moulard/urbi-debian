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
 ** \file ast/match.cc
 ** \brief Implementation of ast::Match.
 */

#include <ast/visitor.hh>
#include <ast/match.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Match::Match (const loc& location, const rExp& pattern, rExp guard)
    : Ast (location),
      pattern_ (pattern),
      guard_ (guard),
      bindings_ (0)
  { }

  Match::~Match ()
  {
  }

  void
  Match::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Match::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Match::node_type() const
  {
    return "Match";
  }

  void
  Match::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

