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
 ** \file ast/this.cc
 ** \brief Implementation of ast::This.
 */

#include <ast/visitor.hh>
#include <ast/this.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  This::This (const loc& location)
    : Exp (location)
  { }

  This::~This ()
  {
  }

  void
  This::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject This::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string This::node_type() const
  {
    return "This";
  }

  void
  This::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

