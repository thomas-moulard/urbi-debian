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
 ** \file ast/return.cc
 ** \brief Implementation of ast::Return.
 */

#include <ast/visitor.hh>
#include <ast/return.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Return::Return (const loc& location, rExp value)
    : Exp (location),
      value_ (value)
  { }

  Return::~Return ()
  {
  }

  void
  Return::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Return::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Return::node_type() const
  {
    return "Return";
  }

  void
  Return::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

