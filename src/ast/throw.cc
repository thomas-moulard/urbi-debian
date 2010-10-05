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
 ** \file ast/throw.cc
 ** \brief Implementation of ast::Throw.
 */

#include <ast/visitor.hh>
#include <ast/throw.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Throw::Throw (const loc& location, rExp value)
    : Exp (location),
      value_ (value)
  { }

  Throw::~Throw ()
  {
  }

  void
  Throw::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Throw::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Throw::node_type() const
  {
    return "Throw";
  }

  void
  Throw::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

