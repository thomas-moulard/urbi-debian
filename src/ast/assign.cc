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
 ** \file ast/assign.cc
 ** \brief Implementation of ast::Assign.
 */

#include <ast/visitor.hh>
#include <ast/assign.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Assign::Assign (const loc& location, const rExp& what, const rExp& value,
                  modifiers_type* modifiers)
    : Exp (location),
      what_ (what),
      value_ (value),
      modifiers_ (modifiers)
  { }

  Assign::~Assign ()
  {
    delete modifiers_;
  }

  void
  Assign::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Assign::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Assign::node_type() const
  {
    return "Assign";
  }

  void
  Assign::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

