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
 ** \file ast/binding.cc
 ** \brief Implementation of ast::Binding.
 */

#include <ast/visitor.hh>
#include <ast/binding.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Binding::Binding (const loc& location, const rLValue& what)
    : LValue (location),
      what_ (what),
      constant_ (false)
  { }

  Binding::~Binding ()
  {
  }

  void
  Binding::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Binding::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Binding::node_type() const
  {
    return "Binding";
  }

  void
  Binding::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

