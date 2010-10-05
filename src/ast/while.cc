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
 ** \file ast/while.cc
 ** \brief Implementation of ast::While.
 */

#include <ast/visitor.hh>
#include <ast/while.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  While::While (const loc& location, const flavor_type& flavor,
                const rExp& test, const rScope& body)
    : Flavored (location, flavor),
      test_ (test),
      body_ (body)
  { }

  While::~While ()
  {
  }

  void
  While::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject While::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string While::node_type() const
  {
    return "While";
  }

  void
  While::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

