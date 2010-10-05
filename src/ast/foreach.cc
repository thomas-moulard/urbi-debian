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
 ** \file ast/foreach.cc
 ** \brief Implementation of ast::Foreach.
 */

#include <ast/visitor.hh>
#include <ast/foreach.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Foreach::Foreach (const loc& location, const flavor_type& flavor,
                    const rLocalDeclaration& index, const rExp& list,
                    const rScope& body)
    : Flavored (location, flavor),
      index_ (index),
      list_ (list),
      body_ (body)
  { }

  Foreach::~Foreach ()
  {
  }

  void
  Foreach::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Foreach::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Foreach::node_type() const
  {
    return "Foreach";
  }

  void
  Foreach::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

