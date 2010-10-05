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
 ** \file ast/property.cc
 ** \brief Implementation of ast::Property.
 */

#include <ast/visitor.hh>
#include <ast/property.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Property::Property (const loc& location, const rExp& owner,
                      const libport::Symbol& name)
    : PropertyAction (location, owner, name)
  { }

  Property::~Property ()
  {
  }

  void
  Property::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Property::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Property::node_type() const
  {
    return "Property";
  }

  void
  Property::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

