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
 ** \file ast/property-write.cc
 ** \brief Implementation of ast::PropertyWrite.
 */

#include <ast/visitor.hh>
#include <ast/property-write.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  PropertyWrite::PropertyWrite (const loc& location, const rExp& owner,
                                const libport::Symbol& name,
                                const rExp& value)
    : PropertyAction (location, owner, name),
      value_ (value)
  { }

  PropertyWrite::~PropertyWrite ()
  {
  }

  void
  PropertyWrite::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject PropertyWrite::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string PropertyWrite::node_type() const
  {
    return "PropertyWrite";
  }

  void
  PropertyWrite::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

