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
 ** \file ast/string.cc
 ** \brief Implementation of ast::String.
 */

#include <ast/visitor.hh>
#include <ast/string.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  String::String (const loc& location, const std::string& value)
    : Exp (location),
      value_ (value)
  { }

  String::~String ()
  {
  }

  void
  String::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject String::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string String::node_type() const
  {
    return "String";
  }

  void
  String::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

