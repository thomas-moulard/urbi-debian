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
 ** \file ast/subscript.cc
 ** \brief Implementation of ast::Subscript.
 */

#include <ast/visitor.hh>
#include <ast/subscript.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Subscript::Subscript (const loc& location, exps_type* arguments,
                        const rExp& target)
    : LValueArgs (location, arguments),
      target_ (target)
  { }

  Subscript::~Subscript ()
  {
  }

  void
  Subscript::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Subscript::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Subscript::node_type() const
  {
    return "Subscript";
  }

  void
  Subscript::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

