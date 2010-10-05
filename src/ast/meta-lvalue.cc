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
 ** \file ast/meta-lvalue.cc
 ** \brief Implementation of ast::MetaLValue.
 */

#include <ast/visitor.hh>
#include <ast/meta-lvalue.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  MetaLValue::MetaLValue (const loc& location, exps_type* arguments,
                          unsigned id)
    : LValueArgs (location, arguments),
      id_ (id)
  { }

  MetaLValue::~MetaLValue ()
  {
  }

  void
  MetaLValue::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject MetaLValue::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string MetaLValue::node_type() const
  {
    return "MetaLValue";
  }

  void
  MetaLValue::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

