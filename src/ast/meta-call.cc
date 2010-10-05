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
 ** \file ast/meta-call.cc
 ** \brief Implementation of ast::MetaCall.
 */

#include <ast/visitor.hh>
#include <ast/meta-call.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  MetaCall::MetaCall (const loc& location, exps_type* arguments,
                      const rExp& target, unsigned id)
    : LValueArgs (location, arguments),
      target_ (target),
      id_ (id)
  { }

  MetaCall::~MetaCall ()
  {
  }

  void
  MetaCall::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject MetaCall::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string MetaCall::node_type() const
  {
    return "MetaCall";
  }

  void
  MetaCall::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

