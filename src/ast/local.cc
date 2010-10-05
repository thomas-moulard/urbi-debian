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
 ** \file ast/local.cc
 ** \brief Implementation of ast::Local.
 */

#include <ast/visitor.hh>
#include <ast/local.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

unsigned Local::local_index_get() const
{
  assert(declaration_);
  return declaration_->local_index_get();
}

  Local::Local (const loc& location, const libport::Symbol& name,
                exps_type* arguments, unsigned depth)
    : Exp (location),
      name_ (name),
      arguments_ (arguments),
      depth_ (depth),
      declaration_ (0)
  { }

  Local::~Local ()
  {
    delete arguments_;
  }

  void
  Local::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Local::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Local::node_type() const
  {
    return "Local";
  }

  void
  Local::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

