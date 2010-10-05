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
 ** \file ast/pipe.cc
 ** \brief Implementation of ast::Pipe.
 */

#include <ast/visitor.hh>
#include <ast/pipe.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Pipe::Pipe (const loc& location, const exps_type& children)
    : Composite (location, children)
  { }

  Pipe::~Pipe ()
  {
  }

  void
  Pipe::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Pipe::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Pipe::node_type() const
  {
    return "Pipe";
  }

  void
  Pipe::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

