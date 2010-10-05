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
 ** \file ast/try.cc
 ** \brief Implementation of ast::Try.
 */

#include <ast/visitor.hh>
#include <ast/try.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Try::Try (const loc& location, const rScope& body,
            const catches_type& handlers, rScope elseclause)
    : Exp (location),
      body_ (body),
      handlers_ (handlers),
      elseclause_ (elseclause)
  { }

  Try::~Try ()
  {
  }

  void
  Try::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Try::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Try::node_type() const
  {
    return "Try";
  }

  void
  Try::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

