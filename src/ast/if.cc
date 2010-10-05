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
 ** \file ast/if.cc
 ** \brief Implementation of ast::If.
 */

#include <ast/visitor.hh>
#include <ast/if.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  If::If (const loc& location, const rExp& test, const rScope& thenclause,
          const rScope& elseclause)
    : Exp (location),
      test_ (test),
      thenclause_ (thenclause),
      elseclause_ (elseclause)
  { }

  If::~If ()
  {
  }

  void
  If::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject If::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string If::node_type() const
  {
    return "If";
  }

  void
  If::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

