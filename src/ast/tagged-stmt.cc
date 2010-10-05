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
 ** \file ast/tagged-stmt.cc
 ** \brief Implementation of ast::TaggedStmt.
 */

#include <ast/visitor.hh>
#include <ast/tagged-stmt.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  TaggedStmt::TaggedStmt (const loc& location, const rExp& tag,
                          const rExp& exp)
    : Exp (location),
      tag_ (tag),
      exp_ (exp)
  { }

  TaggedStmt::~TaggedStmt ()
  {
  }

  void
  TaggedStmt::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject TaggedStmt::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string TaggedStmt::node_type() const
  {
    return "TaggedStmt";
  }

  void
  TaggedStmt::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

