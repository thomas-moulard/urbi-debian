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
 ** \file ast/local-assignment.cc
 ** \brief Implementation of ast::LocalAssignment.
 */

#include <ast/visitor.hh>
#include <ast/local-assignment.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

unsigned LocalAssignment::local_index_get() const
{
  assert(declaration_);
  return declaration_->local_index_get();
}

  LocalAssignment::LocalAssignment (const loc& location,
                                    const libport::Symbol& what,
                                    const rExp& value, unsigned depth)
    : LocalWrite (location, what, value),
      depth_ (depth),
      declaration_ (0)
  { }

  LocalAssignment::~LocalAssignment ()
  {
  }

  void
  LocalAssignment::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject LocalAssignment::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string LocalAssignment::node_type() const
  {
    return "LocalAssignment";
  }

  void
  LocalAssignment::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

