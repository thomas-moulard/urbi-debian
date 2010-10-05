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
 ** \file ast/local-declaration.cc
 ** \brief Implementation of ast::LocalDeclaration.
 */

#include <ast/visitor.hh>
#include <ast/local-declaration.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  LocalDeclaration::LocalDeclaration (const loc& location,
                                      const libport::Symbol& what,
                                      const rExp& value)
    : LocalWrite (location, what, value),
      constant_ (false)
  { }

  LocalDeclaration::~LocalDeclaration ()
  {
  }

  void
  LocalDeclaration::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject LocalDeclaration::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string LocalDeclaration::node_type() const
  {
    return "LocalDeclaration";
  }

  void
  LocalDeclaration::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

