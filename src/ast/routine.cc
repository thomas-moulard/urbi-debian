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
 ** \file ast/routine.cc
 ** \brief Implementation of ast::Routine.
 */

#include <ast/visitor.hh>
#include <ast/routine.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Routine::Routine (const loc& location, bool closure,
                    local_declarations_type* formals, rScope body)
    : Exp (location),
      closure_ (closure),
      formals_ (formals),
      body_ (body),
      local_variables_ (0),
      captured_variables_ (new local_declarations_type()),
      local_size_ (0),
      uses_call_ (false)
  { }

  Routine::~Routine ()
  {
    delete formals_;
    delete local_variables_;
    delete captured_variables_;
  }

  void
  Routine::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Routine::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Routine::node_type() const
  {
    return "Routine";
  }

  void
  Routine::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

