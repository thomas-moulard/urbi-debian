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
 ** \file ast/class.cc
 ** \brief Implementation of ast::Class.
 */

#include <ast/visitor.hh>
#include <ast/class.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Class::Class (const loc& location, const rLValue& what,
                exps_type* protos, const rExp& content)
    : Exp (location),
      what_ (what),
      protos_ (protos),
      content_ (content)
  { }

  Class::~Class ()
  {
    delete protos_;
  }

  void
  Class::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Class::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Class::node_type() const
  {
    return "Class";
  }

  void
  Class::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

