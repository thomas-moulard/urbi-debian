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
 ** \file ast/at.cc
 ** \brief Implementation of ast::At.
 */

#include <ast/visitor.hh>
#include <ast/at.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  At::At (const loc& location, const flavor_type& flavor,
          const loc& flavor_location, const rExp& cond, const rExp& body,
          rExp onleave, rExp duration)
    : Flavored (location, flavor),
      flavor_location_ (flavor_location),
      cond_ (cond),
      body_ (body),
      onleave_ (onleave),
      duration_ (duration)
  { }

  At::~At ()
  {
  }

  void
  At::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject At::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string At::node_type() const
  {
    return "At";
  }

  void
  At::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

