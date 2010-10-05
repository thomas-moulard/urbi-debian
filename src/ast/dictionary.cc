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
 ** \file ast/dictionary.cc
 ** \brief Implementation of ast::Dictionary.
 */

#include <ast/visitor.hh>
#include <ast/dictionary.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Dictionary::Dictionary (const loc& location, const rExp& base,
                          const modifiers_type& value)
    : Exp (location),
      base_ (base),
      value_ (value)
  { }

  Dictionary::~Dictionary ()
  {
  }

  void
  Dictionary::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Dictionary::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Dictionary::node_type() const
  {
    return "Dictionary";
  }

  void
  Dictionary::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

