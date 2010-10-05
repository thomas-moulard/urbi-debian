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
 ** \file ast/finally.cc
 ** \brief Implementation of ast::Finally.
 */

#include <ast/visitor.hh>
#include <ast/finally.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Finally::Finally (const loc& location, const rExp& body,
                    const rExp& finally)
    : Exp (location),
      body_ (body),
      finally_ (finally)
  { }

  Finally::~Finally ()
  {
  }

  void
  Finally::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Finally::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Finally::node_type() const
  {
    return "Finally";
  }

  void
  Finally::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

