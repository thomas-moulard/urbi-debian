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
 ** \file ast/message.cc
 ** \brief Implementation of ast::Message.
 */

#include <ast/visitor.hh>
#include <ast/message.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif

namespace ast
{

  Message::Message (const loc& location, const std::string& text,
                    const std::string& tag)
    : Exp (location),
      text_ (text),
      tag_ (tag)
  { }

  Message::~Message ()
  {
  }

  void
  Message::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Message::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Message::node_type() const
  {
    return "Message";
  }

  void
  Message::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

