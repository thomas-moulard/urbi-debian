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
 ** \file ast/nary.cc
 ** \brief Implementation of ast::Nary.
 */

#include <ast/visitor.hh>
#include <ast/nary.hh>
#include <runner/interpreter.hh>
#include <libport/compilation.hh>

// If in speed mode, inline Interpreters visit methods.
#ifdef LIBPORT_COMPILATION_MODE_SPEED
# include <runner/interpreter-visit.hxx>
#endif
#include <libport/indent.hh>
#include <libport/separate.hh>

#include <ast/message.hh>
#include <ast/stmt.hh>


namespace ast
{

  Nary::Nary()
    : Exp(loc())
    , children_(new exps_type())
  {}

  void
  Nary::clear()
  {
    children_->clear();
    location_ = loc();
  }

  bool
  Nary::empty() const
  {
    return children_->empty();
  }

  void
  Nary::location_adjust()
  {
    if (empty())
      location_.initialize(0);
    else
    {
      location_.begin = children_->front()->location_get().begin;
      location_.end   = children_->back() ->location_get().end;
    }
  }

  static Stmt* stmt(Exp* e, flavor_type f)
  {
    if (Stmt* s = dynamic_cast<Stmt*>(e))
      return s;
    else
      return new Stmt(e->location_get(), f, e);
  }

  void
  Nary::push_back(rExp e, flavor_type f)
  {
    *children_ << stmt(e.get(), f);
    location_adjust();
  }

  void
  Nary::push_message(rMessage m)
  {
    *children_ << m;
  }

  template <typename T>
  libport::intrusive_ptr<T>
  Nary::back()
  {
    return (empty()
            ? libport::intrusive_ptr<T>()
            : libport::unsafe_cast<T>(children_->back()));
  }

  template <typename T>
  libport::intrusive_ptr<const T>
  Nary::back() const
  {
    return (empty()
            ? libport::intrusive_ptr<const T>()
            : libport::unsafe_cast<const T>(children_->back()));
  }

  void
  Nary::back_flavor_set(flavor_type k, const loc& l)
  {
    if (rStmt b = back<Stmt>())
    {
      b->flavor_set(k);
      b->location_set(b->location_get() + l);
    }
    location_adjust();
  }

  void
  Nary::back_flavor_set(flavor_type k)
  {
    if (rStmt b = back<Stmt>())
       b->flavor_set(k);
  }

  flavor_type
  Nary::back_flavor_get() const
  {
    if (rConstStmt b = back<Stmt>())
      return b->flavor_get();
    return flavor_none;
  }

  void
  Nary::pop_back()
  {
    children_->pop_back();
    location_adjust();
  }

  void
  Nary::splice_back(rNary rhs)
  {
    foreach (rExp elt, rhs->children_get())
      *children_ << elt;
    rhs->clear();
    location_adjust();
  }

  Nary::Nary (const loc& location)
    : Exp (location),
      children_ (new exps_type())
  { }

  Nary::~Nary ()
  {
    delete children_;
  }

  void
  Nary::accept (ConstVisitor& v) const
  {
    v.visit(this);
  }

  urbi::object::rObject Nary::eval(runner::Interpreter& r) const
  {
    urbi::object::rObject res = r.visit(this);
    return assert_exp(res);
  }

  std::string Nary::node_type() const
  {
    return "Nary";
  }

  void
  Nary::accept (Visitor& v)
  {
    v.visit(this);
  }

} // namespace ast

