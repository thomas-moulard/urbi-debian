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
 ** \file ast/default-visitor.hxx
 ** \brief Implementation of ast::DefaultVisitor.
 */

#ifndef AST_DEFAULT_VISITOR_HXX
# define AST_DEFAULT_VISITOR_HXX

# include <ast/default-visitor.hh>
# include <ast/all.hh>

namespace ast
{

  template <template <typename> class Const>
  GenDefaultVisitor<Const>::GenDefaultVisitor ()
    : GenVisitor<Const> ()
  {
  }

  template <template <typename> class Const>
  GenDefaultVisitor<Const>::~GenDefaultVisitor ()
  {
  }


  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (And_type*  n)
  {
    visit((typename Const<Composite>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Assign_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->what_get().get());
    operator()(n->value_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Assignment_type*  n)
  {
    visit((typename Const<Write>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Ast_type* )
  {}

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (At_type*  n)
  {
    visit((typename Const<Flavored>::type*) n);
    operator()(n->cond_get().get());
    operator()(n->body_get().get());
    if (n->onleave_get())
      operator()(n->onleave_get().get());
    if (n->duration_get())
      operator()(n->duration_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Binding_type*  n)
  {
    visit((typename Const<LValue>::type*) n);
    operator()(n->what_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Break_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Call_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->target_get().get());
    if (n->arguments_get())
    {
      foreach (libport::intrusive_ptr<typename Const<ast::Exp>::type> e, *n->arguments_get())
        operator()(e.get());
    }

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (CallMsg_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Catch_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    if (n->match_get())
      operator()(n->match_get().get());
    operator()(n->body_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Class_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->what_get().get());
    operator()(n->content_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Composite_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Continue_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Declaration_type*  n)
  {
    visit((typename Const<Write>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Decrementation_type*  n)
  {
    visit((typename Const<Unary>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Dictionary_type*  n)
  {
    /* Kind of dump to iterate on vals without the keys */
    foreach (modifiers_type::value_type exp, n->value_get())
      ; //operator()(exp.second.get());

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Do_type*  n)
  {
    operator()(n->target_get().get());
    visit((typename Const<Scope>::type*) n);

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Emit_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->event_get().get());
    if (n->duration_get())
      operator()(n->duration_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Event_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->exp_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Exp_type*  n)
  {
    visit((typename Const<Ast>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Finally_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->body_get().get());
    operator()(n->finally_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Flavored_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Float_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Foreach_type*  n)
  {
    visit((typename Const<Flavored>::type*) n);
    operator()(n->index_get().get());
    operator()(n->list_get().get());
    operator()(n->body_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (If_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->test_get().get());
    operator()(n->thenclause_get().get());
    operator()(n->elseclause_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Implicit_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Incrementation_type*  n)
  {
    visit((typename Const<Unary>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (LValue_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (LValueArgs_type*  n)
  {
    visit((typename Const<LValue>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (List_type*  n)
  {
    foreach (libport::intrusive_ptr<typename Const<ast::Exp>::type> exp, n->value_get())
      operator()(exp.get());

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Local_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    if (n->arguments_get())
    {
      foreach (libport::intrusive_ptr<typename Const<ast::Exp>::type> e, *n->arguments_get())
        operator()(e.get());
    }

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (LocalAssignment_type*  n)
  {
    visit((typename Const<LocalWrite>::type*) n);
    operator()(n->declaration_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (LocalDeclaration_type*  n)
  {
    visit((typename Const<LocalWrite>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (LocalWrite_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->value_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Match_type*  n)
  {
    visit((typename Const<Ast>::type*) n);
    operator()(n->pattern_get().get());
    if (n->guard_get())
      operator()(n->guard_get().get());
    if (n->bindings_get())
      operator()(n->bindings_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Message_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (MetaArgs_type*  n)
  {
    visit((typename Const<LValue>::type*) n);
    operator()(n->lvalue_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (MetaCall_type*  n)
  {
    visit((typename Const<LValueArgs>::type*) n);
    operator()(n->target_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (MetaExp_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (MetaId_type*  n)
  {
    visit((typename Const<LValueArgs>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (MetaLValue_type*  n)
  {
    visit((typename Const<LValueArgs>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Nary_type*  n)
  {
    foreach (libport::intrusive_ptr<typename Const<ast::Ast>::type> c, n->children_get())
      operator()(c.get());

  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Noop_type*  n)
  {
    visit((typename Const<Scope>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (OpAssignment_type*  n)
  {
    visit((typename Const<Write>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Pipe_type*  n)
  {
    visit((typename Const<Composite>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Property_type*  n)
  {
    visit((typename Const<PropertyAction>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (PropertyAction_type*  n)
  {
    visit((typename Const<LValue>::type*) n);
    operator()(n->owner_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (PropertyWrite_type*  n)
  {
    visit((typename Const<PropertyAction>::type*) n);
    operator()(n->value_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Return_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    if (n->value_get())
      operator()(n->value_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Routine_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    if (n->body_get())
      operator()(n->body_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Scope_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->body_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Stmt_type*  n)
  {
    visit((typename Const<Flavored>::type*) n);
    operator()(n->expression_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (String_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Subscript_type*  n)
  {
    visit((typename Const<LValueArgs>::type*) n);
    operator()(n->target_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (TaggedStmt_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->tag_get().get());
    operator()(n->exp_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (This_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Throw_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    if (n->value_get())
      operator()(n->value_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Try_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->body_get().get());
    if (n->elseclause_get())
      operator()(n->elseclause_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Unary_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->exp_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Unscope_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (While_type*  n)
  {
    visit((typename Const<Flavored>::type*) n);
    operator()(n->test_get().get());
    operator()(n->body_get().get());
  }

  template <template <typename> class Const>
  void
  GenDefaultVisitor<Const>::visit (Write_type*  n)
  {
    visit((typename Const<Exp>::type*) n);
    operator()(n->what_get().get());
    operator()(n->value_get().get());
  }

} // namespace ast
#endif // !AST_DEFAULT_VISITOR_HH

