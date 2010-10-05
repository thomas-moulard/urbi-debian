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
 ** \file ast/dot-generator.cc
 ** \brief Implementation of ast::Cloner.
 */


#include <ast/dot-printer.hh>
#include <ast/all.hh>

namespace ast
{
  DotPrinter::DotPrinter(std::ostream& output)
    : output_(output)
    , id_(0)
    , ids_()
    , root_(true)
  {}

  DotPrinter::~DotPrinter()
  {}

  template <typename T>
  std::string
  escape(const T& e)
  {
    std::stringstream s;
    s << e;
    std::string res = s.str();
    for (std::string::size_type i = res.find_first_of("<>");
         i != std::string::npos;
	 i = res.find_first_of("<>", i+4))
      res.replace(i, 1, res[i] == '<' ? "&lt;" : "&gt;");
    return res;
  }

  void DotPrinter::operator()(const ast::Ast* n)
  {
    if (!n)
      return;
    if (root_)
    {
      root_ = false;
      output_ << "digraph" << std::endl
              << "{" << std::endl
              << "  node [shape=\"record\"];" << std::endl;
      super_type::operator()(n);
      output_ << "}" << std::endl;
    }
    else
      super_type::operator()(n);
  }

  template<typename T>
  void
  DotPrinter::recurse(const T& c)
  {
    foreach (const typename T::value_type& elt, c)
      if (elt.get())
        operator()(elt.get());
  }

  template<>
  void
  DotPrinter::recurse<symbols_type>(const symbols_type&)
  {}

  template<>
  void
  DotPrinter::recurse<ast::modifiers_type>(const ast::modifiers_type&)
  {}

  template<typename T>
  void
  DotPrinter::recurse(const T* c)
  {
    if (c)
      recurse(*c);
  }

    void
  DotPrinter::visit(const And* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{And|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "children";
    recurse(n->children_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Assign* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Assign|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());
    ids_.back().second = "value";
    operator() (n->value_get().get());
    ids_.back().second = "modifiers";
    recurse(n->modifiers_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Assignment* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Assignment|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const At* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{At|{" << escape(n->location_get()) << " }|{flavor: " << escape(n->flavor_get()) << "|flavor_location: " << escape(n->flavor_location_get()) << "}}\"];" << std::endl;
    ids_.back().second = "cond";
    operator() (n->cond_get().get());
    ids_.back().second = "body";
    operator() (n->body_get().get());
    ids_.back().second = "onleave";
    operator() (n->onleave_get().get());
    ids_.back().second = "duration";
    operator() (n->duration_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Binding* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Binding|{" << escape(n->location_get()) << " }|{constant: " << escape(n->constant_get()) << "}}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Break* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Break|{" << escape(n->location_get()) << " }}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Call* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Call|{" << escape(n->location_get()) << " }|{name: " << escape(n->name_get()) << "}}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());
    ids_.back().second = "target";
    operator() (n->target_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const CallMsg* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{CallMsg|{" << escape(n->location_get()) << " }}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Catch* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Catch|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "match";
    operator() (n->match_get().get());
    ids_.back().second = "body";
    operator() (n->body_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Class* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Class|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());
    ids_.back().second = "protos";
    recurse(n->protos_get());
    ids_.back().second = "content";
    operator() (n->content_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Continue* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Continue|{" << escape(n->location_get()) << " }}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Declaration* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Declaration|{" << escape(n->location_get()) << " }|{constant: " << escape(n->constant_get()) << "}}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Decrementation* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Decrementation|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "exp";
    operator() (n->exp_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Dictionary* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Dictionary|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "base";
    operator() (n->base_get().get());
    ids_.back().second = "value";
    recurse(n->value_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Do* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Do|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "body";
    operator() (n->body_get().get());
    ids_.back().second = "target";
    operator() (n->target_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Emit* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Emit|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "event";
    operator() (n->event_get().get());
    ids_.back().second = "arguments";
    recurse(n->arguments_get());
    ids_.back().second = "duration";
    operator() (n->duration_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Event* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Event|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "exp";
    operator() (n->exp_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Finally* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Finally|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "body";
    operator() (n->body_get().get());
    ids_.back().second = "finally";
    operator() (n->finally_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Float* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Float|{" << escape(n->location_get()) << " }|{value: " << escape(n->value_get()) << "}}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Foreach* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Foreach|{" << escape(n->location_get()) << " }|{flavor: " << escape(n->flavor_get()) << "}}\"];" << std::endl;
    ids_.back().second = "index";
    operator() (n->index_get().get());
    ids_.back().second = "list";
    operator() (n->list_get().get());
    ids_.back().second = "body";
    operator() (n->body_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const If* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{If|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "test";
    operator() (n->test_get().get());
    ids_.back().second = "thenclause";
    operator() (n->thenclause_get().get());
    ids_.back().second = "elseclause";
    operator() (n->elseclause_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Implicit* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Implicit|{" << escape(n->location_get()) << " }}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Incrementation* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Incrementation|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "exp";
    operator() (n->exp_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const List* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{List|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "value";
    recurse(n->value_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Local* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Local|{" << escape(n->location_get()) << " }|{name: " << escape(n->name_get()) << "|depth: " << escape(n->depth_get()) << "}}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const LocalAssignment* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{LocalAssignment|{" << escape(n->location_get()) << " }|{what: " << escape(n->what_get()) << "|local_index: " << escape(n->local_index_get()) << "|depth: " << escape(n->depth_get()) << "}}\"];" << std::endl;
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const LocalDeclaration* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{LocalDeclaration|{" << escape(n->location_get()) << " }|{what: " << escape(n->what_get()) << "|local_index: " << escape(n->local_index_get()) << "|constant: " << escape(n->constant_get()) << "}}\"];" << std::endl;
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Match* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Match|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "pattern";
    operator() (n->pattern_get().get());
    ids_.back().second = "guard";
    operator() (n->guard_get().get());
    ids_.back().second = "bindings";
    operator() (n->bindings_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Message* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Message|{" << escape(n->location_get()) << " }|{text: " << escape(n->text_get()) << "|tag: " << escape(n->tag_get()) << "}}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const MetaArgs* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{MetaArgs|{" << escape(n->location_get()) << " }|{id: " << escape(n->id_get()) << "}}\"];" << std::endl;
    ids_.back().second = "lvalue";
    operator() (n->lvalue_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const MetaCall* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{MetaCall|{" << escape(n->location_get()) << " }|{id: " << escape(n->id_get()) << "}}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());
    ids_.back().second = "target";
    operator() (n->target_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const MetaExp* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{MetaExp|{" << escape(n->location_get()) << " }|{id: " << escape(n->id_get()) << "}}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const MetaId* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{MetaId|{" << escape(n->location_get()) << " }|{id: " << escape(n->id_get()) << "}}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const MetaLValue* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{MetaLValue|{" << escape(n->location_get()) << " }|{id: " << escape(n->id_get()) << "}}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Nary* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Nary|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "children";
    recurse(n->children_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Noop* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Noop|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "body";
    operator() (n->body_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const OpAssignment* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{OpAssignment|{" << escape(n->location_get()) << " }|{op: " << escape(n->op_get()) << "}}\"];" << std::endl;
    ids_.back().second = "what";
    operator() (n->what_get().get());
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Pipe* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Pipe|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "children";
    recurse(n->children_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Property* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Property|{" << escape(n->location_get()) << " }|{name: " << escape(n->name_get()) << "}}\"];" << std::endl;
    ids_.back().second = "owner";
    operator() (n->owner_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const PropertyWrite* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{PropertyWrite|{" << escape(n->location_get()) << " }|{name: " << escape(n->name_get()) << "}}\"];" << std::endl;
    ids_.back().second = "owner";
    operator() (n->owner_get().get());
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Return* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Return|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Routine* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Routine|{" << escape(n->location_get()) << " }|{closure: " << escape(n->closure_get()) << "|local_size: " << escape(n->local_size_get()) << "|uses_call: " << escape(n->uses_call_get()) << "}}\"];" << std::endl;
    ids_.back().second = "formals";
    recurse(n->formals_get());
    ids_.back().second = "body";
    operator() (n->body_get().get());
    ids_.back().second = "local_variables";
    recurse(n->local_variables_get());
    ids_.back().second = "captured_variables";
    recurse(n->captured_variables_get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Scope* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Scope|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "body";
    operator() (n->body_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Stmt* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Stmt|{" << escape(n->location_get()) << " }|{flavor: " << escape(n->flavor_get()) << "}}\"];" << std::endl;
    ids_.back().second = "expression";
    operator() (n->expression_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const String* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{String|{" << escape(n->location_get()) << " }|{value: " << escape(n->value_get()) << "}}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Subscript* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Subscript|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "arguments";
    recurse(n->arguments_get());
    ids_.back().second = "target";
    operator() (n->target_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const TaggedStmt* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{TaggedStmt|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "tag";
    operator() (n->tag_get().get());
    ids_.back().second = "exp";
    operator() (n->exp_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const This* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{This|{" << escape(n->location_get()) << " }}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Throw* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Throw|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "value";
    operator() (n->value_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Try* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Try|{" << escape(n->location_get()) << " }}\"];" << std::endl;
    ids_.back().second = "body";
    operator() (n->body_get().get());
    ids_.back().second = "handlers";
    recurse(n->handlers_get());
    ids_.back().second = "elseclause";
    operator() (n->elseclause_get().get());

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const Unscope* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{Unscope|{" << escape(n->location_get()) << " }|{count: " << escape(n->count_get()) << "}}\"];" << std::endl;

    ids_.pop_back();
  }

  void
  DotPrinter::visit(const While* n)
  {
    (void) n;

    ++id_;
    if (!ids_.empty())
      output_ << "  node_" << ids_.back().first << " -> node_" << id_
              << "[label=\"" << ids_.back().second << "\"];" << std::endl;
    ids_.push_back(std::make_pair(id_, ""));
    output_ << "  node_" << id_ << " [label=\"{While|{" << escape(n->location_get()) << " }|{flavor: " << escape(n->flavor_get()) << "}}\"];" << std::endl;
    ids_.back().second = "test";
    operator() (n->test_get().get());
    ids_.back().second = "body";
    operator() (n->body_get().get());

    ids_.pop_back();
  }


}

