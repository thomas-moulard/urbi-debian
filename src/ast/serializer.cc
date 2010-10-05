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
 ** \file ast/serializer-generator.cc
 ** \brief Implementation of ast::Cloner.
 */


#include <ast/serializer.hh>
#include <ast/all.hh>

namespace ast
{
  SerializerPrinter::SerializerPrinter(std::ostream& output)
    : output_(output)
    , id_(0)
    , ids_()
    , root_(true)
  {
  }

  SerializerPrinter::~SerializerPrinter()
  {}

  template <typename T>
  std::string
  escape(const T& e)
  {
    std::stringstream s;
    s << e;
    std::string res = s.str();
    for (std::string::size_type i = res.find_first_of("\\");
         i != std::string::npos;
	 i = res.find_first_of("\\", i+3))
      res.replace(i, 1, "\\\\");
    for (std::string::size_type i = res.find_first_of("\"");
         i != std::string::npos;
	 i = res.find_first_of("\"", i+4))
      res.replace(i, 1, "\\\"");
    return res;
  }

  void SerializerPrinter::operator()(const ast::Ast* n)
  {
    if (!n)
      return;
    if (root_)
    {
      root_ = false;
      startDictionary("");
      super_type::operator()(n);
      stopDictionary();
      output_ << ";" << std::endl;
    }
    else
      super_type::operator()(n);
  }

  template<typename T>
  void
  SerializerPrinter::recurse(const T& c)
  {
    foreach (const typename T::value_type& elt, c)
      if (elt.get())
        operator()(elt.get());
  }

  template<>
  void
  SerializerPrinter::recurse<symbols_type>(const symbols_type&)
  {}

  template<>
  void
  SerializerPrinter::recurse<ast::modifiers_type>(const ast::modifiers_type&)
  {}

  template<typename T>
  void
  SerializerPrinter::recurse(const T* c)
  {
    if (c)
      recurse(*c);
  }

  void
  SerializerPrinter::separator(const char* name)
  {
    fill(stack_.size());
    if (!stack_.empty() && stack_.back() == LISTCONT)
      output_ << ',';
    if (!stack_.empty() && stack_.back() == LISTSTART)
      stack_.back() = LISTCONT;
    if (!stack_.empty() && stack_.back() == DICTIONARY)
    {
      output_ << ", ";
      if (name == std::string("closure"))
        output_ << "\"Closure\"" << "=>";
      else
        output_ << '"' << name << '"' << "=>";
    }
  }
  void
  SerializerPrinter::startDictionary(const char* name)
  {
    //output_ << "startdict" << std::endl;
    separator(name);
    stack_.push_back(DICTIONARY);
    output_  << '[' << "\"NodeName\" =>" << '"' << name << '"' << std::endl;
  }
  void
  SerializerPrinter::stopDictionary()
  {
    //output_ << "stopdict" << std::endl;
    if (stack_.empty() || stack_.back() != DICTIONARY)
      exit(12);
    stack_.pop_back();
    fill(stack_.size());
    output_ << "]" << std::endl;
  }
  void
  SerializerPrinter::startList(const char* name)
  {
    //output_ << "startlist" << std::endl;
    separator(name);
    stack_.push_back(LISTSTART);
    output_ << '[' << std::endl;
  }
  void
  SerializerPrinter::stopList()
  {
    //output_ << "stoplist" << std::endl;
    if (stack_.empty() || stack_.back() == DICTIONARY)
      exit(12);
    stack_.pop_back();
    fill(stack_.size());
    output_ << ']' << std::endl;
  }
  void
  SerializerPrinter::showElement(const char* name, const char* value)
  {
    //output_ << "elem" << std::endl;
    fill(stack_.size());
    separator(name);
    output_ << '"' << escape(value) << '"';
    output_ << std::endl;
  }
  void SerializerPrinter::fill(size_t n)
  {
    char f[1024];
    memset(f, ' ', 1023);
    f[n] = 0;
    output_ << f;
  }
    void
  SerializerPrinter::visit(const And* n)
  {
    (void) n;
      startDictionary("And");
  element("type", "And");
  element("location", n->location_get());
  startList("children");
  recurse(n->children_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Assign* n)
  {
    (void) n;
      startDictionary("Assign");
  element("type", "Assign");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  startList("modifiers");
  recurse(n->modifiers_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Assignment* n)
  {
    (void) n;
      startDictionary("Assignment");
  element("type", "Assignment");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const At* n)
  {
    (void) n;
      startDictionary("At");
  element("type", "At");
  element("location", n->location_get());
  element("flavor", n->flavor_get());
  element("flavor_location", n->flavor_location_get());
  startDictionary("cond");
  operator()(n->cond_get().get());  stopDictionary();
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  startDictionary("onleave");
  operator()(n->onleave_get().get());  stopDictionary();
  startDictionary("duration");
  operator()(n->duration_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Binding* n)
  {
    (void) n;
      startDictionary("Binding");
  element("type", "Binding");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  element("constant", n->constant_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Break* n)
  {
    (void) n;
      startDictionary("Break");
  element("type", "Break");
  element("location", n->location_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Call* n)
  {
    (void) n;
      startDictionary("Call");
  element("type", "Call");
  element("location", n->location_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  startDictionary("target");
  operator()(n->target_get().get());  stopDictionary();
  element("name", n->name_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const CallMsg* n)
  {
    (void) n;
      startDictionary("CallMsg");
  element("type", "CallMsg");
  element("location", n->location_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Catch* n)
  {
    (void) n;
      startDictionary("Catch");
  element("type", "Catch");
  element("location", n->location_get());
  startDictionary("match");
  operator()(n->match_get().get());  stopDictionary();
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Class* n)
  {
    (void) n;
      startDictionary("Class");
  element("type", "Class");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  startList("protos");
  recurse(n->protos_get());
  stopList();
  startDictionary("content");
  operator()(n->content_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Continue* n)
  {
    (void) n;
      startDictionary("Continue");
  element("type", "Continue");
  element("location", n->location_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Declaration* n)
  {
    (void) n;
      startDictionary("Declaration");
  element("type", "Declaration");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  element("constant", n->constant_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Decrementation* n)
  {
    (void) n;
      startDictionary("Decrementation");
  element("type", "Decrementation");
  element("location", n->location_get());
  startDictionary("exp");
  operator()(n->exp_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Dictionary* n)
  {
    (void) n;
      startDictionary("Dictionary");
  element("type", "Dictionary");
  element("location", n->location_get());
  startDictionary("base");
  operator()(n->base_get().get());  stopDictionary();
  startList("value");
  recurse(n->value_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Do* n)
  {
    (void) n;
      startDictionary("Do");
  element("type", "Do");
  element("location", n->location_get());
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  startDictionary("target");
  operator()(n->target_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Emit* n)
  {
    (void) n;
      startDictionary("Emit");
  element("type", "Emit");
  element("location", n->location_get());
  startDictionary("event");
  operator()(n->event_get().get());  stopDictionary();
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  startDictionary("duration");
  operator()(n->duration_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Event* n)
  {
    (void) n;
      startDictionary("Event");
  element("type", "Event");
  element("location", n->location_get());
  startDictionary("exp");
  operator()(n->exp_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Finally* n)
  {
    (void) n;
      startDictionary("Finally");
  element("type", "Finally");
  element("location", n->location_get());
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  startDictionary("finally");
  operator()(n->finally_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Float* n)
  {
    (void) n;
      startDictionary("Float");
  element("type", "Float");
  element("location", n->location_get());
  element("value", n->value_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Foreach* n)
  {
    (void) n;
      startDictionary("Foreach");
  element("type", "Foreach");
  element("location", n->location_get());
  element("flavor", n->flavor_get());
  startDictionary("index");
  operator()(n->index_get().get());  stopDictionary();
  startDictionary("list");
  operator()(n->list_get().get());  stopDictionary();
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const If* n)
  {
    (void) n;
      startDictionary("If");
  element("type", "If");
  element("location", n->location_get());
  startDictionary("test");
  operator()(n->test_get().get());  stopDictionary();
  startDictionary("thenclause");
  operator()(n->thenclause_get().get());  stopDictionary();
  startDictionary("elseclause");
  operator()(n->elseclause_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Implicit* n)
  {
    (void) n;
      startDictionary("Implicit");
  element("type", "Implicit");
  element("location", n->location_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Incrementation* n)
  {
    (void) n;
      startDictionary("Incrementation");
  element("type", "Incrementation");
  element("location", n->location_get());
  startDictionary("exp");
  operator()(n->exp_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const List* n)
  {
    (void) n;
      startDictionary("List");
  element("type", "List");
  element("location", n->location_get());
  startList("value");
  recurse(n->value_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Local* n)
  {
    (void) n;
      startDictionary("Local");
  element("type", "Local");
  element("location", n->location_get());
  element("name", n->name_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  element("depth", n->depth_get());
  startDictionary("declaration");
  operator()(n->declaration_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const LocalAssignment* n)
  {
    (void) n;
      startDictionary("LocalAssignment");
  element("type", "LocalAssignment");
  element("location", n->location_get());
  element("what", n->what_get());
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  element("local_index", n->local_index_get());
  element("depth", n->depth_get());
  startDictionary("declaration");
  operator()(n->declaration_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const LocalDeclaration* n)
  {
    (void) n;
      startDictionary("LocalDeclaration");
  element("type", "LocalDeclaration");
  element("location", n->location_get());
  element("what", n->what_get());
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  element("local_index", n->local_index_get());
  element("constant", n->constant_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Match* n)
  {
    (void) n;
      startDictionary("Match");
  element("type", "Match");
  element("location", n->location_get());
  startDictionary("pattern");
  operator()(n->pattern_get().get());  stopDictionary();
  startDictionary("guard");
  operator()(n->guard_get().get());  stopDictionary();
  startDictionary("bindings");
  operator()(n->bindings_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Message* n)
  {
    (void) n;
      startDictionary("Message");
  element("type", "Message");
  element("location", n->location_get());
  element("text", n->text_get());
  element("tag", n->tag_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const MetaArgs* n)
  {
    (void) n;
      startDictionary("MetaArgs");
  element("type", "MetaArgs");
  element("location", n->location_get());
  startDictionary("lvalue");
  operator()(n->lvalue_get().get());  stopDictionary();
  element("id", n->id_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const MetaCall* n)
  {
    (void) n;
      startDictionary("MetaCall");
  element("type", "MetaCall");
  element("location", n->location_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  startDictionary("target");
  operator()(n->target_get().get());  stopDictionary();
  element("id", n->id_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const MetaExp* n)
  {
    (void) n;
      startDictionary("MetaExp");
  element("type", "MetaExp");
  element("location", n->location_get());
  element("id", n->id_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const MetaId* n)
  {
    (void) n;
      startDictionary("MetaId");
  element("type", "MetaId");
  element("location", n->location_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  element("id", n->id_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const MetaLValue* n)
  {
    (void) n;
      startDictionary("MetaLValue");
  element("type", "MetaLValue");
  element("location", n->location_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  element("id", n->id_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Nary* n)
  {
    (void) n;
      startDictionary("Nary");
  element("type", "Nary");
  element("location", n->location_get());
  startList("children");
  recurse(n->children_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Noop* n)
  {
    (void) n;
      startDictionary("Noop");
  element("type", "Noop");
  element("location", n->location_get());
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const OpAssignment* n)
  {
    (void) n;
      startDictionary("OpAssignment");
  element("type", "OpAssignment");
  element("location", n->location_get());
  startDictionary("what");
  operator()(n->what_get().get());  stopDictionary();
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  element("op", n->op_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Pipe* n)
  {
    (void) n;
      startDictionary("Pipe");
  element("type", "Pipe");
  element("location", n->location_get());
  startList("children");
  recurse(n->children_get());
  stopList();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Property* n)
  {
    (void) n;
      startDictionary("Property");
  element("type", "Property");
  element("location", n->location_get());
  startDictionary("owner");
  operator()(n->owner_get().get());  stopDictionary();
  element("name", n->name_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const PropertyWrite* n)
  {
    (void) n;
      startDictionary("PropertyWrite");
  element("type", "PropertyWrite");
  element("location", n->location_get());
  startDictionary("owner");
  operator()(n->owner_get().get());  stopDictionary();
  element("name", n->name_get());
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Return* n)
  {
    (void) n;
      startDictionary("Return");
  element("type", "Return");
  element("location", n->location_get());
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Routine* n)
  {
    (void) n;
      startDictionary("Routine");
  element("type", "Routine");
  element("location", n->location_get());
  element("closure", n->closure_get());
  startList("formals");
  recurse(n->formals_get());
  stopList();
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  startList("local_variables");
  recurse(n->local_variables_get());
  stopList();
  startList("captured_variables");
  recurse(n->captured_variables_get());
  stopList();
  element("local_size", n->local_size_get());
  element("uses_call", n->uses_call_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Scope* n)
  {
    (void) n;
      startDictionary("Scope");
  element("type", "Scope");
  element("location", n->location_get());
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Stmt* n)
  {
    (void) n;
      startDictionary("Stmt");
  element("type", "Stmt");
  element("location", n->location_get());
  element("flavor", n->flavor_get());
  startDictionary("expression");
  operator()(n->expression_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const String* n)
  {
    (void) n;
      startDictionary("String");
  element("type", "String");
  element("location", n->location_get());
  element("value", n->value_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Subscript* n)
  {
    (void) n;
      startDictionary("Subscript");
  element("type", "Subscript");
  element("location", n->location_get());
  startList("arguments");
  recurse(n->arguments_get());
  stopList();
  startDictionary("target");
  operator()(n->target_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const TaggedStmt* n)
  {
    (void) n;
      startDictionary("TaggedStmt");
  element("type", "TaggedStmt");
  element("location", n->location_get());
  startDictionary("tag");
  operator()(n->tag_get().get());  stopDictionary();
  startDictionary("exp");
  operator()(n->exp_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const This* n)
  {
    (void) n;
      startDictionary("This");
  element("type", "This");
  element("location", n->location_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Throw* n)
  {
    (void) n;
      startDictionary("Throw");
  element("type", "Throw");
  element("location", n->location_get());
  startDictionary("value");
  operator()(n->value_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Try* n)
  {
    (void) n;
      startDictionary("Try");
  element("type", "Try");
  element("location", n->location_get());
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  startList("handlers");
  recurse(n->handlers_get());
  stopList();
  startDictionary("elseclause");
  operator()(n->elseclause_get().get());  stopDictionary();
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const Unscope* n)
  {
    (void) n;
      startDictionary("Unscope");
  element("type", "Unscope");
  element("location", n->location_get());
  element("count", n->count_get());
  stopDictionary();
;
  }

  void
  SerializerPrinter::visit(const While* n)
  {
    (void) n;
      startDictionary("While");
  element("type", "While");
  element("location", n->location_get());
  element("flavor", n->flavor_get());
  startDictionary("test");
  operator()(n->test_get().get());  stopDictionary();
  startDictionary("body");
  operator()(n->body_get().get());  stopDictionary();
  stopDictionary();
;
  }


}

