/*
 * Copyright (C) 2009, 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */
/// \file parser/ugrammar.y
/// \brief Definition of the parser used by the ParserImpl object.

%require "2.3"
%language "C++"
%defines

// Instead of "yytoken yylex(yylval, yylloc)", use "symbol_type yylex()".
%define lex_symbol

// Prefix all our external definition of token names with "TOK_".
%define api.tokens.prefix "TOK_"

// The leading :: are needed to avoid symbol clashes in the
// parser class when it sees a parser namespace occurrence.
%param {::parser::ParserImpl& up} {yyFlexLexer& scanner};

%code requires // Output in ugrammar.hh.
{
#include <kernel/config.h> // YYDEBUG.

#include <ast/call.hh>
#include <ast/catches-type.hh>
#include <ast/event-match.hh>
#include <ast/exps-type.hh>
#include <ast/factory.hh>
#include <ast/fwd.hh>
#include <ast/nary.hh>
#include <ast/symbols-type.hh>
#include <kernel/fwd.hh>
#include <libport/hash.hh>
#include <libport/ufloat.hh>
#include <list>
#include <object/symbols.hh>
#include <parser/fwd.hh>
}

// Locations.
%locations
// We use pointers to store the filename in the locations.  This saves
// space (pointers), time (no deep copy), but leaves the problem of
// deallocation.  The class Symbol provides this.
%define "filename_type" "libport::Symbol"
%initial-action
{
  // Saved when exiting the start symbol.
  scanner.loc = up.loc_;
}


%code // Output in ugrammar.cc.
{
#include <string>
#include <iostream>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <libport/cassert>
#include <libport/finally.hh>
#include <libport/format.hh>
#include <libport/separate.hh>

#include <ast/all.hh>
#include <ast/new-clone.hh>
#include <ast/parametric-ast.hh>
#include <ast/print.hh>

#define MAKE(Kind, ...)                         \
  up.factory().make_ ## Kind(__VA_ARGS__)

#include <parser/parse.hh>
#include <parser/parser-impl.hh>
#include <parser/utoken.hh>

#if defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ == 4
# pragma GCC diagnostic ignored "-Wuninitialized"
#endif

  namespace
  {

    static void
    modifiers_add(parser::ParserImpl& up, const ast::loc& loc,
                  ast::modifiers_type& mods,
                  const ::ast::Factory::modifier_type& mod)
    {
      if (libport::mhas(mods, mod.first))
        up.warn(loc,
                libport::format("modifier redefined: %s", mod.first));
      mods[mod.first] = mod.second;
    }

    static void
    assocs_add(parser::ParserImpl& up, const ast::loc& loc,
                  ast::modifiers_type& mods,
                  const ::ast::Factory::modifier_type& mod)
    {
      if (libport::mhas(mods, mod.first))
        up.warn(loc,
                libport::format("key redefined: %s", mod.first));
      mods[mod.first] = mod.second;
    }

    /// Use the scanner in the right parser::ParserImpl.
    static
    inline
    yy::parser::symbol_type
    yylex(parser::ParserImpl& up, yyFlexLexer& scanner)
    {
      return scanner.yylex(&up);
    }

  } // anon namespace

# define REQUIRE_IDENTIFIER(Loc, Effective, Expected)                   \
  do {                                                                  \
    if (Effective != libport::Symbol(Expected))                         \
      up.error(Loc,                                                     \
               libport::format("unexpected `%s', expecting `%s'",       \
                               Effective, Expected));                   \
  } while (false)

} // %code requires.


/*---------.
| Tokens.  |
`---------*/

%token
        __HERE__     "__HERE__"
        EQ           "="
        BREAK        "break"
        CASE         "case"
        CATCH        "catch"
        CLOSURE      "closure"
        CONST        "const"
        CONTINUE     "continue"
        COLON        ":"
        DEFAULT       "default"
        ELSE         "else"
        EMIT         "emit"
        FINALLY      "finally"
        FREEZEIF     "freezeif"
        FUNCTION     "function"
        IF           "if"
        IN           "in"
        LBRACE       "{"
        LBRACKET     "["
        LPAREN       "("
        ONLEAVE      "onleave"
        POINT        "."
        RBRACE       "}"
        RBRACKET     "]"
        RETURN       "return"
        RPAREN       ")"
        STOPIF       "stopif"
        SWITCH       "switch"
        THROW        "throw"
        TILDA        "~"
        TIMEOUT      "timeout"
        TRY          "try"
        VAR          "var"
        WAITUNTIL    "waituntil"
        WHENEVER     "whenever"

%token EOF 0 "end of command"

 // Special tokens for the prescanner.
%token PRE_EOF        "prescanner end of file"
       PRE_WANTS_MORE "prescanner needs more input" // no terminator found
       PRE_COMPLETE   "prescanner found command" // Complete sentence.

/*----------.
| Flavors.  |
`----------*/

%code requires
{
#include <ast/flavor.hh>
};

// We use variants.
%define variant
// Default printer.
%printer { debug_stream() << libport::deref << $$; } <*>;


%token <ast::flavor_type>
        COMMA        ","
        SEMICOLON    ";"
        AMPERSAND    "&"
        PIPE         "|"
        EVERY        "every"
        FOR          "for"
        LOOP         "loop"
        WHILE        "while"
        AT           "at"
;


 /*---------.
 | Symbol.  |
 `---------*/

%token <libport::Symbol> IDENTIFIER "identifier";

// id is meant to enclose all the symbols we use as operators.  For
// instance "+" is special so that we do have the regular priority and
// asssociativity, yet we can write "foo . + (2)" and call foo's +.
%type <libport::Symbol> id;


/*--------------.
| Expressions.  |
`--------------*/

%type <ast::rExp> block exp exp.opt stmt stmt_loop;


/*----------------------.
| Operator precedence.  |
`----------------------*/

%expect 0

// man operator (precedences are increasing).

// Operator                        Associativity
// --------                        -------------
// ,                               left to right
// = += -= etc.                    right to left
// ?:                              right to left
// ||                              left to right
// &&                              left to right
// |                               left to right
// ^                               left to right
// &                               left to right
// == !=                           left to right
// < <= > >=                       left to right
// << >>                           left to right
// + -                             left to right
// * / %                           left to right
// ! ~ ++ -- - (type) * & sizeof   right to left
// () [] -> .                      left to right

 /*
   ! < ( so that !m(x) be read as !(m(x)).
 */

// This is very painful: because we want to declare the resolution
// in the following case, we give a relative precedence between "in"
// and "identifier" which kills the support of "for (var i in c)".
// We cannot address this properly, unless we have scoped precedences.
//
// Here, because we have "identifier" < "in", we promote the shift,
// which is what we want.  But it is sucks hard.
//
// There is a shift/reduce conflict that results from the two uses of "new":
//
//   new -> "new" . "identifier" args
//   id  -> "new" .
//    "identifier"  shift, and go to state 108
//    "identifier"  [reduce using rule 89 (id)]
//
// Obviously the shift should win.
%precedence "new";
%precedence "identifier";

// ":" > "identifier", to get the iteration:
//   for (var i : [1, 2, 3]) ;
//
// not the tagged statement:
//   for ((var i) : 42) ;
%precedence ":";


// Precedences, increasing.
%precedence "," ";"
%left "|"
%left "&"
%precedence CMDBLOCK
%precedence "else" "onleave" "finally"

%right "=" "+=" "-=" "*=" "/=" "^=" "%="

%precedence "const" "var"

%precedence "~" // This is not the same as in C++, this is for durations.
%left  "||"
%left  "&&"
%nonassoc "in"
%left  "bitor"
%left  "^"
%left  "bitand"
%nonassoc "==" "===" "~=" "=~=" "!=" "!=="
%nonassoc "<" "<=" ">" ">="
%left  "<<" ">>"
%left  "+" "-"
%left  "*" "/" "%"
%right "!" "compl" "++" "--" UNARY     /* Negation--unary minus */
%right "**"
%precedence "=>"
%precedence "("
%precedence "["
%precedence  "."

/* Urbi Grammar */
%%

%start start;
start:
  root
  {
    up.result_->ast_set($1);
    up.loc_ = @$;
  }
;

root:
    /* Minimal error recovery, so that all the tokens are read,
       especially the end-of-lines, to keep error messages in sync. */
  error  { $$ = 0;  }
| stmts  { std::swap($$, $1); }
;


/*--------.
| stmts.  |
`--------*/

%type <ast::rNary> root stmts;

// Statements: with ";" and ",".
stmts:
  cstmt            { $$ = MAKE(nary, @$, $1); }
| stmts ";" cstmt  { $$ = MAKE(nary, @$, $1, @2, $2, $3); }
| stmts "," cstmt  { $$ = MAKE(nary, @$, $1, @2, $2, $3); }
;

%type <ast::rExp> cstmt;
// Composite statement: with "|" and "&".
cstmt:
  stmt            { assert($1); std::swap($$, $1); }
| cstmt "|" cstmt { $$ = MAKE(bin, @$, $2, $1, $3); }
| cstmt "&" cstmt { $$ = MAKE(bin, @$, $2, $1, $3); }
;


/*--------------.
| tagged stmt.  |
`--------------*/

%type <ast::rExp> tag;
tag:
  exp { std::swap($$, $1); }
;

stmt:
  tag ":" stmt
  {
    $$ = new ast::TaggedStmt(@$, $1, MAKE(scope, @$, $3));
  }
;

/*-------.
| Stmt.  |
`-------*/

stmt:
  /* empty */ { $$ = new ast::Noop(@$, 0); }
| exp         { std::swap($$, $1); }
;

block:
  "{" stmts "}"       { $$ = MAKE(strip, $2); }
;

/*----------.
| Classes.  |
`----------*/

%token PRIVATE    "private"
       PROTECTED  "protected"
       PUBLIC     "public"
       ;

// A useless optional visibility.
visibility:
  /* Nothing. */
| "private"
| "protected"
| "public"
;

%type <ast::rExp> proto;
proto:
  visibility exp   { std::swap($$, $2); }
;

%type <ast::exps_type*> protos.1 protos;

protos.1:
  proto           { $$ = new ast::exps_type(1, $1); }
// Cannot add this part currently, as the prescanner cuts
//
//    class A : B, C {};
//
// at the comma.
//
// | protos.1 "," proto  { std::swap($$, $1); *$$ << $3; }
;

// A list of parents to derive from.
protos:
  /* nothing */   { $$ = 0; }
| ":" protos.1    { std::swap($$, $2); }
;

%token CLASS "class";
stmt:
  "class" lvalue protos block
    {
      $$ = MAKE(class, @$, $2, $3, $4);
    }
;

/*-----------.
| Bindings.  |
`-----------*/
from:
  "identifier"
  {
    REQUIRE_IDENTIFIER(@1, $1, "from");
  }
;

// "event" or "function".
%type <libport::Symbol> event_or_function;
event_or_function:
  "function"
  {
    $$ = SYMBOL(function);
  }
| "identifier"
  {
    REQUIRE_IDENTIFIER(@1, $1, "event");
    $$ = SYMBOL(event);
  }
;


%token EXTERNAL "external";
stmt:
  "external" "identifier"[object] "identifier"[id]
  {
    REQUIRE_IDENTIFIER(@object, $object, "object");
    $$ = MAKE(external_object, @$, $id);
  }
| "external" "var" "identifier"[obj] "." "identifier"[slot]
             from "identifier"[id]
  {
    $$ = MAKE(external_var, @$, $obj, $slot, $id);
  }
| "external" event_or_function[kind]
             "(" "float"[arity] ")"
             "identifier"[obj] "." "identifier"[slot]
             from "identifier"[id]
  {
    $$ = MAKE(external_event_or_function,
              @$, $kind, $arity, $obj, $slot, $id);
  }
;


/*---------.
| Events.  |
`---------*/

stmt:
  exp "!" args.opt tilda.opt
  {
    $$ = new ast::Emit(@$, $1, $3, $4);
  }
//<no-space< Emit.
| "emit" k1_id args.opt tilda.opt
  {
    up.deprecated(@$, "emit event(arg, ...)", "event!(arg, ...)");
    $$ = new ast::Emit(@$, $2, $3, $4);
  }
//>no-space>
;


/*------------.
| Functions.  |
`------------*/
// Whether is a closure (otherwise a function).
%type <bool> routine;
routine:
  "closure"  { $$ = true; }
| "function" { $$ = false; }
;

stmt:
  // If you want to use something more general than "k1_id", read the
  // comment of k1_id.
  routine k1_id formals block
    {
      // Compiled as "var name = function args stmt".
      $$ = new ast::Declaration(@$, $2,
                                MAKE(routine, @$, $1, @3, $3, @4, $4));
    }
;

/*-----------------------------.
| k1_id: A simple identifier.  |
`-----------------------------*/

// We would really like to support simultaneously the following
// constructs:
//
//  a.b = function (c) { c }
// and
//  function a.b (c) { c }
//
// unfortunately it unleashes a host of issues.  It requires introducing
// two nonterminals to denote on the one hand side "a.b.c" etc. and otoh
// "a().b(c,d).e".  Of course they overlap, so we have conflicts.
//
// We can also try to use a single nonterminal, i.e., accept to parse:
//
//   function a(b).c(1) { 1 }
//
// but then reject it by hand once "formal arguments" have been "reparsed".
// It sucks.  Yet we have another problem: the two "function"-constructs
// conflict between themselves.  The LR(1) parser cannot tell whether
// "function (" is starting an lambda expression ("a = function (){ 1 }")
// or a named function ("function (a).f() { 1 }").  So I chose to limit
// named function to what we need to be compatible with k1: basically
// "function a ()", "function a.b()", and so on.
//
// Another option would have been to use two keywords, say using "lambda"
// for anonymous functions.  But that's not a good option IMHO (AD).
%type <ast::rCall> k1_id;
k1_id:
  "identifier"               { $$ = MAKE(call, @$, $1); }
| k1_id "." "identifier"     { $$ = MAKE(call, @$, ast::rExp($1), $3); }
;


/*------------.
| Modifiers.  |
`------------*/


%type <::ast::Factory::modifier_type> modifier;

modifier:
  "identifier" ":" exp
  {
    $$.first = $1;
    $$.second = $3;
  }
;

/*-------------------.
| Stmt: Assignment.  |
`-------------------*/

exp:
  exp "=" exp
  {
    ast::rDictionary d = $3.unsafe_cast<ast::Dictionary>();
    // If exp is "e0 k1: e1 k2: e2...", i.e. a dictionary plus a
    // "base" (e0), then this is a trajectory.
    if (d && d->base_get())
      $$ = new ast::Assign(@$, $1, d->base_get(),
                           new ast::modifiers_type(d->value_get()));
    else
      $$ = new ast::Assign(@$, $1, $3, 0);
  }
| exp modifier
  {
    if (ast::rDictionary d = $1.unsafe_cast<ast::Dictionary>())
    {
      modifiers_add(up, @2, d->value_get(), $2);
      $$ = $1;
    }
    else if (ast::rAssign a = $1.unsafe_cast<ast::Assign>())
    {
      ast::modifiers_type* m = a->modifiers_get();
      if (!m)
      {
        m = new ast::modifiers_type();
        a->modifiers_set(m);
      }
      modifiers_add(up, @2, *a->modifiers_get(), $2);
      $$ = $1;
    }
    else
    {
      ast::rDictionary d = new ast::Dictionary(@$, 0, ast::modifiers_type());
      modifiers_add(up, @2, d->value_get(), $2);
      d->base_get() = $1;
      $$ = d;
    }
  }
;

%token <libport::Symbol>
        CARET_EQ    "^="
        MINUS_EQ    "-="
        PERCENT_EQ  "%="
        PLUS_EQ     "+="
        SLASH_EQ    "/="
        STAR_EQ     "*="
;

exp:
  lvalue "+=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
| lvalue "-=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
| lvalue "*=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
| lvalue "/=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
| lvalue "^=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
| lvalue "%=" exp    { $$ = new ast::OpAssignment(@2, $1, $3, $2); }
;

%token  MINUS_MINUS "--"
        PLUS_PLUS   "++"
;
exp:
  lvalue "--"      { $$ = new ast::Decrementation(@$, $1); }
| lvalue "++"      { $$ = new ast::Incrementation(@$, $1); }
;


/*-------------.
| Properties.  |
`-------------*/

%token MINUS_GT     "->";
exp:
  lvalue "->" id
    {
      $$ = new ast::Property(@$, $1->call(), $3);
    }
;

/*---------------------.
| Stmt: Control flow.  |
`---------------------*/

// non-empty-statement: A statement that triggers a warning if empty.
%type <ast::rExp> nstmt;
nstmt:
  stmt
    {
      std::swap($$, $1);
      if (ast::implicit($$))
        up.warn(@1,
                "implicit empty instruction.  Use '{}' to make it explicit.");
    }
;

stmt:
  stmt_loop
    {
      std::swap($$, $1);
    }
| "at" "(" exp tilda.opt ")" nstmt onleave.opt
    {
      $$ = MAKE(at, @$, @1, $1, $3, $6, $7, $4);
    }
| "at" "(" event_match ")" nstmt onleave.opt
    {
      $$ = MAKE(at_event, @$, @1, $1, $3, $5, $6);
    }
| "every" "(" exp ")" nstmt
    {
      $$ = MAKE(every, @$, @1, $1, $3, $5);
    }
| "if" "(" stmts ")" nstmt else.opt
    {
      $$ = MAKE(if, @$, $3, $5, $6);
    }
| "freezeif" "(" exp ")" stmt
    {
      $$ = MAKE(freezeif, @$, $3, $5);
    }
| "stopif" "(" exp ")" stmt
    {
      $$ = MAKE(stopif, @$, $3, $5);
    }
| "switch" "(" exp ")" "{" cases default.opt "}"
    {
      $$ = MAKE(switch, @3, $3, $6, $7);
    }
| "timeout" "(" exp ")" stmt
    {
      $$ = MAKE(timeout, $3, $5);
    }
| "return" exp.opt
    {
      $$ = new ast::Return(@$, $2);
    }
| "break"
    {
      $$ = new ast::Break(@$);
    }
| "continue"
    {
      $$ = new ast::Continue(@$);
    }
| "waituntil" "(" exp tilda.opt ")"
    {
      $$ = MAKE(waituntil, @$, $3, $4);
    }
| "waituntil" "(" event_match ")"
    {
      $$ = MAKE(waituntil_event, @$, $3);
    }
| "whenever" "(" exp tilda.opt ")" nstmt else.opt
    {
      $$ = MAKE(whenever, @$, $3, $6, $7, $4);
    }
| "whenever" "(" event_match ")" nstmt else.opt
    {
      $$ = MAKE(whenever_event, @$, $3, $5, $6);
    }
;



/*----------------------------------------.
| Optional default/else/onleave clauses.  |
`----------------------------------------*/

// CMDBLOCK < "else", "finally", and "onleave" to promote shift in
// else.opt, finally.opt and onleave.opt.

%type <ast::rNary> default.opt;
default.opt:
  /* nothing. */ %prec CMDBLOCK   { $$ = 0;            }
|  "default" ":" stmts            { std::swap($$, $3); }
;

%type <ast::rExp> else.opt;
else.opt:
  /* nothing. */ %prec CMDBLOCK   { $$ = 0;            }
| "else" nstmt                    { std::swap($$, $2); }
;

// An optional onleave clause.
%type <ast::rExp> onleave.opt;
onleave.opt:
  /* nothing. */ %prec CMDBLOCK   { $$ = 0;            }
| "onleave" nstmt                 { std::swap($$, $2); }
;


/*--------.
| Cases.  |
`--------*/

%type <::ast::Factory::cases_type> cases;

cases:
  /* empty */  {}
| cases case   { std::swap($$, $1); $$ << $2; }
;

%type <::ast::Factory::case_type> case;

case:
  "case" match ":" stmts  { $$ = ::ast::Factory::case_type($2, $4); }
;

/*-------------.
| Exceptions.  |
`-------------*/

%type <ast::catches_type> catches.1;
catches.1:
  catch           { $$ = ast::catches_type(); $$ << $catch; }
| catches.1 catch { std::swap($$, $1);        $$ << $catch; }
;

%type <ast::rMatch> match match.opt;
match:
  exp           { $$ = new ast::Match(@$, $1, 0);  }
| exp "if" exp  { $$ = new ast::Match(@$, $1, $3); }

match.opt:
  /* empty */   { $$ = 0; }
| "(" match ")" { std::swap($$, $2); }

%type <ast::rCatch> catch;
catch:
  "catch" match.opt block
  {
    $$ = new ast::Catch(@$, $[match.opt], $[block]);
  }
;

%type <ast::rExp> finally.opt;
finally.opt:
  /* empty */ %prec CMDBLOCK  { $$ = 0;  }
| "finally" block             { $$ = $2; }
;

stmt:
  "try" block catches.1 else.opt finally.opt
  {
    $$ = MAKE(try, @$, $block, $[catches.1], $[else.opt], $[finally.opt]);
  }
| "try" block "finally" block
  {
    $$ = MAKE(finally, @$, $2, $4);
  }
| "throw" exp.opt
  {
    $$ = MAKE(throw, @$, $2);
  }
;

/*--------.
| Loops.  |
`--------*/

stmt_loop:
/*
 *  This loop keyword can't be converted to a for, since it would
 *  cause an ambiguity in the language. Consider this line:
 *
 *      for (42);
 *
 *  It could be either:
 *
 *      for (42)
 *        ;
 *
 *  i.e, while 42 is true execute the empty instruction, either:
 *
 *      for
 *        42;
 *
 *  i.e. execute "42"  forever, with 42 being parenthesized.
 */
  "loop" stmt %prec CMDBLOCK
    {
      $$ = MAKE(loop, @$, @1, $1, @2, $2);
    }
| "for" "(" exp ")" stmt %prec CMDBLOCK
    {
      $$ = MAKE(for, @$, @1, $1, $3, $5);
    }
| "for" "(" stmt[init] ";" exp[cond] ";" stmt[inc] ")" stmt[body] %prec CMDBLOCK
    {
      $$ = MAKE(for, @$, @1, $1, $init, $cond, $inc, $body);
    }
| "for" "(" "var" "identifier"[id] in_or_colon exp ")" stmt %prec CMDBLOCK
    {
      $$ = MAKE(for, @$, @1, $1, @id, $id, $exp, $stmt);
    }
| "while" "(" exp ")" stmt %prec CMDBLOCK
    {
      $$ = MAKE(while, @$, @1, $1, $exp, @stmt, $stmt);
    }
;

in_or_colon: "in" | ":";


/*---------------.
| Control flow.  |
`---------------*/

%token DO "do";

exp:
                   block  { $$ = MAKE(scope, @$, 0, $1);  }
| "do" "(" exp ")" block  { $$ = MAKE(scope, @$, $3, $5); }
;

/*-------------.
| Assertions.  |
`-------------*/

%token ASSERT "assert";
exp:
  "assert" "(" exp ")"    { $$ = MAKE(assert, @$, $3); }
| "assert" "{" claims "}" { $$ = MAKE(assert, @$, $3); }
;


/*---------------------------.
| Function calls, messages.  |
`---------------------------*/

%type <ast::rLValue> lvalue;
lvalue:
          id    { $$ = MAKE(call, @$, $1); }
| exp "." id    { $$ = MAKE(call, @$, $1, $3); }
;

id:
  "identifier"  { std::swap($$, $1); }
;

exp:
  "var" exp[lvalue]
  {
    $$ = MAKE(binding, @$, false, @lvalue, $lvalue);
  }
| "const" "var" exp[lvalue]
  {
    $$ = MAKE(binding, @$, true, @lvalue, $lvalue);
  }
| lvalue
  {
    $$ = $1;
  }
| lvalue args
  {
    $$ = $1;
    $$.unchecked_cast<ast::LValueArgs>()->arguments_set($2);
    $$->location_set(@$);
  }
;

//<no-space< new "identifier".
// Instantiation looks a lot like a function call.
%token <libport::Symbol> NEW "new";
%type <ast::rExp> new;
new:
  "new" "identifier" args.opt
  {
    // Compiled as "id . new (args)".
    $$ = MAKE(call, @$, MAKE(call, @$, $2), SYMBOL(new), $3);
    up.deprecated(@$, "new Obj(x)", "Obj.new(x)");
  }
;

exp:
  new   { std::swap($$, $1); }
;

// Allow Object.new etc.
id:
  "new" { std::swap($$, $1); }
;
//>no-space>


/*---------------------.
| Anonymous function.  |
`---------------------*/

exp:
  routine formals block
    {
      $$ = MAKE(routine, @$, $1, @2, $2, @3, $3);
    }
;


/*----------.
| Numbers.  |
`----------*/

%token <libport::ufloat>
        ANGLE     "angle"
        DURATION  "duration"
        FLOAT     "float";
%type <ast::rExp> exp_float;
exp_float:
  "float"  { $$ = MAKE(float, @$, $1); }
;


/*-----------.
| Duration.  |
`-----------*/

%type <libport::ufloat> duration;
duration:
  "duration"          { $$ = $1;      }
| duration "duration" { $$ = $1 + $2; }
;

/*-------------.
| Dictionary.  |
`-------------*/

%token EQ_GT "=>";

%type <ast::Factory::modifier_type> assoc;
%type <ast::rDictionary> assocs assocs.1 dictionary;

assoc:
  string "=>" exp
  {
    $$.first = libport::Symbol($1);
    $$.second = $3;
  }
;

assocs.1:
  assoc
  {
    $$ = new ast::Dictionary(@$, 0, ast::modifiers_type());
    assocs_add(up, @1, $$->value_get(), $1);
  }
| assocs.1 "," assoc
  {
    std::swap($$, $1);
    assocs_add(up, @3, $$->value_get(), $3);
  }
;

assocs:
  assocs.1     { std::swap($$, $1); }
| assocs.1 "," { std::swap($$, $1); }
;

dictionary:
  "[" "=>" "]"
  {
    $$ = new ast::Dictionary(@$, 0, ast::modifiers_type());
  }
| "[" assocs "]"
  {
    std::swap($$, $2);
  }
;

/*--------.
| Tuple.  |
`--------*/

%type <ast::exps_type*> tuple.exps tuple;
// %type <ast::rExp> tuple tuple.exps;

tuple.exps:
  /* empty */ { $$ = new ast::exps_type; }
| exps.1 ","  { std::swap($$, $1); }
| exps.2      { std::swap($$, $1); }
;

tuple:
  "(" tuple.exps ")"   { $$ = $2; }
;

/*-----------.
| Literals.  |
`-----------*/

exp:
  exp_float      { std::swap($$, $1);  }
| "angle"        { $$ = MAKE(float, @$, $1);  }
| duration       { $$ = MAKE(float, @$, $1);  }
| string         { $$ = MAKE(string, @$, $1); }
| "[" exps "]"   { $$ = MAKE(list, @$, $2); }
| dictionary     { $$ = $1; }
| tuple          { $$ = MAKE(tuple, @$, $1); }
;

%token <std::string> STRING "string";
%type <std::string> string;
string:
  "string"        { std::swap($$, $1);  }
| string "string" { std::swap($$, $1); $$ += $2; }
;

/*------------.
| Locations.  |
`------------*/

exp:
  "__HERE__"  { $$ = MAKE(position, @$); }
;

/*---------.
| Events.  |
`---------*/

%token QUEST_MARK "?";
%type <ast::EventMatch> event_match;
event_match:
  exp "?" args.opt tilda.opt guard.opt
  {
    $$ = MAKE(event_match, @$, $[exp], $[args.opt], $[tilda.opt], $[guard.opt]);
  }
//<no-space< ? event.
| "?" exp guard.opt
  {
    up.deprecated(@$, "?event(arg, ...)", "event?(arg, ...)");
    $$ = MAKE(event_match, @$, $[exp], $[guard.opt]);
  }
//>no-space>
;

%type <ast::rExp> guard.opt;
guard.opt:
  /* nothing */  { $$ = 0; }
| "if" exp       { std::swap($$, $2); }
;

%type<ast::rExp> tilda.opt;
tilda.opt:
  /* nothing */ { $$ = 0; }
| "~" exp       { std::swap($$, $2); }
;


/*---------------------------.
| Square brackets operator.  |
`---------------------------*/

lvalue:
  exp "[" exps "]"
  {
    $$ = new ast::Subscript(@$, $3, $1);
  }
;


/*--------------------.
| Special variables.  |
`--------------------*/

%token  CALL         "call"
        THIS         "this"
;

exp:
  "this"         { $$ = new ast::This(@$); }
| "call"         { $$ = new ast::CallMsg(@$); }
;

/*---------------------.
| Numeric operations.  |
`---------------------*/

// The name of the operators are the name of the messages.
%token <libport::Symbol>
        BANG       "!"
        BITAND     "bitand"
        BITOR      "bitor"
        CARET      "^"
        COMPL      "compl"
        GT_GT      ">>"
        LT_LT      "<<"
        MINUS      "-"
        PERCENT    "%"
        PLUS       "+"
        SLASH      "/"
        STAR       "*"
        STAR_STAR  "**"
;

exp:
  exp "+" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "-" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "*" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "**" exp            { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "/" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "%" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "^" exp             { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "<<" exp            { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "bitand" exp        { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "bitor" exp         { $$ = MAKE(call, @$, $1, $2, $3); }
| exp ">>" exp            { $$ = MAKE(call, @$, $1, $2, $3); }
| "+" exp    %prec UNARY  { $$ = MAKE(call, @$, $2, $1); }
| "-" exp    %prec UNARY  { $$ = MAKE(call, @$, $2, $1); }
| "!" exp                 { $$ = MAKE(call, @$, $2, $1); }
| "compl" exp             { $$ = MAKE(call, @$, $2, $1); }
| "(" exp ")"             { std::swap($$, $2); }
;

/*--------.
| Tests.  |
`--------*/
%token <libport::Symbol>
        EQ_TILDA_EQ   "=~="
        EQ_EQ         "=="
        EQ_EQ_EQ      "==="
        GT_EQ         ">="
        GT            ">"
        LT_EQ         "<="
        LT            "<"
        BANG_EQ       "!="
        BANG_EQ_EQ    "!=="
        TILDA_EQ      "~="

        AMPERSAND_AMPERSAND  "&&"
        PIPE_PIPE            "||"
;

exp:
  exp "!="  exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "!==" exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "<"   exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "<="  exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "=="  exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "===" exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "=~=" exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp ">"   exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp ">="  exp { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "~="  exp { $$ = MAKE(call, @$, $1, $2, $3); }

| exp "&&" exp  { $$ = MAKE(call, @$, $1, $2, $3); }
| exp "||" exp  { $$ = MAKE(call, @$, $1, $2, $3); }
;

// e in c => c.has(e).
exp:
  exp[m]     "in" exp[c] { $$ = MAKE(call, @$, $c, SYMBOL(has),    $m); }
| exp[m] "!" "in" exp[c] { $$ = MAKE(call, @$, $c, SYMBOL(hasNot), $m); }
;

exp.opt:
  /* nothing */ { $$ = 0; }
| exp           { std::swap($$, $1); }
;

/*---------------------.
| Desugaring internals |
`---------------------*/

%type <unsigned> unsigned;
unsigned:
  "float" { $$ = static_cast<unsigned int>($1); }
;

// This token is used to move declaration of variable made in the current
// scope to the /n/ parent scope.  This is useful when desugaring variable
// declarations.
%token PERCENT_UNSCOPE_COLON "%unscope:";
exp:
  "%unscope:" unsigned
  {
    $$ = new ast::Unscope(@$, $2);
  }
;

/*----------------.
| Metavariables.  |
`----------------*/

%token PERCENT_EXP_COLON "%exp:";
exp:
  "%exp:" unsigned
  {
    $$ = new ast::MetaExp(@$, $2);
  }
;

%token PERCENT_LVALUE_COLON "%lvalue:";
lvalue:
  "%lvalue:" unsigned
  {
    $$ = new ast::MetaLValue(@$, new ast::exps_type(),
                             $2);
  }
;

%token PERCENT_ID_COLON "%id:";
lvalue:
  "%id:" unsigned
  {
    $$ = new ast::MetaId(@$, 0, $2);
  }
| exp "." "%id:" unsigned
  {
    $$ = new ast::MetaCall(@$, 0, $1, $4);
  }
;

%token PERCENT_EXPS_COLON "%exps:";
exp:
  lvalue "(" "%exps:" unsigned ")"
  {
    assert($1.unsafe_cast<ast::LValueArgs>());
    assert(!$1.unsafe_cast<ast::LValueArgs>()->arguments_get());
    $$ = new ast::MetaArgs(@$, $1, $4);
  }
;


/*--------------.
| Expressions.  |
`--------------*/

%type <ast::exps_type*> claims claims.1 exps exps.1 exps.2 args args.opt;

// claims: a list of "exp"s separated/terminated with semicolons.
claims:
  /* empty */   { $$ = new ast::exps_type; }
| claims.1      { std::swap($$, $1); }
| claims.1 ";"  { std::swap($$, $1); }
;

claims.1:
  exp               { $$ = new ast::exps_type(1, $1); }
| claims.1 ";" exp  { std::swap($$, $1); *$$ << $3; }
;


// exps: a list of "exp"s separated/terminated with colons.
exps:
  /* empty */ { $$ = new ast::exps_type; }
| exps.1      { std::swap($$, $1); }
| exps.1 ","  { std::swap($$, $1); }
;

exps.1:
  exp             { $$ = new ast::exps_type (1, $1); }
| exps.1 "," exp  { std::swap($$, $1); *$$ << $3; }
;

exps.2:
  exps.1 "," exp  { std::swap($$, $1); *$$ << $3; }
;

// Effective arguments: 0 or more arguments in parens, or nothing.
args:
  "(" exps ")"  { std::swap($$, $2); }
;

args.opt:
  /* empty */  { $$ = 0; }
| args         { std::swap($$, $1); }
;


/*----------------------.
| List of identifiers.  |
`----------------------*/

// "var"?
var.opt:
  /* empty. */
| "var"
;

%type <::ast::Factory::formal_type> formal;
formal:
  var.opt "identifier"          { $$ = ::ast::Factory::formal_type($2, 0);  }
| var.opt "identifier" "=" exp  { $$ = ::ast::Factory::formal_type($2, $4); }
;

// One or several comma-separated identifiers.
%type <::ast::Factory::formals_type*> formals.0 formals.1 formals;
formals.1:
  formal                 { $$ = new ::ast::Factory::formals_type(1, $1); }
| formals.1 "," formal   { std::swap($$, $1); *$$ << $3; }
;

// Zero or several comma-separated identifiers.
formals.0:
  /* empty */    { $$ = new ::ast::Factory::formals_type; }
| formals.1      { std::swap($$, $1); }
| formals.1 ","  { std::swap($$, $1); }
;

// Function formal arguments.
formals:
  /* empty */         { $$ = 0; }
| "(" formals.0 ")"   { std::swap($$, $2); }
;

%%

// The error function that 'bison' calls.
void
yy::parser::error(const location_type& l, const std::string& m)
{
  up.error(l, m);
}

// Local Variables:
// mode: c++
// End:
