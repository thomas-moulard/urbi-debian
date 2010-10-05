#line 9216 "../../doc/bison.texinfo"
%skeleton "lalr1.cc"                          /*  -*- C++ -*- */
%require "2.3b.563-48ca"
%defines
%define parser_class_name "calcxx_parser"
#line 9232 "../../doc/bison.texinfo"
%define variant
%define parse.assert
%define lex_symbol
#line 9249 "../../doc/bison.texinfo"
%code requires
{
# include <string>
class calcxx_driver;
}
#line 9263 "../../doc/bison.texinfo"
// The parsing context.
%param { calcxx_driver& driver }
#line 9275 "../../doc/bison.texinfo"
%locations
%initial-action
{
  // Initialize the initial location.
  @$.begin.filename = @$.end.filename = &driver.file;
};
#line 9289 "../../doc/bison.texinfo"
%define parse.trace
%define parse.error verbose
#line 9300 "../../doc/bison.texinfo"
%code
{
# include "calc++-driver.hh"
}
#line 9316 "../../doc/bison.texinfo"
%define api.tokens.prefix "TOK_"
%token
  END  0  "end of file"
  ASSIGN  ":="
  MINUS   "-"
  PLUS    "+"
  STAR    "*"
  SLASH   "/"
  LPAREN  "("
  RPAREN  ")"
;
#line 9336 "../../doc/bison.texinfo"
%token <std::string> IDENTIFIER "identifier"
%token <int> NUMBER "number"
%type  <int> exp
#line 9350 "../../doc/bison.texinfo"
%printer { debug_stream () << $$; } <*>;
#line 9359 "../../doc/bison.texinfo"
%%
%start unit;
unit: assignments exp  { driver.result = $2; };

assignments:
  assignments assignment {}
| /* Nothing.  */        {};

assignment:
  "identifier" ":=" exp { driver.variables[$1] = $3; };

%left "+" "-";
%left "*" "/";
exp:
  exp "+" exp   { $$ = $1 + $3; }
| exp "-" exp   { $$ = $1 - $3; }
| exp "*" exp   { $$ = $1 * $3; }
| exp "/" exp   { $$ = $1 / $3; }
| "(" exp ")"   { std::swap ($$, $2); }
| "identifier"  { $$ = driver.variables[$1]; }
| "number"      { std::swap ($$, $1); };
%%
#line 9389 "../../doc/bison.texinfo"
void
yy::calcxx_parser::error (const location_type& l,
                          const std::string& m)
{
  driver.error (l, m);
}
