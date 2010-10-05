# C++ skeleton for Bison

# Copyright (C) 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009,
# 2010 Free Software Foundation, Inc.

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

m4_include(b4_pkgdatadir/[c++.m4])


# b4_integral_parser_table_declare(TABLE-NAME, CONTENT, COMMENT)
# --------------------------------------------------------------
# Declare "parser::yy<TABLE-NAME>_" which contents is CONTENT.
m4_define([b4_integral_parser_table_declare],
[m4_ifval([$3], [b4_c_comment([$3], [  ])
])dnl
  static const b4_int_type_for([$2]) yy$1_[[]];dnl
])

# b4_integral_parser_table_define(TABLE-NAME, CONTENT, COMMENT)
# -------------------------------------------------------------
# Define "parser::yy<TABLE-NAME>_" which contents is CONTENT.
m4_define([b4_integral_parser_table_define],
[  const b4_int_type_for([$2])
  b4_parser_class_name::yy$1_[[]] =
  {
  $2
  };dnl
])


# b4_symbol_value_template(VAL, [TYPE])
# -------------------------------------
# Same as b4_symbol_value, but used in a template method.  It makes
# a difference when using variants.
m4_copy([b4_symbol_value], [b4_symbol_value_template])


# b4_lhs_value([TYPE])
# --------------------
# Expansion of $<TYPE>$.
m4_define([b4_lhs_value],
          [b4_symbol_value([yylhs.value], [$1])])


# b4_lhs_location()
# -----------------
# Expansion of @$.
m4_define([b4_lhs_location],
          [yylhs.location])


# b4_rhs_data(RULE-LENGTH, NUM)
# -----------------------------
# Return the data corresponding to the symbol #NUM, where the current
# rule has RULE-LENGTH symbols on RHS.
m4_define([b4_rhs_data],
          [yystack_@{b4_subtract($@)@}])


# b4_rhs_state(RULE-LENGTH, NUM)
# ------------------------------
# The state corresponding to the symbol #NUM, where the current
# rule has RULE-LENGTH symbols on RHS.
m4_define([b4_rhs_state],
          [b4_rhs_data([$1], [$2]).state])


# b4_rhs_value(RULE-LENGTH, NUM, [TYPE])
# --------------------------------------
# Expansion of $<TYPE>NUM, where the current rule has RULE-LENGTH
# symbols on RHS.
m4_define([b4_rhs_value],
          [b4_symbol_value([b4_rhs_data([$1], [$2]).value], [$3])])


# b4_rhs_location(RULE-LENGTH, NUM)
# ---------------------------------
# Expansion of @NUM, where the current rule has RULE-LENGTH symbols
# on RHS.
m4_define([b4_rhs_location],
          [b4_rhs_data([$1], [$2]).location])


# b4_symbol_action(SYMBOL-NUM, KIND)
# ----------------------------------
# Run the action KIND (destructor or printer) for SYMBOL-NUM.
# Same as in C, but using references instead of pointers.
m4_define([b4_symbol_action],
[b4_symbol_if([$1], [has_$2],
[m4_pushdef([b4_dollar_dollar],
    [b4_symbol_value_template([yysym.value],
                              b4_symbol_if([$1], [has_type],
                                           [b4_symbol([$1], [type])]))])dnl
m4_pushdef([b4_at_dollar], [yysym.location])dnl
      b4_symbol_case_([$1])
b4_syncline([b4_symbol([$1], [$2_line])], ["b4_symbol([$1], [$2_file])"])
        b4_symbol([$1], [$2])
b4_syncline([@oline@], [@ofile@])
        break;

m4_popdef([b4_at_dollar])dnl
m4_popdef([b4_dollar_dollar])dnl
])])


m4_pushdef([b4_copyright_years],
           [2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010])

m4_define([b4_parser_class_name],
          [b4_percent_define_get([[parser_class_name]])])

# The header is mandatory.
b4_defines_if([],
              [b4_fatal([b4_skeleton[: using %%defines is mandatory]])])

b4_locations_if([b4_percent_define_ifdef([[location_type]], [],
  [# Backward compatibility.
  m4_define([b4_location_constructors])
  m4_include(b4_pkgdatadir/[location.cc])])])
m4_include(b4_pkgdatadir/[stack.hh])
b4_variant_if([m4_include(b4_pkgdatadir/[variant.hh])])

# We do want M4 expansion after # for CPP macros.
m4_changecom()
m4_divert_push(0)dnl
@output(b4_spec_defines_file@)@
b4_copyright([Skeleton interface for Bison LALR(1) parsers in C++])
dnl FIXME: This is wrong, we want computed header guards.
[
/* C++ LALR(1) parser skeleton written by Akim Demaille.  */

#ifndef PARSER_HEADER_H
# define PARSER_HEADER_H

]b4_percent_code_get([[requires]])[

]b4_parse_assert_if([#include <cassert>])[
#include <stdexcept>
#include <string>
#include <iostream>
#include "stack.hh"
]b4_locations_if([b4_percent_define_ifdef([[location_type]], [],
                                          [[#include "location.hh"]])])[

]b4_variant_if([b4_namespace_open
b4_variant_define
b4_namespace_close])[

/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG ]b4_parse_trace_if([1], [0])[
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE ]b4_error_verbose_if([1], [0])[
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE ]b4_token_table[
#endif

]b4_namespace_open[

  /// A Bison parser.
  class ]b4_parser_class_name[
  {
  public:
]b4_public_types_declare[
    /// Build a parser object.
    ]b4_parser_class_name[ (]b4_parse_param_decl[);
    virtual ~]b4_parser_class_name[ ();

    /// Parse.
    /// \returns  0 iff parsing succeeded.
    virtual int parse ();

#if YYDEBUG
    /// The current debugging stream.
    std::ostream& debug_stream () const;
    /// Set the current debugging stream.
    void set_debug_stream (std::ostream &);

    /// Type for debugging levels.
    typedef int debug_level_type;
    /// The current debugging level.
    debug_level_type debug_level () const;
    /// Set the current debugging level.
    void set_debug_level (debug_level_type l);
#endif

    /// Report a syntax error.]b4_locations_if([
    /// \param loc    where the syntax error is found.])[
    /// \param msg    a description of the syntax error.
    virtual void error (]b4_locations_if([const location_type& loc, ])[const std::string& msg);

  private:
    /// State numbers.
    typedef int state_type;

    /// Generate an error message.
    /// \param yystate   the state where the error occurred.
    /// \param yytoken   the lookahead token.
    virtual std::string yysyntax_error_ (state_type yystate, int yytoken);

    /// Compute post-reduction state.
    /// \param yystate   the current state
    /// \param yylhs     the nonterminal to push on the stack
    state_type yy_lr_goto_state_ (state_type yystate, int yylhs);

    /// Whether the given \c yypact_ value indicates a defaulted state.
    /// \param yyvalue   the value to check
    static bool yy_pact_value_is_default_ (int yyvalue);

    /// Whether the given \c yytable_ value indicates a syntax error.
    /// \param yyvalue   the value to check
    static bool yy_table_value_is_error_ (int yyvalue);

    /// Internal symbol numbers.
    typedef ]b4_int_type_for([b4_translate])[ token_number_type;
    static const ]b4_int_type(b4_pact_ninf, b4_pact_ninf)[ yypact_ninf_;
    static const ]b4_int_type(b4_table_ninf, b4_table_ninf)[ yytable_ninf_;

    /* Tables.  */
]b4_parser_tables_declare[

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
    /// For a symbol, its name in clear.
    static const char* const yytname_[];
#endif]b4_error_verbose_if([

    /// Convert the symbol name \a n to a form suitable for a diagnostic.
    static std::string yytnamerr_ (const char *n);])[

#if YYDEBUG
]b4_integral_parser_table_declare([rline], [b4_rline],
     [YYRLINE[YYN] -- Source line where rule number YYN was defined.])[
    /// Report on the debug stream that the rule \a r is going to be reduced.
    virtual void yy_reduce_print_ (int r);
    /// Print the state stack on the debug stream.
    virtual void yystack_print_ ();

    /* Debugging.  */
    int yydebug_;
    std::ostream* yycdebug_;
#endif

    /// Convert a scanner token number \a t to a symbol number.
    static inline token_number_type yytranslate_ (]b4_lex_symbol_if([token_type], [int])[ t);

#if YYDEBUG
    /// \brief Display a symbol type, value and location.
    /// \param yyo    The output stream.
    /// \param yysym  The symbol.
    template <typename Exact>
    void yy_print_ (std::ostream& yyo,
                    const symbol_base_type<Exact>& yysym) const;
#endif

    /// \brief Reclaim the memory associated to a symbol.
    /// \param yymsg     Why this token is reclaimed.
    ///                  If null, print nothing.
    /// \param s         The symbol.
    template <typename Exact>
    inline void yy_destroy_ (const char* yymsg,
                             symbol_base_type<Exact>& yysym) const;

  private:
    /// Element of the stack: a state and its attributes.
    struct stack_symbol_type : symbol_base_type<stack_symbol_type>
    {
      /// The parent class.
      typedef symbol_base_type<stack_symbol_type> super_type;

      /// Default constructor.
      inline stack_symbol_type ();

      /// Constructor.
      inline stack_symbol_type (]b4_args([state_type s],
                                         [const semantic_type& v],
                                         b4_locations_if([const location_type& l]))[);

      /// The state.
      state_type state;

      /// The type (corresponding to \a state).
      inline int type_get_ () const;
    };

    /// Stack type.
    typedef stack<stack_symbol_type> stack_type;

    /// The stack.
    stack_type yystack_;

    /// Push a new state on the stack.
    /// \param m    a debug message to display
    ///             if null, no trace is output.
    /// \param s    the symbol
    /// \warning the contents of \a s.value is stolen.
    inline void yypush_ (const char* m, stack_symbol_type& s);

    /// Push a new look ahead token on the state on the stack.
    /// \param m    a debug message to display
    ///             if null, no trace is output.
    /// \param s    the state
    /// \param sym  the symbol (for its value and location).
    /// \warning the contents of \a s.value is stolen.
    inline void yypush_ (const char* m, state_type s, symbol_type& sym);

    /// Pop \a n symbols the three stacks.
    inline void yypop_ (unsigned int n = 1);

    /* Constants.  */
    enum
    {
      yyeof_ = 0,
      yylast_ = ]b4_last[,           //< Last index in yytable_.
      yynnts_ = ]b4_nterms_number[,  //< Number of nonterminal symbols.
      yyempty_ = -2,
      yyfinal_ = ]b4_final_state_number[, //< Termination state number.
      yyterror_ = 1,
      yyerrcode_ = 256,
      yyntokens_ = ]b4_tokens_number[,   //< Number of tokens.
    };

]b4_parse_param_vars[
  };

]b4_lex_symbol_if([b4_yytranslate_define
b4_public_types_define])[
]b4_namespace_close[

]b4_percent_define_flag_if([[global_tokens_and_yystype]],
[b4_token_defines(b4_tokens)

#ifndef YYSTYPE
 /* Redirection for backward compatibility.  */
# define YYSTYPE b4_namespace_ref::b4_parser_class_name::semantic_type
#endif
])
b4_percent_code_get([[provides]])[]dnl

[#endif /* ! defined PARSER_HEADER_H */]
@output(b4_parser_file_name@)@
b4_copyright([Skeleton implementation for Bison LALR(1) parsers in C++])
b4_percent_code_get([[top]])[]dnl
m4_if(b4_prefix, [yy], [],
[
// Take the name prefix into account.
#define yylex   b4_prefix[]lex])[

/* First part of user declarations.  */
]b4_user_pre_prologue[

#include "@basename(]b4_spec_defines_file[@)"

/* User implementation prologue.  */
]b4_user_post_prologue
b4_percent_code_get[]dnl

[#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* FIXME: INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

]b4_locations_if([dnl
[/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K].location)
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)                               \
 do                                                                    \
   if (N)                                                              \
     {                                                                 \
       (Current).begin = YYRHSLOC (Rhs, 1).begin;                      \
       (Current).end   = YYRHSLOC (Rhs, N).end;                        \
     }                                                                 \
   else                                                                \
     {                                                                 \
       (Current).begin = (Current).end = YYRHSLOC (Rhs, 0).end;        \
     }                                                                 \
 while (false)
#endif]])[

/* Suppress unused-variable warnings by "using" E.  */
#define YYUSE(e) ((void) (e))

/* Enable debugging if requested.  */
#if YYDEBUG

/* A pseudo ostream that takes yydebug_ into account.  */
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Symbol)         \
  do {                                          \
    if (yydebug_)                               \
    {                                           \
      *yycdebug_ << Title << ' ';               \
      yy_print_ (*yycdebug_, Symbol);           \
      *yycdebug_ << std::endl;                  \
    }                                           \
  } while (false)

# define YY_REDUCE_PRINT(Rule)		\
  do {					\
    if (yydebug_)                       \
      yy_reduce_print_ (Rule);		\
  } while (false)

# define YY_STACK_PRINT()		\
  do {					\
    if (yydebug_)                       \
      yystack_print_ ();                \
  } while (false)

#else /* !YYDEBUG */

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Symbol)  static_cast<void>(0)
# define YY_REDUCE_PRINT(Rule)           static_cast<void>(0)
# define YY_STACK_PRINT()                static_cast<void>(0)

#endif /* !YYDEBUG */

#define yyerrok         (yyerrstatus_ = 0)
#define yyclearin       (yyempty = true)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)

]b4_namespace_open[]b4_error_verbose_if([[

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
  ]b4_parser_class_name[::yytnamerr_ (const char *yystr)
  {
    if (*yystr == '"')
      {
        std::string yyr = "";
        char const *yyp = yystr;

        for (;;)
          switch (*++yyp)
            {
            case '\'':
            case ',':
              goto do_not_strip_quotes;

            case '\\':
              if (*++yyp != '\\')
                goto do_not_strip_quotes;
              /* Fall through.  */
            default:
              yyr += *yyp;
              break;

            case '"':
              return yyr;
            }
      do_not_strip_quotes: ;
      }

    return yystr;
  }
]])[

  /// Build a parser object.
  ]b4_parser_class_name::b4_parser_class_name[ (]b4_parse_param_decl[)]m4_ifset([b4_parse_param], [
    :])[
#if YYDEBUG
    ]m4_ifset([b4_parse_param], [  ], [ :])[yydebug_ (false),
      yycdebug_ (&std::cerr)]m4_ifset([b4_parse_param], [,])[
#endif]b4_parse_param_cons[
  {
  }

  ]b4_parser_class_name::~b4_parser_class_name[ ()
  {
  }


  /*---------------.
  | Symbol types.  |
  `---------------*/

]b4_lex_symbol_if([], [b4_public_types_define])[

  // stack_symbol_type.
  ]b4_parser_class_name[::stack_symbol_type::stack_symbol_type ()
    : super_type ()
    , state ()
  {
  }

  ]b4_parser_class_name[::stack_symbol_type::stack_symbol_type (]b4_args(
                 [state_type s],
                 [const semantic_type& v],
                 b4_locations_if([const location_type& l]))[)
    : super_type (v]b4_locations_if([, l])[)
    , state (s)
  {
  }

  int
  ]b4_parser_class_name[::stack_symbol_type::type_get_ () const
  {
    return yystos_[state];
  }


  template <typename Exact>
  void
  ]b4_parser_class_name[::yy_destroy_ (const char* yymsg,
                                       symbol_base_type<Exact>& yysym) const
  {
    int yytype = yysym.type_get ();
    YYUSE (yymsg);
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);

    // User destructor.
    switch (yytype)
      {
]b4_symbol_foreach([b4_symbol_destructor])dnl
[       default:
          break;
      }]b4_variant_if([

    // Type destructor.
  b4_symbol_variant([[yytype]], [[yysym.value]], [[template destroy]])])[
  }

#if YYDEBUG
  template <typename Exact>
  void
  ]b4_parser_class_name[::yy_print_ (std::ostream& yyo,
                                     const symbol_base_type<Exact>& yysym) const
  {
    int yytype = yysym.type_get ();
    yyo << (yytype < yyntokens_ ? "token" : "nterm")
        << ' ' << yytname_[yytype] << " ("]b4_locations_if([
        << yysym.location << ": "])[;
    switch (yytype)
      {
]b4_symbol_foreach([b4_symbol_printer])dnl
[       default:
	  break;
      }
    yyo << ')';
  }
#endif

  void
  ]b4_parser_class_name[::yypush_ (const char* m, state_type s,
                                   symbol_type& sym)
  {
    if (m)
      YY_SYMBOL_PRINT (m, sym);
]b4_variant_if(
[[    yystack_.push (stack_symbol_type (]b4_args(
                    [s],
                    [semantic_type()],
                    b4_locations_if([sym.location]))[));
    ]b4_symbol_variant([[yystos_[s]]], [[yystack_[0].value]],
                       [build], [sym.value])],
[[    yystack_.push (stack_symbol_type (]b4_args(
                      [s],
                      [sym.value],
                      b4_locations_if([sym.location]))[));]])[
  }

  void
  ]b4_parser_class_name[::yypush_ (const char* m, stack_symbol_type& s)
  {
    if (m)
      YY_SYMBOL_PRINT (m, s);
]b4_variant_if(
[[    yystack_.push (stack_symbol_type (]b4_args(
                       [s.state],
                       [semantic_type()],
                       b4_locations_if([s.location]))[));
    ]b4_symbol_variant([[yystos_[s.state]]], [[yystack_[0].value]],
                       [build], [s.value])],
[    yystack_.push (s);])[
  }

  void
  ]b4_parser_class_name[::yypop_ (unsigned int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
  ]b4_parser_class_name[::debug_stream () const
  {
    return *yycdebug_;
  }

  void
  ]b4_parser_class_name[::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


  ]b4_parser_class_name[::debug_level_type
  ]b4_parser_class_name[::debug_level () const
  {
    return yydebug_;
  }

  void
  ]b4_parser_class_name[::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif

  inline ]b4_parser_class_name[::state_type
  ]b4_parser_class_name[::yy_lr_goto_state_ (state_type yystate, int yylhs)
  {
    int yyr = yypgoto_[yylhs - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yylhs - yyntokens_];
  }

  inline bool
  ]b4_parser_class_name[::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  inline bool
  ]b4_parser_class_name[::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
  ]b4_parser_class_name[::parse ()
  {
    /// Whether yyla contains a lookahead.
    bool yyempty = true;

    /* State.  */
    int yyn;
    int yylen = 0;

    /* Error handling.  */
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// The lookahead symbol.
    symbol_type yyla;]b4_locations_if([[

    /// The locations where the error started and ended.
    stack_symbol_type yyerror_range[3];]])[

    /// $$ and @@$.
    stack_symbol_type yylhs;

    /// The return value of parse().
    int yyresult;

    YYCDEBUG << "Starting parse" << std::endl;

]m4_ifdef([b4_initial_action], [
m4_pushdef([b4_at_dollar],     [yyla.location])dnl
m4_pushdef([b4_dollar_dollar], [yyla.value])dnl
    /* User initialization code.  */
    b4_user_initial_action
m4_popdef([b4_dollar_dollar])dnl
m4_popdef([b4_at_dollar])])dnl

  [  /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_ = stack_type (0);
    yypush_ (0, 0, yyla);

    // A new symbol was pushed on the stack.
  yynewstate:
    YYCDEBUG << "Entering state " << yystack_[0].state << std::endl;

    /* Accept?  */
    if (yystack_[0].state == yyfinal_)
      goto yyacceptlab;

    goto yybackup;

    /* Backup.  */
  yybackup:

    /* Try to take a decision without lookahead.  */
    yyn = yypact_[yystack_[0].state];
    if (yy_pact_value_is_default_ (yyn))
      goto yydefault;

    /* Read a lookahead token.  */
    if (yyempty)
      {
        YYCDEBUG << "Reading a token: ";
]b4_lex_symbol_if(
[        yyla = b4_c_function_call([yylex], [symbol_type],
                                   m4_ifdef([b4_lex_param], b4_lex_param));],
[        yyla.type = yytranslate_ (b4_c_function_call([yylex], [int],
				     [[YYSTYPE*], [&yyla.value]][]dnl
b4_locations_if([, [[location*], [&yyla.location]]])dnl
m4_ifdef([b4_lex_param], [, ]b4_lex_param)));])[
        yyempty = false;
      }
    YY_SYMBOL_PRINT ("Next token is", yyla);

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.type;
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.type)
      goto yydefault;

    /* Reduce or error.  */
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
	if (yy_table_value_is_error_ (yyn))
	  goto yyerrlab;
	yyn = -yyn;
	goto yyreduce;
      }

    /* Discard the token being shifted.  */
    yyempty = true;

    /* Count tokens shifted since error; after three, turn off error
       status.  */
    if (yyerrstatus_)
      --yyerrstatus_;

    /* Shift the lookahead token.  */
    yypush_ ("Shifting", yyn, yyla);
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[yystack_[0].state];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- Do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    yylhs.state = yy_lr_goto_state_(yystack_[yylen].state, yyr1_[yyn]);]b4_variant_if([
    /* Variants are always initialized to an empty instance of the
       correct type. The default $$=$1 action is NOT applied when using
       variants.  */
    b4_symbol_variant([[yyr1_@{yyn@}]], [yylhs.value], [build])],[
    /* If YYLEN is nonzero, implement the default value of the action:
       `$$ = $1'.  Otherwise, use the top of the stack.

       Otherwise, the following line sets YYLHS.VALUE to garbage.
       This behavior is undocumented and Bison
       users should not rely upon it.  */
    if (yylen)
      yylhs.value = yystack_@{yylen - 1@}.value;
    else
      yylhs.value = yystack_@{0@}.value;])[
]b4_locations_if([dnl
[
    // Compute the default @@$.
    {
      slice<stack_symbol_type, stack_type> slice (yystack_, yylen);
      YYLLOC_DEFAULT (yylhs.location, slice, yylen);
    }]])[

    // Perform the reduction.
    YY_REDUCE_PRINT (yyn);
    try
    {
      switch (yyn)
      {
]b4_user_actions[
	default:
          break;
      }
    }
    catch (const syntax_error& yyexc)
    {
      error (]b4_args(b4_locations_if([yyexc.location]),
                      [[yyexc.what()]])[);
      YYERROR;
    }
    YY_SYMBOL_PRINT ("-> $$ =", yylhs);
]b4_variant_if([[
    // Destroy the rhs symbols.
    for (int i = 0; i < yylen; ++i)
      // Destroy a variant which value may have been swapped with
      // yylhs.value (for instance if the action was "std::swap($$,
      // $1)").  The value of yylhs.value (hence possibly one of these
      // rhs symbols) depends on the default contruction for this
      // type.  In the case of pointers for instance, no
      // initialization is done, so the value is junk.  Therefore do
      // not try to report the value of symbols about to be destroyed
      // in the debug trace, it's possibly junk.  Hence yymsg = 0.
      // Besides, that keeps exactly the same traces as with the other
      // Bison skeletons.
      yy_destroy_ (0, yystack_[i]);]])[

    yypop_ (yylen);
    yylen = 0;
    YY_STACK_PRINT ();

    /* Shift the result of the reduction.  */
    yypush_ (0, yylhs);
    goto yynewstate;

  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    /* If not already recovering from an error, report this error.  */
    if (!yyerrstatus_)
      {
	++yynerrs_;
	error (]b4_args(b4_locations_if([yyla.location]),
                        [[yysyntax_error_ (yystack_[0].state, yyla.type)]])[);
      }

]b4_locations_if([[
    yyerror_range[1].location = yyla.location;]])[
    if (yyerrstatus_ == 3)
      {
	/* If just tried and failed to reuse lookahead token after an
           error, discard it.  */

        /* Return failure if at end of input.  */
        if (yyla.type == yyeof_)
          YYABORT;
        else if (!yyempty)
          {
            yy_destroy_ ("Error: discarding", yyla);
            yyempty = true;
	  }
      }

    /* Else will try to reuse lookahead token after shifting the error
       token.  */
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
       YYERROR and the label yyerrorlab therefore never appears in user
       code.  */
    if (false)
      goto yyerrorlab;]b4_locations_if([[
    yyerror_range[1].location = yystack_[yylen - 1].location;]])b4_variant_if([[
    /* $$ was initialized before running the user action.  */
    yy_destroy_ ("Error: discarding", yylhs);]])[
    /* Do not reclaim the symbols of the rule which action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;	/* Each real token shifted decrements this.  */
    {
      stack_symbol_type error_token;
      for (;;)
        {
          yyn = yypact_[yystack_[0].state];
          if (!yy_pact_value_is_default_ (yyn))
            {
              yyn += yyterror_;
              if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
                {
                  yyn = yytable_[yyn];
                  if (0 < yyn)
                    break;
                }
            }

          // Pop the current state because it cannot handle the error token.
          if (yystack_.size () == 1)
            YYABORT;
]b4_locations_if([[
          yyerror_range[1].location = yystack_[0].location;]])[
          yy_destroy_ ("Error: popping", yystack_[0]);
          yypop_ ();
          YY_STACK_PRINT ();
        }
]b4_locations_if([[
      yyerror_range[2].location = yyla.location;
      YYLLOC_DEFAULT (error_token.location, yyerror_range, 2);]])[

      /* Shift the error token.  */
      error_token.state = yyn;
      yypush_ ("Shifting", error_token);
    }
    goto yynewstate;

    /* Accept.  */
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    /* Abort.  */
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  yyreturn:
    if (!yyempty)
      yy_destroy_ ("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule which action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (yystack_.size () != 1)
      {
	yy_destroy_ ("Cleanup: popping", yystack_[0]);
	yypop_ ();
      }

    return yyresult;
  }

  // Generate an error message.
  std::string
  ]b4_parser_class_name[::yysyntax_error_ (]dnl
b4_error_verbose_if([state_type yystate, int yytoken],
                    [int, int])[)
  {
    std::string yyres;]b4_error_verbose_if([[
    int yyn = yypact_[yystate];
    if (yypact_ninf_ < yyn && yyn <= yylast_)
      {
	/* Start YYX at -YYN if negative to avoid negative indexes in
	   YYCHECK.  In other words, skip the first -YYN actions for this
	   state because they are default actions.  */
	int yyxbegin = yyn < 0 ? -yyn : 0;

	/* Stay within bounds of both yycheck and yytname.  */
	int yychecklim = yylast_ - yyn + 1;
	int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;

        // Number of reported tokens (one for the "unexpected", one per
        // "expected").
	size_t yycount = 0;
        // Its maximum.
        enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
        // Arguments of yyformat.
        char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
        yyarg[yycount++] = yytname_[yytoken];
	for (int yyx = yyxbegin; yyx < yyxend; ++yyx)
	  if (yycheck_[yyx + yyn] == yyx && yyx != yyterror_
	      && !yy_table_value_is_error_ (yytable_[yyx + yyn]))
          {
            if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
            {
              yycount = 1;
              break;
            }
            else
              yyarg[yycount++] = yytname_[yyx];
          }

        char const* yyformat = 0;
        switch (yycount)
        {
#define YYCASE_(N, S)                           \
          case N:                               \
            yyformat = S;                       \
          break
          YYCASE_(1, YY_("syntax error, unexpected %s"));
          YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
          YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
          YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
          YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
#undef YYCASE_
        }
        // Argument number.
        size_t yyi = 0;
        for (char const* yyp = yyformat; *yyp; ++yyp)
          if (yyp[0] == '%' && yyp[1] == 's' && yyi < yycount)
          {
            yyres += yytnamerr_ (yyarg[yyi++]);
            ++yyp;
          }
          else
            yyres += *yyp;
      }
    else
  ]])dnl
[    yyres = YY_("syntax error");
    return yyres;
  }


  const ]b4_int_type(b4_pact_ninf, b4_pact_ninf) b4_parser_class_name::yypact_ninf_ = b4_pact_ninf[;

  const ]b4_int_type(b4_table_ninf, b4_table_ninf) b4_parser_class_name::yytable_ninf_ = b4_table_ninf[;

]b4_parser_tables_define[

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
  /* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
     First, the terminals, then, starting at \a yyntokens_, nonterminals.  */
  const char*
  const ]b4_parser_class_name[::yytname_[] =
  {
  ]b4_tname[
  };
#endif

#if YYDEBUG
]b4_integral_parser_table_define([rline], [b4_rline])[

  // Print the state stack on the debug stream.
  void
  ]b4_parser_class_name[::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (stack_type::const_iterator
           i = yystack_.begin (),
           i_end = yystack_.end ();
	 i != i_end; ++i)
      *yycdebug_ << ' ' << i->state;
    *yycdebug_ << std::endl;
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
  ]b4_parser_class_name[::yy_reduce_print_ (int yyrule)
  {
    unsigned int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    /* Print the symbols being reduced, and their result.  */
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
	       << " (line " << yylno << "):" << std::endl;
    /* The symbols being reduced.  */
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
                       ]b4_rhs_data(yynrhs, yyi + 1)[);
  }
#endif // YYDEBUG

]b4_lex_symbol_if([], [b4_yytranslate_define])[
]b4_namespace_close[
]b4_epilogue[]dnl
m4_divert_pop(0)
m4_popdef([b4_copyright_years])dnl
