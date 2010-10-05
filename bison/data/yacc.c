                                                             -*- C -*-
# Yacc compatible skeleton for Bison

# Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005,
# 2006, 2007, 2008, 2009, 2010 Free Software Foundation, Inc.

m4_pushdef([b4_copyright_years],
           [1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010])

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

# Check the value of %define api.push-pull.
b4_percent_define_default([[api.push-pull]], [[pull]])
b4_percent_define_check_values([[[[api.push-pull]],
                                 [[pull]], [[push]], [[both]]]])
b4_define_flag_if([pull]) m4_define([b4_pull_flag], [[1]])
b4_define_flag_if([push]) m4_define([b4_push_flag], [[1]])
m4_case(b4_percent_define_get([[api.push-pull]]),
        [pull], [m4_define([b4_push_flag], [[0]])],
        [push], [m4_define([b4_pull_flag], [[0]])])

# Handle BISON_USE_PUSH_FOR_PULL for the test suite.  So that push parsing
# tests function as written, don't let BISON_USE_PUSH_FOR_PULL modify Bison's
# behavior at all when push parsing is already requested.
b4_define_flag_if([use_push_for_pull])
b4_use_push_for_pull_if([
  b4_push_if([m4_define([b4_use_push_for_pull_flag], [[0]])],
             [m4_define([b4_push_flag], [[1]])])])

m4_include(b4_pkgdatadir/[c.m4])

## ---------------- ##
## Default values.  ##
## ---------------- ##

# Stack parameters.
m4_define_default([b4_stack_depth_max], [10000])
m4_define_default([b4_stack_depth_init],  [200])


## ------------------------ ##
## Pure/impure interfaces.  ##
## ------------------------ ##

b4_percent_define_default([[api.pure]], [[false]])
b4_define_flag_if([pure])
m4_define([b4_pure_flag],
          [b4_percent_define_flag_if([[api.pure]], [[1]], [[0]])])

# b4_yacc_pure_if(IF-TRUE, IF-FALSE)
# ----------------------------------
# Expand IF-TRUE, if %pure-parser and %parse-param, IF-FALSE otherwise.
m4_define([b4_yacc_pure_if],
[b4_pure_if([m4_ifset([b4_parse_param],
		      [$1], [$2])],
	    [$2])])


# b4_yyerror_args
# ---------------
# Arguments passed to yyerror: user args plus yylloc.
m4_define([b4_yyerror_args],
[b4_yacc_pure_if([b4_locations_if([&yylloc, ])])dnl
m4_ifset([b4_parse_param], [b4_c_args(b4_parse_param), ])])


# b4_lex_param
# ------------
# Accumulate in b4_lex_param all the yylex arguments.
# b4_lex_param arrives quoted twice, but we want to keep only one level.
m4_define([b4_lex_param],
m4_dquote(b4_pure_if([[[[YYSTYPE *]], [[&yylval]]][]dnl
b4_locations_if([, [[YYLTYPE *], [&yylloc]]])m4_ifdef([b4_lex_param], [, ])])dnl
m4_ifdef([b4_lex_param], b4_lex_param)))


## ------------ ##
## Data Types.  ##
## ------------ ##

# b4_int_type(MIN, MAX)
# ---------------------
# Return the smallest int type able to handle numbers ranging from
# MIN to MAX (included).  Overwrite the version from c.m4, which
# uses only C89 types, so that the user can override the shorter
# types, and so that pre-C89 compilers are handled correctly.
m4_define([b4_int_type],
[m4_if(b4_ints_in($@,      [0],   [255]), [1], [yytype_uint8],
       b4_ints_in($@,   [-128],   [127]), [1], [yytype_int8],

       b4_ints_in($@,      [0], [65535]), [1], [yytype_uint16],
       b4_ints_in($@, [-32768], [32767]), [1], [yytype_int16],

       m4_eval([0 <= $1]),                [1], [unsigned int],

					       [int])])


## ----------------- ##
## Semantic Values.  ##
## ----------------- ##


# b4_lhs_value([TYPE])
# --------------------
# Expansion of $<TYPE>$.
m4_define([b4_lhs_value],
[b4_symbol_value(yyval, [$1])])


# b4_rhs_value(RULE-LENGTH, NUM, [TYPE])
# --------------------------------------
# Expansion of $<TYPE>NUM, where the current rule has RULE-LENGTH
# symbols on RHS.
m4_define([b4_rhs_value],
          [b4_symbol_value([yyvsp@{b4_subtract([$2], [$1])@}], [$3])])



## ----------- ##
## Locations.  ##
## ----------- ##

# b4_lhs_location()
# -----------------
# Expansion of @$.
m4_define([b4_lhs_location],
[(yyloc)])


# b4_rhs_location(RULE-LENGTH, NUM)
# ---------------------------------
# Expansion of @NUM, where the current rule has RULE-LENGTH symbols
# on RHS.
m4_define([b4_rhs_location],
          [(yylsp@{b4_subtract([$2], [$1])@})])


## ------------------ ##
## Parser variables.  ##
## ------------------ ##

# b4_declare_scanner_communication_variables
# ------------------------------------------
# Declare the variables that are global, or local to YYPARSE if
# pure-parser.
m4_define([b4_declare_scanner_communication_variables], [[
/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;]b4_locations_if([[

/* Location data for the lookahead symbol.  */
YYLTYPE yylloc;]])b4_pure_if([], [[

/* Number of syntax errors so far.  */
int yynerrs;]])])


# b4_declare_parser_state_variables
# ---------------------------------
# Declare all the variables that are needed to maintain the parser state
# between calls to yypush_parse.
m4_define([b4_declare_parser_state_variables], [b4_pure_if([[
    /* Number of syntax errors so far.  */
    int yynerrs;
]])[
    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       `yyss': related to states.
       `yyvs': related to semantic values.]b4_locations_if([[
       `yyls': related to locations.]])[

       Refer to the stacks thru separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;]b4_locations_if([[

    /* The location stack.  */
    YYLTYPE yylsa[YYINITDEPTH];
    YYLTYPE *yyls;
    YYLTYPE *yylsp;

    /* The locations where the error started and ended.  */
    YYLTYPE yyerror_range[3];]])[

    YYSIZE_T yystacksize;]])


## --------------------------------------------------------- ##
## Defining symbol actions, e.g., printers and destructors.  ##
## --------------------------------------------------------- ##

# We do want M4 expansion after # for CPP macros.
m4_changecom()
m4_divert_push(0)dnl
@output(b4_parser_file_name@)@
b4_copyright([Implementation for Bison's Yacc-like parsers in C])[

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

]b4_identification
b4_percent_code_get([[top]])[]dnl
m4_if(b4_prefix, [yy], [],
[[/* Substitute the variable and function names.  */]b4_pull_if([[
#define yyparse         ]b4_prefix[parse]])b4_push_if([[
#define yypush_parse    ]b4_prefix[push_parse]b4_pull_if([[
#define yypull_parse    ]b4_prefix[pull_parse]])[
#define yypstate_new    ]b4_prefix[pstate_new
#define yypstate_delete ]b4_prefix[pstate_delete
#define yypstate        ]b4_prefix[pstate]])[
#define yylex           ]b4_prefix[lex
#define yyerror         ]b4_prefix[error
#define yylval          ]b4_prefix[lval
#define yychar          ]b4_prefix[char
#define yydebug         ]b4_prefix[debug
#define yynerrs         ]b4_prefix[nerrs
]b4_locations_if([[#define yylloc          ]b4_prefix[lloc]])])[

/* Copy the first part of user declarations.  */
]b4_user_pre_prologue[

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

]b4_percent_code_get([[requires]])[]dnl

b4_token_enums_defines(b4_tokens)[

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
]m4_ifdef([b4_stype],
[[typedef union ]b4_union_name[
{
]b4_user_stype[
} YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1]],
[m4_if(b4_tag_seen_flag, 0,
[[typedef int YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1]])])[
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif]b4_locations_if([[

#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
} YYLTYPE;
# define yyltype YYLTYPE /* obsolescent; will be withdrawn */
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif]])b4_push_if([[

#ifndef YYPUSH_DECLS
#  define YYPUSH_DECLS
struct yypstate;
typedef struct yypstate yypstate;
enum { YYPUSH_MORE = 4 };

]b4_pull_if([b4_c_function_decl([[yyparse]], [[int]], b4_parse_param)
])b4_c_function_decl([[yypush_parse]], [[int]],
  [[[yypstate *yyps]], [[yyps]]]b4_pure_if([,
  [[[int yypushed_char]], [[yypushed_char]]],
  [[[YYSTYPE const *yypushed_val]], [[yypushed_val]]]b4_locations_if([,
  [[[YYLTYPE const *yypushed_loc]], [[yypushed_loc]]]])])m4_ifset([b4_parse_param], [,
  b4_parse_param]))
b4_pull_if([b4_c_function_decl([[yypull_parse]], [[int]],
  [[[yypstate *yyps]], [[yyps]]]m4_ifset([b4_parse_param], [,
  b4_parse_param]))])
b4_c_function_decl([[yypstate_new]], [[yypstate *]], [[[void]], []])
b4_c_function_decl([[yypstate_delete]], [[void]],
                   [[[yypstate *yyps]], [[yyps]]])[
#endif]])

b4_percent_code_get([[provides]])[]dnl

[/* Copy the second part of user declarations.  */
]b4_user_post_prologue
b4_percent_code_get[]dnl

[#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif ]b4_c_modern[
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && ]b4_c_modern[
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
]b4_c_function_def([YYID], [static int], [[int yyi], [yyi]])[
{
  return yyi;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

]b4_push_if([],
[[/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && ]b4_c_modern[
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

]])dnl
[# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && ]b4_c_modern[
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && ]b4_c_modern[
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (]b4_locations_if([[defined YYLTYPE_IS_TRIVIAL && YYLTYPE_IS_TRIVIAL \
	     && ]])[defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;]b4_locations_if([
  YYLTYPE yyls_alloc;])[
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
]b4_locations_if(
[# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE) + sizeof (YYLTYPE)) \
      + 2 * YYSTACK_GAP_MAXIMUM)],
[# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)])[

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)				\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack_alloc, Stack, yysize);			\
	Stack = &yyptr->Stack_alloc;					\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  ]b4_final_state_number[
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   ]b4_last[

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  ]b4_tokens_number[
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  ]b4_nterms_number[
/* YYNRULES -- Number of rules.  */
#define YYNRULES  ]b4_rules_number[
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  ]b4_states_number[

/* YYTRANSLATE[YYX] -- Symbol number corresponding to YYX as returned
   by yylex, with out-of-bounds checking.  */
#define YYUNDEFTOK  ]b4_undef_token_number[
#define YYMAXUTOK   ]b4_user_token_number_max[

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, without out-of-bounds checking.  */
static const ]b4_int_type_for([b4_translate])[ yytranslate[] =
{
  ]b4_translate[
};

#if YYDEBUG
]b4_integral_parser_table_define([rline], [b4_rline],
     [YYRLINE[YYN] -- Source line where rule number YYN was defined.])[
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  ]b4_tname[
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[NUM] -- (External) token number corresponding to the
   (internal) symbol number NUM (which must be that of a token).  */
static const ]b4_int_type_for([b4_toknum])[ yytoknum[] =
{
  ]b4_toknum[
};
# endif

#define YYPACT_NINF ]b4_pact_ninf[

#define yypact_value_is_default(yystate) \
  ]b4_table_value_equals([[pact]], [[yystate]], [b4_pact_ninf])[

#define YYTABLE_NINF ]b4_table_ninf[

#define yytable_value_is_error(yytable_value) \
  ]b4_table_value_equals([[table]], [[yytable_value]], [b4_table_ninf])[

]b4_parser_tables_define[

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  However,
   YYFAIL appears to be in use.  Nevertheless, it is formally deprecated
   in Bison 2.4.2's NEWS entry, where a plan to phase it out is
   discussed.  */

#define YYFAIL		goto yyerrlab
#if defined YYFAIL
  /* This is here to suppress warnings from the GCC cpp's
     -Wunused-macros.  Normally we don't worry about that warning, but
     some users do, and we want to make it easy for users to remove
     YYFAIL uses, which will produce warnings from Bison 2.5.  */
#endif

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (]b4_yyerror_args[YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif]b4_locations_if([[


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if defined YYLTYPE_IS_TRIVIAL && YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif]], [[


/* This macro is provided for backward compatibility. */

#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif]])[


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (]b4_pure_if([&yylval[]b4_locations_if([, &yylloc]), ])[YYLEX_PARAM)
#else
# define YYLEX ]b4_c_function_call([yylex], [int], b4_lex_param)[
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value]b4_locations_if([, Location])[]b4_user_args[); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))

]b4_yy_symbol_print_generate([b4_c_function_def])[

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

]b4_c_function_def([yy_stack_print], [static void],
		   [[yytype_int16 *yybottom], [yybottom]],
		   [[yytype_int16 *yytop],    [yytop]])[
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

]b4_c_function_def([yy_reduce_print], [static void],
                   [[yytype_int16 *yyssp], [yyssp]],
                   [[YYSTYPE *yyvsp], [yyvsp]],
    b4_locations_if([[[YYLTYPE *yylsp], [yylsp]],
                   ])[[int yyrule], [yyrule]]m4_ifset([b4_parse_param], [,
                   b4_parse_param]))[
{
  unsigned long int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       yystos[yyssp[yyi + 1 - yynrhs]],
                       &]b4_rhs_value(yynrhs, yyi + 1)[
                       ]b4_locations_if([, &]b4_rhs_location(yynrhs, yyi + 1))[]dnl
                       b4_user_args[);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyssp, yyvsp, ]b4_locations_if([yylsp, ])[Rule]b4_user_args[); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH ]b4_stack_depth_init[
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH ]b4_stack_depth_max[
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
]b4_c_function_def([yystrlen], [static YYSIZE_T],
   [[const char *yystr], [yystr]])[
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
]b4_c_function_def([yystpcpy], [static char *],
   [[char *yydest], [yydest]], [[const char *yysrc], [yysrc]])[
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
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
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN while in state YYSTATE.

   Return 0 if *YYMSG was successfully written.  Return 1 if an ordinary
   "syntax error" message will suffice instead.  Return 2 if *YYMSG is
   not large enough to hold the message.  In the last case, also set
   *YYMSG_ALLOC to either (a) the required number of bytes or (b) zero
   if the required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                int yystate, int yytoken)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 1;
  else
    {
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytoken]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      /* Internationalized format string. */
      const char *yyformat = 0;
      /* Arguments of yyformat. */
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  In other words, skip the first -YYN actions for this
	 state because they are default actions.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      /* Number of reported tokens (one for the "unexpected", one per
         "expected"). */
      int yycount = 0;
      int yyx;

      yyarg[yycount++] = yytname[yytoken];

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
	    && !yytable_value_is_error (yytable[yyx + yyn]))
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    if (! (yysize <= yysize1
		   && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
	      {
		/* Overflow.  */
		*yymsg_alloc = 0;
		return 2;
	      }
	    yysize = yysize1;
	  }

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

      yysize1 = yysize + yystrlen (yyformat);
      if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
        {
          /* Overflow.  */
          *yymsg_alloc = 0;
          return 2;
        }
      yysize = yysize1;

      if (*yymsg_alloc < yysize)
        {
          *yymsg_alloc = 2 * yysize;
          if (! (yysize <= *yymsg_alloc
                 && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
            *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
          return 2;
        }

      /* Avoid sprintf, as that infringes on the user's name space.
         Don't have undefined behavior even if the translation
         produced a string with the wrong number of "%s"s.  */
      {
        char *yyp = *yymsg;
        int yyi = 0;
        while ((*yyp = *yyformat) != '\0')
          if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
            {
              yyp += yytnamerr (yyp, yyarg[yyi++]);
              yyformat += 2;
            }
          else
            {
              yyp++;
              yyformat++;
            }
      }
      return 0;
    }
}
#endif /* YYERROR_VERBOSE */


]b4_yydestruct_generate([b4_c_function_def])b4_push_if([], [[


/* Prevent warnings from -Wmissing-prototypes.  */
#ifdef YYPARSE_PARAM
]b4_c_function_decl([yyparse], [int],
                    [[void *YYPARSE_PARAM], [YYPARSE_PARAM]])[
#else /* ! YYPARSE_PARAM */
]b4_c_function_decl([yyparse], [int], b4_parse_param)[
#endif /* ! YYPARSE_PARAM */]])b4_pure_if([], [

b4_declare_scanner_communication_variables])[]b4_push_if([[


struct yypstate
  {]b4_declare_parser_state_variables[
    /* Used to determine if this is the first time this instance has
       been used.  */
    int yynew;
  };]b4_pure_if([], [[

static char yypstate_allocated = 0;]])b4_pull_if([

b4_c_function_def([[yyparse]], [[int]], b4_parse_param)[
{
  return yypull_parse (0]m4_ifset([b4_parse_param],
                                  [[, ]b4_c_args(b4_parse_param)])[);
}

]b4_c_function_def([[yypull_parse]], [[int]],
  [[[yypstate *yyps]], [[yyps]]]m4_ifset([b4_parse_param], [,
  b4_parse_param]))[
{
  int yystatus;
  yypstate *yyps_local;]b4_pure_if([[
  int yychar;
  YYSTYPE yylval;]b4_locations_if([[
  YYLTYPE yylloc;]])])[
  if (yyps == 0)
    {
      yyps_local = yypstate_new ();
      if (!yyps_local)
        {]b4_pure_if([[
          yyerror (]b4_yyerror_args[YY_("memory exhausted"));]], [[
          if (!yypstate_allocated)
            yyerror (]b4_yyerror_args[YY_("memory exhausted"));]])[
          return 2;
        }
    }
  else
    yyps_local = yyps;
  do {
    yychar = YYLEX;
    yystatus =
      yypush_parse (yyps_local]b4_pure_if([[, yychar, &yylval]b4_locations_if([[, &yylloc]])])m4_ifset([b4_parse_param], [, b4_c_args(b4_parse_param)])[);
  } while (yystatus == YYPUSH_MORE);
  if (yyps == 0)
    yypstate_delete (yyps_local);
  return yystatus;
}]])[

/* Initialize the parser data structure.  */
]b4_c_function_def([[yypstate_new]], [[yypstate *]])[
{
  yypstate *yyps;]b4_pure_if([], [[
  if (yypstate_allocated)
    return 0;]])[
  yyps = (yypstate *) malloc (sizeof *yyps);
  if (!yyps)
    return 0;
  yyps->yynew = 1;]b4_pure_if([], [[
  yypstate_allocated = 1;]])[
  return yyps;
}

]b4_c_function_def([[yypstate_delete]], [[void]],
                   [[[yypstate *yyps]], [[yyps]]])[
{
#ifndef yyoverflow
  /* If the stack was reallocated but the parse did not complete, then the
     stack still needs to be freed.  */
  if (!yyps->yynew && yyps->yyss != yyps->yyssa)
    YYSTACK_FREE (yyps->yyss);
#endif
  free (yyps);]b4_pure_if([], [[
  yypstate_allocated = 0;]])[
}
]b4_pure_if([[
#define ]b4_prefix[nerrs yyps->]b4_prefix[nerrs]])[
#define yystate yyps->yystate
#define yyerrstatus yyps->yyerrstatus
#define yyssa yyps->yyssa
#define yyss yyps->yyss
#define yyssp yyps->yyssp
#define yyvsa yyps->yyvsa
#define yyvs yyps->yyvs
#define yyvsp yyps->yyvsp]b4_locations_if([[
#define yylsa yyps->yylsa
#define yyls yyps->yyls
#define yylsp yyps->yylsp
#define yyerror_range yyps->yyerror_range]])[
#define yystacksize yyps->yystacksize


/*---------------.
| yypush_parse.  |
`---------------*/

]b4_c_function_def([[yypush_parse]], [[int]],
  [[[yypstate *yyps]], [[yyps]]]b4_pure_if([,
  [[[int yypushed_char]], [[yypushed_char]]],
  [[[YYSTYPE const *yypushed_val]], [[yypushed_val]]]b4_locations_if([,
  [[[YYLTYPE const *yypushed_loc]], [[yypushed_loc]]]])])m4_ifset([b4_parse_param], [,
  b4_parse_param]))], [[


/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
]b4_c_function_def([yyparse], [int],
                   [[void *YYPARSE_PARAM], [YYPARSE_PARAM]])[
#else /* ! YYPARSE_PARAM */
]b4_c_function_def([yyparse], [int], b4_parse_param)[
#endif]])[
{]b4_pure_if([b4_declare_scanner_communication_variables
])b4_push_if([b4_pure_if([], [[
  int yypushed_char = yychar;
  YYSTYPE yypushed_val = yylval;]b4_locations_if([[
  YYLTYPE yypushed_loc = yylloc;]])
])],
  [b4_declare_parser_state_variables
])[
  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;]b4_locations_if([[
  YYLTYPE yyloc;]])[

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N)]b4_locations_if([, yylsp -= (N)])[)

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;]b4_push_if([[

  if (!yyps->yynew)
    {
      yyn = yypact[yystate];
      goto yyread_pushed_token;
    }]])[

  yytoken = 0;
  yyss = yyssa;
  yyvs = yyvsa;]b4_locations_if([[
  yyls = yylsa;]])[
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */
  yyssp = yyss;
  yyvsp = yyvs;]b4_locations_if([[
  yylsp = yyls;

#if defined YYLTYPE_IS_TRIVIAL && YYLTYPE_IS_TRIVIAL
  /* Initialize the default location before parsing starts.  */
  yylloc.first_line   = yylloc.last_line   = ]b4_location_initial_line[;
  yylloc.first_column = yylloc.last_column = ]b4_location_initial_column[;
#endif]])
m4_ifdef([b4_initial_action],[
m4_pushdef([b4_at_dollar],     [m4_define([b4_at_dollar_used])yylloc])dnl
m4_pushdef([b4_dollar_dollar], [m4_define([b4_dollar_dollar_used])yylval])dnl
/* User initialization code.  */
b4_user_initial_action
m4_popdef([b4_dollar_dollar])dnl
m4_popdef([b4_at_dollar])])dnl
m4_ifdef([b4_dollar_dollar_used],[[  yyvsp[0] = yylval;
]])dnl
m4_ifdef([b4_at_dollar_used], [[  yylsp[0] = yylloc;
]])[
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;]b4_locations_if([
	YYLTYPE *yyls1 = yyls;])[

	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),]b4_locations_if([
		    &yyls1, yysize * sizeof (*yylsp),])[
		    &yystacksize);
]b4_locations_if([
	yyls = yyls1;])[
	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss_alloc, yyss);
	YYSTACK_RELOCATE (yyvs_alloc, yyvs);]b4_locations_if([
	YYSTACK_RELOCATE (yyls_alloc, yyls);])[
#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;]b4_locations_if([
      yylsp = yyls + yysize - 1;])[

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {]b4_push_if([[
      if (!yyps->yynew)
        {]b4_use_push_for_pull_if([], [[
          YYDPRINTF ((stderr, "Return for a new token:\n"));]])[
          yyresult = YYPUSH_MORE;
          goto yypushreturn;
        }
      yyps->yynew = 0;]b4_pure_if([], [[
      /* Restoring the pushed token is only necessary for the first
         yypush_parse invocation since subsequent invocations don't overwrite
         it before jumping to yyread_pushed_token.  */
      yychar = yypushed_char;
      yylval = yypushed_val;]b4_locations_if([[
      yylloc = yypushed_loc;]])])[
yyread_pushed_token:]])[
      YYDPRINTF ((stderr, "Reading a token: "));]b4_push_if([b4_pure_if([[
      yychar = yypushed_char;
      if (yypushed_val)
        yylval = *yypushed_val;]b4_locations_if([[
      if (yypushed_loc)
        yylloc = *yypushed_loc;]])])], [[
      yychar = YYLEX;]])[
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;
]b4_locations_if([  *++yylsp = yylloc;])[
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];

]b4_locations_if(
[[  /* Default location.  */
  YYLLOC_DEFAULT (yyloc, (yylsp - yylen), yylen);]])[
  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
      ]b4_user_actions[
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;]b4_locations_if([
  *++yylsp = yyloc;])[

  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (]b4_yyerror_args[YY_("syntax error"));
#else
      while (1)
        {
          int yysyntax_error_status =
            yysyntax_error (&yymsg_alloc, &yymsg, yystate, yytoken);
          if (yysyntax_error_status == 2 && 0 < yymsg_alloc)
            {
              if (yymsg != yymsgbuf)
                YYSTACK_FREE (yymsg);
              yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
              if (yymsg)
                continue;
              yymsg = yymsgbuf;
              yymsg_alloc = sizeof yymsgbuf;
            }
          if (yysyntax_error_status == 0)
            yyerror (]b4_yyerror_args[yymsg);
          else
            yyerror (]b4_yyerror_args[YY_("syntax error"));
          if (yysyntax_error_status == 2)
            goto yyexhaustedlab;
          break;
        }
#endif
    }

]b4_locations_if([[  yyerror_range[1] = yylloc;]])[

  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval]b4_locations_if([, &yylloc])[]b4_user_args[);
	  yychar = YYEMPTY;
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
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

]b4_locations_if([[  yyerror_range[1] = yylsp[1-yylen];
]])[  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;

]b4_locations_if([[      yyerror_range[1] = *yylsp;]])[
      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp]b4_locations_if([, yylsp])[]b4_user_args[);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  *++yyvsp = yylval;
]b4_locations_if([[
  yyerror_range[2] = yylloc;
  /* Using YYLLOC is tempting, but would change the location of
     the lookahead.  YYLOC is available though.  */
  YYLLOC_DEFAULT (yyloc, yyerror_range, 2);
  *++yylsp = yyloc;]])[

  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined(yyoverflow) || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (]b4_yyerror_args[YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval]b4_locations_if([, &yylloc])[]b4_user_args[);
    }
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp]b4_locations_if([, yylsp])[]b4_user_args[);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif]b4_push_if([[
  yyps->yynew = 1;

yypushreturn:]])[
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}

]b4_epilogue[]dnl
b4_defines_if(
[@output(b4_spec_defines_file@)@
b4_copyright([Interface for Bison's Yacc-like parsers in C])dnl

b4_percent_code_get([[requires]])[]dnl

b4_token_enums_defines(b4_tokens)

[#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
]m4_ifdef([b4_stype],
[[typedef union ]b4_union_name[
{
]b4_user_stype[
} YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1]],
[m4_if(b4_tag_seen_flag, 0,
[[typedef int YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1]])])[
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif

]b4_pure_if([], [[extern YYSTYPE ]b4_prefix[lval;]])

b4_locations_if(
[#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
} YYLTYPE;
# define yyltype YYLTYPE /* obsolescent; will be withdrawn */
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif

]b4_pure_if([], [[extern YYLTYPE ]b4_prefix[lloc;]])
)dnl b4_locations_if
b4_push_if([[
#ifndef YYPUSH_DECLS
#  define YYPUSH_DECLS
struct ]b4_prefix[pstate;
typedef struct ]b4_prefix[pstate ]b4_prefix[pstate;
enum { YYPUSH_MORE = 4 };
]b4_pull_if([b4_c_function_decl([b4_prefix[parse]], [[int]], b4_parse_param)
])b4_c_function_decl([b4_prefix[push_parse]], [[int]],
  [[b4_prefix[pstate *yyps]], [[yyps]]]b4_pure_if([,
  [[[int yypushed_char]], [[yypushed_char]]],
  [[[YYSTYPE const *yypushed_val]], [[yypushed_val]]]b4_locations_if([,
  [[[YYLTYPE const *yypushed_loc]], [[yypushed_loc]]]])])m4_ifset([b4_parse_param], [,
  b4_parse_param]))
b4_pull_if([b4_c_function_decl([b4_prefix[pull_parse]], [[int]],
  [[b4_prefix[pstate *yyps]], [[yyps]]]m4_ifset([b4_parse_param], [,
  b4_parse_param]))])
b4_c_function_decl([b4_prefix[pstate_new]], [b4_prefix[pstate *]],
                    [[[void]], []])
b4_c_function_decl([b4_prefix[pstate_delete]], [[void]],
                   [[b4_prefix[pstate *yyps]], [[yyps]]])[
#endif
]])
b4_percent_code_get([[provides]])[]dnl
])dnl b4_defines_if
m4_divert_pop(0)
m4_popdef([b4_copyright_years])
