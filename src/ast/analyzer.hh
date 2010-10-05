/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/**
 ** \file ast/analyzer.hh
 ** \brief Declaration of ast::Analyzer.
 */

#ifndef AST_ANALYZER_HH
# define AST_ANALYZER_HH

# include <ast/cloner.hh>
# include <ast/error.hh>

namespace ast
{

  /// \brief Duplicate an Ast.
  class Analyzer : public ast::Cloner
  {
  public:
    typedef ast::DefaultConstVisitor super_type;

    Analyzer();
    virtual ~Analyzer();

    /// The errors seen so far.
    ast::Error& errors_get();

  protected:
    /// The errors found so far.
    ast::Error errors_;

    /// Factory.
    std::auto_ptr<ast::Factory> factory_;
  };

  /// Apply a visitor on a.
  /// Handle errors.
  rExp analyze(Analyzer& v, rConstAst a);

} // namespace ast

#endif // !AST_ANALYZER_HH
