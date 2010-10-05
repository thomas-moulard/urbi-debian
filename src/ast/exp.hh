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
 ** \file ast/exp.hh
 ** \brief Declaration of ast::Exp.
 */

#ifndef AST_EXP_HH
# define AST_EXP_HH

# include <ast/ast.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// Exp.
  class Exp : public Ast
  {
  public:
    /// Whether it is an empty Nary.
    virtual bool empty() const;
    /// Whether is an instance of ast::Implicit.
    virtual bool implicit() const;

    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct an Exp node.
    Exp (const loc& location);
    /// Destroy an Exp node.
    virtual ~Exp ();
    /** \} */

#if defined ENABLE_SERIALIZATION
    /// \name Serialization.
    /// \{ */
  public:
    template <typename T>
    Exp(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif

  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/exp.hxx>

#endif // !AST_EXP_HH
