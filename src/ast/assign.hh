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
 ** \file ast/assign.hh
 ** \brief Declaration of ast::Assign.
 */

#ifndef AST_ASSIGN_HH
# define AST_ASSIGN_HH

# include <ast/exp.hh>
# include <libport/hash.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// Assign.
  class Assign : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct an Assign node.
    Assign (const loc& location, const rExp& what, const rExp& value,
            modifiers_type* modifiers);
    /// Destroy an Assign node.
    virtual ~Assign ();
    /** \} */

    /// \name Visitors entry point.
    /// \{ */
  public:
    /// Accept a const visitor \a v.
    virtual void accept (ConstVisitor& v) const;
    /// Accept a non-const visitor \a v.
    virtual void accept (Visitor& v);
    /// Evaluate the node in interpreter \a r
    virtual urbi::object::rObject eval(runner::Interpreter& r) const ;
    /// Return the node type
    virtual std::string node_type() const;
    /// \}

#if defined ENABLE_SERIALIZATION
    /// \name Serialization.
    /// \{ */
  public:
    template <typename T>
    Assign(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return what.
    const rExp& what_get () const;
    /// Return what.
    rExp& what_get ();
    /// Return value.
    const rExp& value_get () const;
    /// Return value.
    rExp& value_get ();
    /// Return modifiers.
    const modifiers_type* modifiers_get () const;
    /// Return modifiers.
    modifiers_type* modifiers_get ();
    /// Set modifiers.
    void modifiers_set (modifiers_type*);
    /** \} */

  protected:
    /// what.
    rExp what_;
    /// value.
    rExp value_;
    /// modifiers.
    modifiers_type* modifiers_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/assign.hxx>

#endif // !AST_ASSIGN_HH
