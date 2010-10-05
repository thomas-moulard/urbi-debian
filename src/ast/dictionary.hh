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
 ** \file ast/dictionary.hh
 ** \brief Declaration of ast::Dictionary.
 */

#ifndef AST_DICTIONARY_HH
# define AST_DICTIONARY_HH

# include <ast/exp.hh>
# include <libport/hash.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// Dictionary.
  class Dictionary : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a Dictionary node.
    Dictionary (const loc& location, const rExp& base,
                const modifiers_type& value);
    /// Destroy a Dictionary node.
    virtual ~Dictionary ();
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
    Dictionary(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return base value. this is a generator if set.
    const rExp& base_get () const;
    /// Return base value. this is a generator if set.
    rExp& base_get ();
    /// Return stored dictionary.
    const modifiers_type& value_get () const;
    /// Return stored dictionary.
    modifiers_type& value_get ();
    /** \} */

  protected:
    /// Base value. This is a Generator if set.
    rExp base_;
    /// Stored dictionary.
    modifiers_type value_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/dictionary.hxx>

#endif // !AST_DICTIONARY_HH
