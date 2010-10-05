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
 ** \file ast/while.hh
 ** \brief Declaration of ast::While.
 */

#ifndef AST_WHILE_HH
# define AST_WHILE_HH

# include <ast/exp.hh>
# include <ast/flavored.hh>
# include <ast/scope.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// While.
  class While : public Flavored
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a While node.
    While (const loc& location, const flavor_type& flavor,
           const rExp& test, const rScope& body);
    /// Destroy a While node.
    virtual ~While ();
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
    While(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return condition to continue.
    const rExp& test_get () const;
    /// Return condition to continue.
    rExp& test_get ();
    /// Return executed if test is true.
    const rScope& body_get () const;
    /// Return executed if test is true.
    rScope& body_get ();
    /** \} */

  protected:
    /// Condition to continue.
    rExp test_;
    /// Executed if test is true.
    rScope body_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/while.hxx>

#endif // !AST_WHILE_HH
