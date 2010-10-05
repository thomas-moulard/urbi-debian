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
 ** \file ast/catch.hh
 ** \brief Declaration of ast::Catch.
 */

#ifndef AST_CATCH_HH
# define AST_CATCH_HH

# include <ast/exp.hh>
# include <ast/match.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// Catch.
  class Catch : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a Catch node.
    Catch (const loc& location, rMatch match, const rExp& body);
    /// Destroy a Catch node.
    virtual ~Catch ();
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
    Catch(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return match.
    const rMatch& match_get () const;
    /// Return match.
    rMatch& match_get ();
    /// Return the handler to execute if we have a match.
    const rExp& body_get () const;
    /// Return the handler to execute if we have a match.
    rExp& body_get ();
    /** \} */

  protected:
    /// match.
    rMatch match_;
    /// The handler to execute if we have a match.
    rExp body_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/catch.hxx>

#endif // !AST_CATCH_HH
