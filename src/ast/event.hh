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
 ** \file ast/event.hh
 ** \brief Declaration of ast::Event.
 */

#ifndef AST_EVENT_HH
# define AST_EVENT_HH

# include <ast/exp.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// Event.
  class Event : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct an Event node.
    Event (const loc& location, const rExp& exp);
    /// Destroy an Event node.
    virtual ~Event ();
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
    Event(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return exp.
    const rExp& exp_get () const;
    /// Return exp.
    rExp& exp_get ();
    /// Set exp.
    void exp_set (const rExp&);
    /** \} */

  protected:
    /// exp.
    rExp exp_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/event.hxx>

#endif // !AST_EVENT_HH
