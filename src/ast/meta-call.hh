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
 ** \file ast/meta-call.hh
 ** \brief Declaration of ast::MetaCall.
 */

#ifndef AST_META_CALL_HH
# define AST_META_CALL_HH

# include <ast/exp.hh>
# include <ast/lvalue-args.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// MetaCall.
  class MetaCall : public LValueArgs
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a MetaCall node.
    MetaCall (const loc& location, exps_type* arguments,
              const rExp& target, unsigned id);
    /// Destroy a MetaCall node.
    virtual ~MetaCall ();
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
    MetaCall(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return target.
    const rExp& target_get () const;
    /// Return target.
    rExp& target_get ();
    /// Return id.
    const unsigned& id_get () const;
    /// Return id.
    unsigned& id_get ();
    /** \} */

  protected:
    /// target.
    rExp target_;
    /// id.
    unsigned id_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/meta-call.hxx>

#endif // !AST_META_CALL_HH
