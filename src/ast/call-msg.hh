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
 ** \file ast/call-msg.hh
 ** \brief Declaration of ast::CallMsg.
 */

#ifndef AST_CALL_MSG_HH
# define AST_CALL_MSG_HH

# include <ast/exp.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// CallMsg.
  class CallMsg : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a CallMsg node.
    CallMsg (const loc& location);
    /// Destroy a CallMsg node.
    virtual ~CallMsg ();
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
    CallMsg(libport::serialize::ISerializer<T>& ser);
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
# include <ast/call-msg.hxx>

#endif // !AST_CALL_MSG_HH
