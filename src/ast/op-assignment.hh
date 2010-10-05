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
 ** \file ast/op-assignment.hh
 ** \brief Declaration of ast::OpAssignment.
 */

#ifndef AST_OP_ASSIGNMENT_HH
# define AST_OP_ASSIGNMENT_HH

# include <libport/symbol.hh>
# include <ast/write.hh>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

  /// OpAssignment.
  class OpAssignment : public Write
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct an OpAssignment node.
    OpAssignment (const loc& location, const rLValue& what,
                  const rExp& value, const libport::Symbol& op);
    /// Destroy an OpAssignment node.
    virtual ~OpAssignment ();
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
    OpAssignment(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return op.
    const libport::Symbol& op_get () const;
    /// Return op.
    libport::Symbol& op_get ();
    /** \} */

  protected:
    /// op.
    libport::Symbol op_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/op-assignment.hxx>

#endif // !AST_OP_ASSIGNMENT_HH
