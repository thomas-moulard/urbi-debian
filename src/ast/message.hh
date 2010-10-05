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
 ** \file ast/message.hh
 ** \brief Declaration of ast::Message.
 */

#ifndef AST_MESSAGE_HH
# define AST_MESSAGE_HH

# include <ast/exp.hh>
# include <string>
# include <urbi/object/fwd.hh>
# include <kernel/config.h>

#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif

namespace ast
{

/// Display a message when visited.  Used for syntactic issues.
  class Message : public Exp
  {
    /** \name Ctor & dtor.
     ** \{ */
  public:
    /// Construct a Message node.
    Message (const loc& location, const std::string& text,
             const std::string& tag);
    /// Destroy a Message node.
    virtual ~Message ();
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
    Message(libport::serialize::ISerializer<T>& ser);
    template <typename T>
    void
    serialize(libport::serialize::OSerializer<T>& ser) const;
    /// \}
#endif


    /** \name Accessors.
     ** \{ */
  public:
    /// Return the message to display when visited.
    const std::string& text_get () const;
    /// Return the message to display when visited.
    std::string& text_get ();
    /// Return the tag for the message (error or warning).
    const std::string& tag_get () const;
    /// Return the tag for the message (error or warning).
    std::string& tag_get ();
    /** \} */

  protected:
    /// The message to display when visited.
    std::string text_;
    /// The tag for the message (error or warning).
    std::string tag_;
  };

} // namespace ast


#if ENABLE_SERIALIZATION
# include <serialize/serialize.hh>
#endif
# include <ast/message.hxx>

#endif // !AST_MESSAGE_HH
