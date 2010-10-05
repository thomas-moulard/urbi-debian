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
 ** \file ast/message.hxx
 ** \brief Inline methods of ast::Message.
 */

#ifndef AST_MESSAGE_HXX
# define AST_MESSAGE_HXX

# include <ast/message.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Message::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<std::string>("text", text_);
    ser.template serialize<std::string>("tag", tag_);
  }

  template <typename T>
  Message::Message(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    text_ = ser.template unserialize<std::string>("text");
    tag_ = ser.template unserialize<std::string>("tag");
  }
#endif

  inline const std::string&
  Message::text_get () const
  {
    return text_;
  }
  inline std::string&
  Message::text_get ()
  {
    return text_;
  }

  inline const std::string&
  Message::tag_get () const
  {
    return tag_;
  }
  inline std::string&
  Message::tag_get ()
  {
    return tag_;
  }


} // namespace ast

#endif // !AST_MESSAGE_HXX
