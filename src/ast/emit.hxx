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
 ** \file ast/emit.hxx
 ** \brief Inline methods of ast::Emit.
 */

#ifndef AST_EMIT_HXX
# define AST_EMIT_HXX

# include <ast/emit.hh>

namespace ast
{


#if defined ENABLE_SERIALIZATION
 template <typename T>
  void
  Emit::serialize(libport::serialize::OSerializer<T>& ser) const
  {
    (void)ser;
    Exp::serialize(ser);
    ser.template serialize<rExp>("event", event_);
    ser.template serialize<exps_type*>("arguments", arguments_);
    ser.template serialize<rExp>("duration", duration_);
  }

  template <typename T>
  Emit::Emit(libport::serialize::ISerializer<T>& ser)
    : Exp(ser)
  {
    (void)ser;
    event_ = ser.template unserialize<rExp>("event");
    arguments_ = ser.template unserialize<exps_type*>("arguments");
    duration_ = ser.template unserialize<rExp>("duration");
  }
#endif

  inline const rExp&
  Emit::event_get () const
  {
    return event_;
  }
  inline rExp&
  Emit::event_get ()
  {
    return event_;
  }

  inline const exps_type*
  Emit::arguments_get () const
  {
    return arguments_;
  }
  inline exps_type*
  Emit::arguments_get ()
  {
    return arguments_;
  }

  inline const rExp&
  Emit::duration_get () const
  {
    return duration_;
  }
  inline rExp&
  Emit::duration_get ()
  {
    return duration_;
  }


} // namespace ast

#endif // !AST_EMIT_HXX
