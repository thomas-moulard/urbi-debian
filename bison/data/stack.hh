# C++ skeleton for Bison

# Copyright (C) 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009,
# 2010 Free Software Foundation, Inc.

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

m4_pushdef([b4_copyright_years],
           [2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010])

# We do want M4 expansion after # for CPP macros.
m4_changecom()
m4_divert_push(0)dnl
@output(b4_dir_prefix[]stack.hh@)@
b4_copyright([Stack handling for Bison parsers in C++])[

#ifndef BISON_STACK_HH
# define BISON_STACK_HH

#include <deque>

]b4_namespace_open[
  template <class T, class S = std::deque<T> >
  class stack
  {
  public:
    // Hide our reversed order.
    typedef typename S::reverse_iterator iterator;
    typedef typename S::const_reverse_iterator const_iterator;

    stack ()
      : seq_ ()
    {
    }

    stack (unsigned int n)
      : seq_ (n)
    {
    }

    inline
    T&
    operator [] (unsigned int i)
    {
      return seq_[i];
    }

    inline
    const T&
    operator [] (unsigned int i) const
    {
      return seq_[i];
    }

    inline
    void
    push (const T& t)
    {
      seq_.push_front (t);
    }

    inline
    void
    pop (unsigned int n = 1)
    {
      for (; n; --n)
	seq_.pop_front ();
    }

    inline
    typename S::size_type
    size () const
    {
      return seq_.size ();
    }

    inline
    const_iterator
    begin () const
    {
      return seq_.rbegin ();
    }

    inline
    const_iterator
    end () const
    {
      return seq_.rend ();
    }

  private:
    /// The wrapped container.
    S seq_;
  };

  /// Present a slice of the top of a stack.
  template <class T, class S = stack<T> >
  class slice
  {
  public:
    slice (const S& stack, unsigned int range)
      : stack_ (stack),
        range_ (range)
    {
    }

    inline
    const T&
    operator [] (unsigned int i) const
    {
      return stack_[range_ - i];
    }

  private:
    const S& stack_;
    unsigned int range_;
  };
]b4_namespace_close[

#endif // not BISON_STACK_HH[]dnl
]
m4_divert_pop(0)
m4_popdef([b4_copyright_years])dnl
m4_changecom([#])
