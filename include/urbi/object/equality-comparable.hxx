/*
 * Copyright (C) 2009, 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

namespace urbi
{
  namespace object
  {
    template <typename Exact, typename Value>
    inline
    EqualityComparable<Exact, Value>::~EqualityComparable()
    {
    }

    template <typename Exact, typename Value>
    inline
    bool
    EqualityComparable<Exact, Value>::operator==(const rObject& rhs) const
    {
      return (rhs->is_a<Exact>()
              && *this == rhs->as<Exact>()->value_get());
    }

    template <typename Exact, typename Value>
    inline
    bool
    EqualityComparable<Exact, Value>::operator==(const value_type& rhs) const
    {
      return value_get() == rhs;
    }
  }
}
