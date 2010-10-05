/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

namespace libport
{

  inline
  Backtrace::const_iterator
  Backtrace::begin() const
  {
    return backtrace_.begin();
  }

  inline
  Backtrace::const_iterator
  Backtrace::end() const
  {
    return backtrace_.end();
  }

  inline
  std::ostream&
  operator<<(std::ostream& o, const Backtrace& b)
  {
    return b.dump(o);
  }


}
