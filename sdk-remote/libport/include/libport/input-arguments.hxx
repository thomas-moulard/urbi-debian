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

  namespace opts
  {
    inline
    std::ostream&
    operator<< (std::ostream& o, const Data& d)
    {
      return d.dump(o);
    }

  }

}
