/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef LIBPORT_SERIALIZE_FWD_HH
# define LIBPORT_SERIALIZE_FWD_HH

namespace libport
{
  namespace serialize
  {
    enum pointer_status
    {
      null,
      cached,
      serialized,
    };
  }
}

#endif
