/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <libport/config.h>
#include <libport/utime.hh>

namespace libport
{
  static boost::posix_time::ptime reference =
    boost::posix_time::microsec_clock::local_time();

  const boost::posix_time::ptime&
  utime_reference()
  {
    return reference;
  }

  void
  utime_reference_set(const boost::posix_time::ptime& ref)
  {
    reference = ref;
  }

  utime_t
  utime()
  {
    return (boost::posix_time::microsec_clock::local_time()
            - utime_reference()).total_microseconds();
  }
}
