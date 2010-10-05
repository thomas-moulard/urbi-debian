/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <libport/indent.hh>
#include <libport/separate.hh>

#include <ast/exp.hh>
#include <ast/exps-type.hh>
#include <ast/print.hh>

namespace ast
{
  std::ostream&
  operator<<(std::ostream& o, const exps_type& es)
  {
    // Specifying template parameters is needed for gcc-3.
    return o << libport::separate<const exps_type,
                                  std::ostream&(*)(std::ostream&)>
                                 (es, libport::iendl);
  }
}
