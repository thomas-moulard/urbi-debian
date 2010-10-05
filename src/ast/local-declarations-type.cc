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

#include <ast/local-declarations-type.hh>
#include <ast/print.hh>

namespace ast
{
  std::ostream&
  operator<<(std::ostream& o, const local_declarations_type& decs)
  {
    bool tail = false;
    foreach (ast::rConstLocalDeclaration dec, decs)
    {
      if (tail++)
        o << ", ";
      o << "var " << dec->what_get();
    }
    return o;
  }
}
