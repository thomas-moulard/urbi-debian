/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/**
 ** \file object/symbols.cc
 ** \brief Frequently used symbol names.
 */

#include <object/symbols.hh>

#if defined SYMBOLS_PRECOMPILED

namespace object
{

# define SYMBOL_DEFINE(Name, Value)		\
  libport::Symbol symbol_ ## Name (Value);
  SYMBOLS_APPLY(SYMBOL_DEFINE);
# undef SYMBOL_DEFINE

} // namespace object

#else // ! SYMBOLS_PRECOMPILED

# include <map>
# include <string>
# include <object/precompiled-symbols.hh>

namespace object
{

  class SymbolMap
  {
  public:
    SymbolMap()
      : map_()
    {
      libport::Symbol value;
# define SYMBOL_DEFINE(Name, Value)		\
      value = libport::Symbol(Value);		\
      map_[#Name] = value;
      SYMBOLS_APPLY(SYMBOL_DEFINE);
# undef SYMBOL_DEFINE
    }

    /// Return the symbol associated to \a s if there is one.
    /// Otherwise create a symbol equal to \a s.
    /// \warning: Special characters (SP, LT, etc.) are *not*
    /// handled in this case.
    libport::Symbol
    operator[](const std::string& s)
    {
      // See ``Efficient STL''.
      map_type::iterator i = map_.lower_bound(s);
      if (i != map_.end() && !map_.key_comp()(s, i->first))
        return i->second;
      else
      {
        libport::Symbol res(s);
        map_.insert(map_type::value_type(s, res));
        return res;
      }
    }

  private:
    typedef std::map<const std::string, libport::Symbol> map_type;
    map_type map_;
  };

  libport::Symbol
  symbol_get(const std::string& s)
  {
    static SymbolMap map;
    return map[s];
  }

} // namespace object

#endif // ! SYMBOLS_PRECOMPILED
