/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef LIBPORT_SERIALIZE_BINARY_I_SERIALIZER_HH
# define LIBPORT_SERIALIZE_BINARY_I_SERIALIZER_HH

# include <libport/hash.hh>
# include <libport/symbol.hh>
# include <serialize/export.hh>
# include <serialize/i-serializer.hh>

namespace libport
{
  namespace serialize
  {
    class SERIALIZE_API BinaryISerializer: public ISerializer<BinaryISerializer>
    {
    public:
      BinaryISerializer(std::istream& input);
      ~BinaryISerializer();
      template <typename T>
      struct Impl;

    private:
      template <typename T>
      struct PCImpl;
      template <typename T>
      struct PHImpl;

      typedef std::vector<void*> ptr_map_type;
      ptr_map_type ptr_map_;

      typedef std::vector<libport::Symbol> symbol_map_type;
      symbol_map_type sym_map_;

      template <typename T>
      friend struct PCImpl;
      template <typename T>
      friend struct PHImpl;
    };
  }
}

# include <serialize/binary-i-serializer.hxx>

#endif
