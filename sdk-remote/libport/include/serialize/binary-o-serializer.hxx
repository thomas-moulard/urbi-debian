/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef LIBPORT_SERIALIZE_BINARY_O_SERIALIZER_HXX
# define LIBPORT_SERIALIZE_BINARY_O_SERIALIZER_HXX


# include <vector>

# include <libport/arpa/inet.h>
# include <libport/meta.hh>
# include <libport/foreach.hh>
# include <libport/hierarchy.hh>
# include <serialize/fwd.hh>

namespace libport
{
  namespace serialize
  {
    /*----------------.
    | Generic class.  |
    `----------------*/
    template <typename T>
    struct CImpl
    {
      static void
      put(const std::string&, const T& v, std::ostream&, BinaryOSerializer& ser)
      {
        v.serialize(ser);
      }
    };

    /*------------.
    | Hierarchy.  |
    `------------*/
    typedef std::pair<const meta::BaseHierarchy*, BinaryOSerializer*> ICookie;
    template <typename T>
    struct Serialize
    {
      static void res(const ICookie& c)
      {
        const T* v = static_cast<const T*>(c.first);
        // FIXME: triple lookup
        // FIXME: 256 max classes?
        c.second->serialize<unsigned char>("id", v->id());
        v->serialize(*c.second);
      }
    };

    template <typename T>
    struct HImpl
    {
      static void
      put(const std::string&,
          const T& v,
          std::ostream&, BinaryOSerializer& ser)
      {
        ICookie c(&v, &ser);
        // FIXME: double lookup
        v.template dispatch<Serialize, ICookie>(v.id(), c);
      }
    };

    /*-----------.
    | Fallback.  |
    `-----------*/
    template <typename T>
    struct BinaryOSerializer::Impl
    {
      static void
      put(const std::string& name, const T& v,
          std::ostream& output, BinaryOSerializer& ser)
      {
        meta::If<meta::Inherits<T, meta::BaseHierarchy>::res,
          HImpl<T>, CImpl<T> >::res::put(name, v, output, ser);
      }
    };

    /*-------.
    | char.  |
    `-------*/
    template <>
    struct BinaryOSerializer::Impl<char>
    {
      static void put(const std::string&, char c, std::ostream& output,
                      BinaryOSerializer&)
      {
        write_(output, c);
      }
    };

    /*---------------------.
    | unsigned int/short.  |
    `---------------------*/
# define SERIALIZE_NET_INTEGRAL(Type, Function) \
    template <>                                 \
    struct BinaryOSerializer::Impl<Type>        \
    {                                           \
      static void                               \
      put(const std::string&,                   \
          Type i, std::ostream& output,         \
          BinaryOSerializer&)                   \
      {                                         \
        write_(output, Function(i));            \
      }                                         \
    }

    SERIALIZE_NET_INTEGRAL(unsigned int,   htonl);
    SERIALIZE_NET_INTEGRAL(unsigned short, htons);
#undef SERIALIZE_NET_INTEGRAL

    /*---------.
    | double.  |
    `---------*/
    template <>
    struct BinaryOSerializer::Impl<double>
    {
      static void put(const std::string&, double d, std::ostream& output,
                      BinaryOSerializer&)
      {
        // FIXME: non-portable
        write_(output, d);
      }
    };

#define BOUNCE(From, To)                                                \
    template <>                                                         \
    struct BinaryOSerializer::Impl<From>                                \
    {                                                                   \
      static void put(const std::string& name, From v,                  \
                      std::ostream& output, BinaryOSerializer& ser)     \
      {                                                                 \
        Impl<To>::put(name, static_cast<To>(v), output, ser);           \
      }                                                                 \
    };                                                                  \

    BOUNCE(bool,           char);
    BOUNCE(unsigned char,  char);
    BOUNCE(int,            unsigned int);
    BOUNCE(short,          unsigned short);

#undef BOUNCE

    /*-----------.
    | Pointers.  |
    `-----------*/
    template <typename T>
    struct BinaryOSerializer::Impl<T*>
    {
      static void
      put(const std::string&, const T* ptr, std::ostream& output,
          BinaryOSerializer& ser)
      {
        if (!ptr)
        {
          Impl<char>::put("opt", null, output, ser);
          return;
        }
        ptr_map_type::iterator it =
          ser.ptr_map_.find(reinterpret_cast<long>(ptr));
        if (it != ser.ptr_map_.end())
        {
          Impl<char>::put("opt", cached, output, ser);
          Impl<unsigned>::put("id", it->second, output, ser);
        }
        else
        {
          unsigned id = ser.ptr_id_++;
          ser.ptr_map_[reinterpret_cast<long>(ptr)] = id;
          Impl<char>::put("opt", serialized, output, ser);
          Impl<T>::put("value", *ptr, output, ser);
        }
      }
    };

    /*--------------.
    | std::string.  |
    `--------------*/
    template <>
    struct BinaryOSerializer::Impl<std::string>
    {
      static void
      put(const std::string& name,
          const std::string& s, std::ostream& output,
          BinaryOSerializer& ser)
      {
        size_t size = s.size();
        // FIXME: throw in case of overflow
        Impl<unsigned short>::put(name, size, output, ser);
        output.write(s.c_str(), size);
      }
    };

    /*--------------.
    | std::vector.  |
    `--------------*/
    template <typename T, typename A>
    struct BinaryOSerializer::Impl<std::vector<T, A> >
    {
      static void
      put(const std::string& name,
          const std::vector<T, A>& v, std::ostream& output,
          BinaryOSerializer& ser)
      {
        // FIXME: raise if overflow
        Impl<unsigned short>::put(name, v.size(), output, ser);
        foreach (const T& elt, v)
          Impl<T>::put(name, elt, output, ser);
      }
    };

    // Hash and Symbol serialization is defined here because of
    // serialization/hash/symbol dependency loop.

    /*------------------.
    | libport::Symbol.  |
    `------------------*/
    template <>
    struct BinaryOSerializer::Impl<libport::Symbol>
    {
      static void
      put(const std::string& name,
          libport::Symbol s, std::ostream& output,
          BinaryOSerializer& ser)
      {
        symbol_map_type::iterator it = ser.symbol_map_.find(s);
        if (it == ser.symbol_map_.end())
        {
          Impl<bool>::put("opt", false, output, ser);
          Impl<std::string>::put(name, s.name_get(), output, ser);
          ser.symbol_map_[s] = ser.symbol_id_++;
        }
        else
        {
          Impl<bool>::put("opt", true, output, ser);
          Impl<unsigned>::put("id", it->second, output, ser);
        }
      }
    };

    /*----------------.
    | libport::hash.  |
    `----------------*/
    template <typename K, typename V>
    struct BinaryOSerializer::Impl<boost::unordered_map<K, V> >
    {
      static void
      put(const std::string&,
          const boost::unordered_map<K, V>& m, std::ostream&,
          BinaryOSerializer& ser)
      {
        typedef typename boost::unordered_map<K, V>::value_type Value;
        // FIXME: raise if overflow
        ser.serialize<unsigned short>("size", m.size());
        foreach (const Value& elt, m)
        {
          ser.template serialize<K>("key", elt.first);
          ser.template serialize<V>("value", elt.second);
        }
      }
    };
  }
}

#endif
