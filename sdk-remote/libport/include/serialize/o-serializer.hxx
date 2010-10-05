/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef O_SERIALIZER_HXX
# define O_SERIALIZER_HXX

namespace libport
{
  namespace serialize
  {
    template <class Exact>
    OSerializer<Exact>::OSerializer(std::ostream& output)
      : stream_(output)
    {}

    template <class Exact>
    OSerializer<Exact>::~OSerializer()
    {}

    template <class Exact>
    template <typename T>
    void
    OSerializer<Exact>::serialize(const std::string& name,
                                  typename traits::Arg<T>::res v)
    {
      Exact::template Impl<T>::put(name, v, stream_,
                                   static_cast<Exact&>(*this));
    }

    template <typename T>
    inline
    void
    write_(std::ostream& o, T t)
    {
      o.write(reinterpret_cast<char*>(&t), sizeof t);
    }
  }

}

#endif
