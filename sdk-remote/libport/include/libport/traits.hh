/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef LIBPORT_TRAITS_HH
# define LIBPORT_TRAITS_HH

# include <boost/tr1/type_traits.hpp>

# include <libport/meta.hh>

namespace libport
{
  namespace traits
  {
    /*------.
    | IsPOD |
    `------*/

    /// Whether 'T' is a POD.
    template <typename T>
    struct IsPOD
    {
      static const bool res = std::tr1::is_pod<T>::value;
    };

    /*---.
    | Id |
    `---*/

    /// Identity
    template <typename T>
    struct Id
    {
      typedef T res;
    };

    /*------.
    | Const |
    `------*/

    /// Constify T
    template <typename T>
    struct Const
    {
      typedef const T res;
    };

    /*----.
    | Ptr |
    `----*/

    /// Make T a pointer
    template <typename T>
    struct Ptr
    {
      typedef T* res;
    };

    /*----.
    | Ref |
    `----*/

    /// Make T a reference
    template <typename T>
    struct Ref
    {
      typedef T& res;
    };

    /*---------.
    | ConstPtr |
    `---------*/

    /// Make T a const pointer
    template <typename T>
    struct ConstPtr
    {
      typedef typename meta::Compose<Ptr, Const>::Res<T>::res res;
    };

    /*---------.
    | ConstRef |
    `---------*/

    /// Make T a const reference
    template <typename T>
    struct ConstRef
    {
      typedef typename meta::Compose<Ref, Const>::Res<T>::res res;
    };

    /*-------------------.
    | Remove reference.  |
    `-------------------*/

    /// Remove reference mark from T
    template <typename T>
    struct remove_reference
    {
      typedef T type;
    };

    template <typename T>
    struct remove_reference<T&>
    {
      typedef T type;
    };

    /*----.
    | Arg |
    `----*/

    /// Give the best type for passing a T as argument
    /* That is, T if T is a POD, const T& otherwise.
     *
     */
    template <typename T>
    struct Arg
    {
      typedef typename meta::If<IsPOD<T>::res, T, const T&>::res res;
    };
  }
}

#endif
