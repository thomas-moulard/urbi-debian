/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef OBJECT_HASH_SLOTS_HH
# define OBJECT_HASH_SLOTS_HH

# include <libport/hash.hh>

# include <object/slots.hh>

namespace object
{
  class HashSlots: public Slots
  {
    public:
      typedef boost::unordered_map<key_type, rObject> content_type;
      typedef content_type::iterator iterator;
      typedef content_type::const_iterator const_iterator;

      inline bool set(const key_type& key, value_type v);
      inline void update(const key_type& key, value_type v);
      inline value_type get(const key_type& key) const;
      inline void erase(const key_type& key);
      inline bool has(const key_type& key) const;
      inline std::pair<bool, iterator> where(const key_type& key);
      iterator begin();
      iterator end();
      const_iterator begin() const;
      const_iterator end() const;

    private:
      content_type content_;
  };
}

# include <object/hash-slots.hxx>

#endif
