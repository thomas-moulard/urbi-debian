/*
 * Copyright (C) 2009, 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/**
 ** \file object/tag.hh
 ** \brief Definition of the Urbi object tag.
 */

#ifndef OBJECT_TAG_HH
# define OBJECT_TAG_HH

# include <urbi/object/cxx-object.hh>
# include <urbi/object/fwd.hh>
# include <sched/tag.hh>

namespace urbi
{
  namespace object
  {
    class Tag : public object::CxxObject
    {
    public:
      typedef sched::rTag value_type;

      Tag();
      Tag(const value_type& value);
      Tag(rTag model);
      value_type& value_get();
      const value_type& value_get() const;

      void block(const objects_type&);
      void init(const objects_type& args);
      void freeze();
      libport::Symbol name() const;
      void name_set(libport::Symbol s);
      static rTag new_flow_control(const objects_type&);

      /// Return a tag for the current urbiscript scope.
      static rTag scope();

      typedef sched::prio_type priority_type;
      priority_type priority() const;
      priority_type priority_set(priority_type);

      /// Stop the tagged jobs, forcing \a payload as value.
      void stop(const rObject& payload = void_class);
      void stop(const objects_type&);
      void unblock();
      void unfreeze();
      bool frozen() const;
      bool blocked() const;

      /// Return, potentially creating first, the enter event for \a this.
      rObject enter();
      /// Return, potentially creating first, the leave event for \a this.
      rObject leave();

      /// Trigger \a this' enter event.
      void triggerEnter();
      /// Trigger \a this' leave event.
      void triggerLeave();

      /// Manipulate parent tag.
      rTag parent_get();

    private:
      value_type value_;
      rTag parent_;

    URBI_CXX_OBJECT_(Tag);
    };

  } // namespace object
}

# include <urbi/object/cxx-object.hxx>
# include <urbi/object/tag.hxx>

#endif // !OBJECT_TAG_HH
