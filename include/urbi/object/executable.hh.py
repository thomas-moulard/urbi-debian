#! /usr/bin/python

prototypes = ''
implems = ''

def collapse(l, f):
    return ''.join(map(f, l))

for narg in range(9):
    args = range(1, narg + 1)
    types = collapse(args, lambda n: ', typename T%s' % n)
    formals = collapse(args, lambda n: ', T%s arg%s' % (n, n))
    push = collapse(args, lambda n: '\n           << CxxConvert<T%s>::from(arg%s)' % (n, n))
    prototypes += \
    '''
      template <typename S%s>
      rObject operator()(S self%s);
''' % (types, formals)
    implems += \
'''
    template <typename S%s>
    rObject Executable::operator()(S self%s)
    {
      objects_type args;
      args << CxxConvert<S>::from(self)%s;
      return operator()(args);
    }
''' % (types, formals, push)

print '''#ifndef OBJECT_EXECUTABLE_HH
# define OBJECT_EXECUTABLE_HH

# include <urbi/object/cxx-object.hh>
# include <urbi/export.hh>

namespace urbi
{
  namespace object
  {
    class URBI_SDK_API Executable: public CxxObject
    {
    public:
      Executable();
      Executable(rExecutable model);
      virtual rObject operator() (object::objects_type args);
      %s

      URBI_CXX_OBJECT_(Executable);
    };
  }
}

# include <urbi/object/cxx-object.hxx>

namespace urbi
{
  namespace object
  {
  %s
  }
}

#endif''' % (prototypes, implems)
