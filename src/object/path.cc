/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <exception> // bad_alloc
#include <libport/cerrno>
#include <libport/cstring>
#include <libport/fcntl.h>
#include <libport/file-system.hh>
#include <libport/format.hh>
#include <libport/lexical-cast.hh>
#include <libport/sys/stat.h>
#include <libport/sys/types.h>
#include <libport/unistd.h> // For stat, getcwd

#include <object/symbols.hh>
#include <urbi/object/directory.hh>
#include <urbi/object/file.hh>
#include <urbi/object/path.hh>
#include <urbi/runner/raise.hh>

namespace urbi
{
  namespace object
  {
    /*--------------.
    | C++ methods.  |
    `--------------*/

    const Path::value_type& Path::value_get() const
    {
      return path_;
    }

    void Path::value_set(const value_type& val)
    {
      path_ = val;
    }


    /*---------------.
    | Urbi methods.  |
    `---------------*/

    // Construction

    Path::Path()
      : path_("/")
    {
      proto_add(proto);
    }

    Path::Path(rPath model)
      : path_(model->value_get())
    {
      proto_add(model);
    }

    Path::Path(const std::string& value)
      : path_(value)
    {
      proto_add(proto ? rObject(proto) : Object::proto);
    }

    void Path::init(const std::string& path)
    {
      path_ = path;
    }

    bool
    Path::operator<=(const rPath& rhs) const
    {
      return path_.to_string() <= rhs->path_.to_string();
    }


    /*---------------------.
    | Global information.  |
    `---------------------*/

    rPath
    Path::cwd()
    {
      return new Path(libport::get_current_directory());
    }

    // Stats

    bool
    Path::absolute() const
    {
      return path_.absolute_get();
    }

    struct stat Path::stat() const
    {
      struct stat res;

      if (::stat(path_.to_string().c_str(), &res))
        handle_any_error();

      return res;
    }

    bool Path::is_dir() const
    {
      return stat().st_mode & S_IFDIR;
    }

    bool Path::is_reg() const
    {
      return stat().st_mode & S_IFREG;
    }

    bool Path::exists() const
    {
      if (!::access(path_.to_string().c_str(), F_OK))
        return true;
      if (errno == ENOENT)
        return false;
      handle_any_error();
    }

    bool Path::readable() const
    {
      if (!::access(path_.to_string().c_str(), R_OK))
        return true;
      if (errno == EACCES)
        return false;
      handle_any_error();
    }

    bool Path::writable() const
    {
      if (!::access(path_.to_string().c_str(), W_OK))
        return true;
      if (errno == EACCES || errno == EROFS)
        return false;
      handle_any_error();
    }

    // Operations.
    std::string Path::basename() const
    {
      return path_.basename();
    }

    rPath Path::cd() const
    {
      if (chdir(as_string().c_str()))
        handle_any_error();
      return cwd();
    }

    rPath Path::path_concat(rPath other) const
    {
      try
      {
        return new Path(path_ / other->path_);
      }
      catch (const libport::path::invalid_path& e)
      {
        RAISE(e.what());
      }
    }

    rPath Path::string_concat(rString other) const
    {
      return path_concat(new Path(other->value_get()));
    }

    OVERLOAD_TYPE(concat_bouncer, 1, 1,
                  Path, &Path::path_concat,
                  String, &Path::string_concat);

    rPath Path::dirname() const
    {
      return new Path(path_.dirname());
    }

    rObject Path::open() const
    {
      if (is_dir())
        return new Directory(path_);
      if (is_reg())
        return new File(path_);
      FRAISE("unsupported file type: %s", path_);
    }

    // Conversions

    std::string Path::as_string() const
    {
      return path_.to_string();
    }

    std::string Path::as_printable() const
    {
      return libport::format("Path(\"%s\")", as_string());
    }

    rList Path::as_list() const
    {
      List::value_type res;
      foreach (const std::string& c, path_.components())
        res << new String(c);
      return new List(res);
    }


    /*----------.
    | Details.  |
    `----------*/

    void Path::handle_any_error() const
    {
      FRAISE("%1%: %2%", strerror(errno), path_);
    }

    /*-----------------.
    | Binding system.  |
    `-----------------*/

    void Path::initialize(CxxObject::Binder<Path>& bind)
    {
      bind(SYMBOL(EQ_EQ),
           static_cast<bool (self_type::*)(const rObject&) const>
           (&self_type::operator==));
      bind(SYMBOL(SLASH), concat_bouncer);

#define DECLARE(Urbi, Cxx)                      \
      bind(SYMBOL(Urbi), &Path::Cxx)

      DECLARE(LT_EQ, operator<=);
      DECLARE(absolute, absolute);
      DECLARE(asList, as_list);
      DECLARE(asPrintable, as_printable);
      DECLARE(asString, as_string);
      DECLARE(basename, basename);
      DECLARE(cd, cd);
      DECLARE(cwd, cwd);
      DECLARE(dirname, dirname);
      DECLARE(exists, exists);
      DECLARE(init, init);
      DECLARE(isDir, is_dir);
      DECLARE(isReg, is_reg);
      DECLARE(open, open);
      DECLARE(readable, readable);
      DECLARE(writable, writable);
    }

    URBI_CXX_OBJECT_REGISTER(Path)
      : path_(WIN32_IF("C:\\", "/"))
    {}

  }
}
