/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef OBJECT_PROCESS_HH
# define OBJECT_PROCESS_HH

# include <libport/fd-stream.hh>

# include <urbi/object/cxx-object.hh>

namespace urbi
{
  namespace object
  {
    class Process: public CxxObject
    {

    /*---------------------------.
    | Construction / Destruction |
    `---------------------------*/

    public:
      typedef std::vector<std::string> arguments_type;
      Process(const std::string& binary,
              const arguments_type& argv);
      Process(rProcess model);
      virtual ~Process();

    /*-------------.
    | Urbi methods |
    `-------------*/

    public:
      void init(const std::string& binary,
                const arguments_type& argv);
      void run();
      /// Run, writing stderr and stdout to filename.
      void runTo(const std::string& filename);
      void join() const;
      void kill();
      bool done() const;
      rObject status() const;
      std::string name() const;
      virtual std::string as_string() const;

    /*--------.
    | Details |
    `--------*/

    private:
      static void monitor_children();
      void run_(boost::optional<std::string> outFile
                 = boost::optional<std::string>());
      std::string name_;
      pid_t pid_;
      std::string binary_;
      arguments_type argv_;
      int status_;
      URBI_CXX_OBJECT_(Process);
    };
  }
}

#endif
