/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/// \file libuobject/main.cc

#include <libport/cstdio>
#include <libport/unistd.h>
#include <libport/cerrno>

#include <iostream>
#include <list>
#include <sstream>

#include <libport/debug.hh>
#include <libport/cli.hh>
#include <libport/containers.hh>
#include <libport/foreach.hh>
#include <libport/lexical-cast.hh>
#include <libport/package-info.hh>
#include <libport/program-name.hh>
#include <libport/sysexits.hh>

#include <urbi/package-info.hh>
#include <urbi/uexternal.hh>
#include <urbi/umain.hh>
#include <urbi/umessage.hh>
#include <urbi/uobject.hh>
#include <urbi/urbi-root.hh>
#include <urbi/usyncclient.hh>
#include <libuobject/remote-ucontext-impl.hh>
using libport::program_name;

GD_CATEGORY(urbi-launch);

namespace urbi
{
  static impl::RemoteUContextImpl* defaultContext;

  UCallbackAction
  debug(const UMessage& msg)
  {
    GD_SWARN("unexpected message " << msg);
    return URBI_CONTINUE;
  }

  UCallbackAction
  static
  endProgram(const UMessage& msg)
  {
    GD_SWARN("got a disconnection message: "<< msg);
    exit(1);
    return URBI_CONTINUE; //stupid gcc
  }

  static
  void
  usage()
  {
    std::cout <<
      "usage:\n" << libport::program_name() << " [OPTION]...\n"
      "\n"
      "Options:\n"
      "  -b, --buffer SIZE     input buffer size"
		 << " [" << UAbstractClient::URBI_BUFLEN << "]\n"
      "  -h, --help            display this message and exit\n"
      "  -H, --host ADDR       server host name"
                 << " [" << UClient::default_host() << "]\n"
      "      --server          put remote in server mode\n"
      "      --no-sync-client  Use UClient instead of USyncClient\n"
      "  -p, --port PORT       tcp port URBI will listen to"
		 << " [" << UAbstractClient::URBI_PORT << "]\n"
      "  -r, --port-file FILE  file containing the port to listen to\n"
      "  -V, --version         print version information and exit\n"
      "  -d, --disconnect      exit program if disconnected(defaults)\n"
      "  -s, --stay-alive      do not exit program if disconnected\n"
      "  --describe            describe loaded UObjects and exit\n"
      "  --describe-file FILE  write the list of present UObjects to FILE\n"
		 << libport::exit (EX_OK);
  }

  static
  void
  version()
  {
    std::cout << urbi::package_info() << std::endl
              << libport::exit (EX_OK);
  }

  typedef std::vector<std::string> files_type;
  int
  initialize(const std::string& host, int port, size_t buflen,
	     bool exitOnDisconnect, bool server, const files_type& files,
             bool useSyncClient)
  {
    std::cerr << program_name()
	      << ": " << urbi::package_info() << std::endl
	      << program_name()
	      << ": Remote Component Running on "
	      << host << " " << port << std::endl;
    if (useSyncClient)
    {
      USyncClient::options o;
      o.server(server);
      new USyncClient(host, port, buflen, o);
    }
    else
    {
      std::cerr << "#WARNING: the no-sync-client mode is dangerous.\n"
	"Any attempt to use synchronous operation will crash your program."
        << std::endl;
      UClient::options o;
      o.server(server);
      setDefaultClient(new UClient(host, port, buflen, o));
    }
    if (exitOnDisconnect)
    {
      if (!getDefaultClient() || getDefaultClient()->error())
	std::cerr << "ERROR: failed to connect, exiting..." << std::endl
		  << libport::exit(1);
      getDefaultClient()->setClientErrorCallback(callback(&endProgram));
    }
    if (!getDefaultClient() || getDefaultClient()->error())
      return 1;

#ifdef LIBURBIDEBUG
    getDefaultClient()->setWildcardCallback(callback(&debug));
#else
    getDefaultClient()->setErrorCallback(callback(&debug));
#endif

    // Wait for client to be connected if in server mode.
    while (getDefaultClient()
           && !getDefaultClient()->error()
           && !getDefaultClient()->isConnected())
      usleep(20000);

    defaultContext = new impl::RemoteUContextImpl(
      (USyncClient*)dynamic_cast<UClient*>(getDefaultClient()));
    // Waiting for connectionID.
    while (getDefaultClient()
           && getDefaultClient()->connectionID() == "")
    usleep(5000);
    // Initialize in the correct thread.
    getDefaultClient()->notifyCallbacks(UMessage(*getDefaultClient(), 0,
                             externalModuleTag.c_str(),
                             ("[" + boost::lexical_cast<std::string>(UEM_INIT)
                               + "]").c_str()
                             ));
    // Load initialization files.
    foreach (const std::string& file, files)
      getDefaultClient()->sendFile(file);
    return 0;
  }

  namespace
  {
    static
    void
    argument_with_option(const char* longopt,
                         char shortopt,
                         const std::string& val)
    {
      std::cerr
        << program_name()
        << ": warning: arguments without options are deprecated"
        << std::endl
        << "use `-" << shortopt << ' ' << val << '\''
        << " or `--" << longopt << ' ' << val << "' instead"
        << std::endl
        << "Try `" << program_name() << " --help' for more information."
        << std::endl;
    }

  }


  URBI_SDK_API int
  main(const libport::cli_args_type& args, UrbiRoot&, bool block, bool)
  {
    std::string host = UClient::default_host();
    bool exitOnDisconnect = true;
    int port = UAbstractClient::URBI_PORT;
    bool server = false;
    bool useSyncClient = true;
    bool describeMode = false;
    std::string describeFile;
    size_t buflen = UAbstractClient::URBI_BUFLEN;
    // Files to load
    files_type files;

    // The number of the next (non-option) argument.
    unsigned argp = 1;
    for (unsigned i = 1; i < args.size(); ++i)
    {
      const std::string& arg = args[i];
      if (arg == "--buffer" || arg == "-b")
	buflen = libport::convert_argument<size_t>(args, i++);
      else if (arg == "--disconnect" || arg == "-d")
	exitOnDisconnect = true;
      else if (arg == "--file" || arg == "-f")
        files.push_back(libport::convert_argument<const char*>(args, i++));
      else if (arg == "--stay-alive" || arg == "-s")
	exitOnDisconnect = false;
      else if (arg == "--help" || arg == "-h")
	usage();
      else if (arg == "--host" || arg == "-H")
	host = libport::convert_argument<std::string>(args, i++);
      else if (arg == "--no-sync-client")
        useSyncClient = false;
      else if (arg == "--port" || arg == "-p")
	port = libport::convert_argument<unsigned>(args, i++);
      else if (arg == "--port-file" || arg == "-r")
	port =
          (libport::file_contents_get<int>
           (libport::convert_argument<const char*>(args, i++)));
      else if (arg == "--server")
	server = true;
      // FIXME: Remove -v some day.
      else if (arg == "--version" || arg == "-V" || arg == "-v")
	version();
      else if (arg == "--describe")
	describeMode = true;
      else if (arg == "--describe-file")
         describeFile = libport::convert_argument<const char*>(args, i++);
      else if (arg[0] == '-')
	libport::invalid_option(arg);
      else
	// A genuine argument.
	switch (argp++)
	{
	  case 1:
            argument_with_option("host", 'H', args[i]);
	    host = args[i];
	    break;
	  case 2:
            argument_with_option("port", 'p', args[i]);
	    port = libport::convert_argument<unsigned>("port", args[i]);
	    break;
	  default:
	    std::cerr << "unexpected argument: " << arg << std::endl
		      << libport::exit(EX_USAGE);
	}
    }

    if (describeMode)
    {
      foreach(baseURBIStarter* s, baseURBIStarter::list())
        std::cout << s->name << std::endl;
      foreach(baseURBIStarterHub* s, baseURBIStarterHub::list())
        std::cout << s->name << std::endl;
      return 0;
    }
    if (!describeFile.empty())
    {
      std::ofstream of(describeFile.c_str());
      foreach(baseURBIStarter* s, baseURBIStarter::list())
        of << s->name << std::endl;
      foreach(baseURBIStarterHub* s, baseURBIStarterHub::list())
        of << s->name << std::endl;
    }
    initialize(host, port, buflen, exitOnDisconnect, server, files,
               useSyncClient);

    if (block)
      while (true)
        usleep(30000000);
    return 0;
  }

}
