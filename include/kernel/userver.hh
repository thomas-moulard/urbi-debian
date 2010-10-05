/*
 * Copyright (C) 2009, 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/// \file kernel/userver.hh
/// \brief Definition of the UServer class.

#ifndef KERNEL_USERVER_HH
# define KERNEL_USERVER_HH

# include <memory>
# include <sstream>

# include <libport/config.h>
# include <libport/attributes.hh>

# if LIBPORT_HAVE_WINDOWS_H
// Without this, windows.h may include winsock.h, which will conflict with
// winsock2.h when we will try to include it.
#  define WIN32_LEAN_AND_MEAN
# endif

# include <libport/fwd.hh>
# include <libport/compiler.hh>
# include <libport/file-library.hh>
# include <libport/lockable.hh>
# include <libport/ufloat.h>
# include <libport/utime.hh>
# include <libport/pthread.h>
# include <libport/synchronizer.hh>

# include <kernel/fwd.hh>
# include <kernel/utypes.hh>
# include <urbi/export.hh>
# include <urbi/object/object.hh>
# include <urbi/urbi-root.hh>

namespace kernel
{
  /// Global variable for the server
  extern URBI_SDK_API class UServer* urbiserver;

  /// The current server.
  UServer& server();

  /// Convenience wrapper to get urbiserver's current runner.
  runner::Runner& runner();

  /// Convenience wrapper to get urbiserver's current interpreter.
  runner::Interpreter& interpreter();

  /// Convenience wrapper to get urbiserver's current scheduler.
  sched::Scheduler& scheduler();

  //! Handle all Urbi system processing.
  /*! There must be one UServer defined in the program and it must be overloaded
      to make it specific to the particular robot.

      UServer is used to store the UConnection list and the UDevice list.
      This object does all the internal processing of Urbi and handles the pool
      of UCommand's.
  */
  class URBI_SDK_API UServer
  {
  public:
    UServer(UrbiRoot& urbi_root);
    virtual ~UServer();

  public:
    //! Initialization of the server. Displays the header message & init stuff
    /*! This function must be called once the server is operational and
     * able to print messages. It is a requirement for Urbi compliance to print
     * the header at start, so this function *must* be called. Beside, it also
     * do initalization work for the devices and system variables.
     *
     * \param interactive  whether the ghostconnection is interactive.
     *                     FIXME: this interface is unpleasant, something
     *                     nicer is needed.
     */
    void initialize(bool interactive = false);

    /// Accessors for interactive value bound to System.interactive,
    // also define the behavior when receiving SIGINT.
    bool interactive_get() const;
    void interactive_set(bool);

    /// Support a kernel and a user mode.
    ///
    /// Exceptions may not be launched in user mode, they are hard
    /// error.  This is used to catch error when the system is
    /// incomplete, say when an error occurs when loading urbi/urbi.u.
    /// Therefore, "of course", defaults to mode_kernel.
    enum mode_type
    {
      mode_kernel,
      mode_user,
    };
    ATTRIBUTE_R(mode_type, mode);

  public:
    /// Process the jobs.
    /// \return the time when should be called again.
    libport::utime_t work();

    /// Set the system.args list in Urbi.
    void main(int argc, const char* argv[]);

    /// Package information about this server.
    static const libport::PackageInfo& package_info();

    //! Overload this function to return the running time of the server.
    /*! The running time of the server must be in microseconds.
     */
    virtual libport::utime_t getTime() const = 0;

    /// Path to explore when looking for .u files.
    libport::file_library search_path;

    /// Return the full file name, handle paths.
    /// Return \a f on failure.
    virtual std::string find_file(const libport::path& path) const;

    /// Type of UCommandQueue
    enum QueueType
    {
      /// The UCommandQueue contains urbiscript code.
      QUEUE_URBI,
      /// The UCommandQueue contains data, not to be messed with.
      QUEUE_DATA
    };

    /// Load a file into the connection.
    /// Returns UFAIL if anything goes wrong, USUCCESS otherwise.
    virtual UErrorValue load_file(const std::string& file_name,
                                  UQueue& loadQueue,
                                  QueueType t = QUEUE_URBI);

    /// Load \a fn in the ghostqueue.
    UErrorValue load_init_file(const char* fn);

    /// Same but calls init_error on errors.
    void xload_init_file(const char* fn);

    /// Save content to a file
    /// This function must be redefined by the robot-specific server.
    /// Returns UFAIL if anything goes wrong, USUCCESS otherwise.
    virtual UErrorValue save_file(const std::string& filename,
                                  const std::string& content) = 0;

    //! Overload this function to specify how your system will reboot
    virtual void reboot() = 0;

    //! Overload this function to specify how your system will shutdown
    virtual void shutdown();

    //! Function called before work
    /*! Redefine this virtual function if you need to do pre-processing before
      the work function starts.
    */
    virtual void beforeWork();

    //! Function called after work
    /*! Redefine this virtual function if you need to do post-processing
      before the work function ends.
    */
    virtual void afterWork();

    /// Display a message on the robot console.
    void display(const char*);

    //! Accessor for lastTime_.
    libport::utime_t lastTime();

    //! Update lastTime_ to current time.
    //! Update the server's time using the robot-specific implementation
    /*! It is necessary to have an update of the server time to
     increase the performance of successive calls to getTime.
     It allows also to see a whole processing session (like the
     processing of the command tree) as occuring AT the same time,
     from the server's point of view.
     */
    void updateTime();

    /*----------------.
    | Configuration.  |
    `----------------*/

    ATTRIBUTE_RW(bool, opt_banner);

    /*--------------.
    | Connections.  |
    `--------------*/

    public:
    /// Add a new connection to the connection list.
    /// Take ownership on c. Also perform some error testing on the connection
    /// value and UError return code
    /// \precondition c != 0
    void connection_add(UConnection* c);

    /// Remove from connection_.
    /// Also perform some error testing on the connection
    /// value and UError return code
    /// Destroy \a c.
    void connection_remove(UConnection* c);

    /// \returns A usual connection to stop dependencies.
    ///          (kernel/ghost-connection.hh is not public).
    UConnection& ghost_connection_get();


    /*--------------------.
    | Scheduler, runner.  |
    `--------------------*/
  public:
    const sched::Scheduler& scheduler_get() const;
    sched::Scheduler& scheduler_get();

    runner::Runner& getCurrentRunner() const;
    runner::Runner* getCurrentRunnerOpt() const;

    boost::asio::io_service& get_io_service();

    /// Wake up from main loop.
    void wake_up();

    /*--------------------------.
    | Asynchronous scheduling.  |
    `--------------------------*/

  public:
    void schedule(urbi::object::rObject target, libport::Symbol method,
                  const urbi::object::objects_type& args = urbi::object::objects_type());

  private:
    struct AsyncJob
    {
      AsyncJob(urbi::object::rObject t, libport::Symbol m,
               const urbi::object::objects_type& a);

      urbi::object::rObject target;
      libport::Symbol method;
      urbi::object::objects_type args;
    };
    std::vector<AsyncJob> async_jobs_;
    libport::Lockable async_jobs_lock_;

  protected:
    //! Overload this function to specify how your robot is displaying messages.
    virtual void effectiveDisplay(const char*) = 0;

    libport::Synchronizer synchronizer_;
  private:
    // Pointer to stop the header dependency.
    sched::Scheduler* scheduler_;


  private:
    /// \{ Various parts of @c UServer::work.
    /// Scan currently opened connections for deleting marked commands or
    /// killall order
    void work_handle_stopall_();
    void work_test_cpuoverload_();
    /// \}

    /// Call synchronizer::check() and yield in a loop.
    void handle_synchronizer_();
  public:
    /// Stops all commands in all connections.
    bool stopall;

    /// True iff current thread is different from server thread Id.
    bool isAnotherThread() const;

    libport::Synchronizer& synchronizer_get();

    UrbiRoot& urbi_root_get();
  private:
    /// Store the time on the last call to updateTime().
    libport::utime_t lastTime_;

    /// List of active connections: includes one UGhostConnection.
    // FIXME: This is meant to become a runner::Job and move out of this class.
    /// A pointer to stop dependencies.
    std::auto_ptr<kernel::ConnectionSet> connections_;

    /// The ghost connection used for urbi.u, URBI.INI, etc.
    // Does not need to be an auto_ptr, as it is stored in connections_
    // which handles memory management.
    UGhostConnection* ghost_;

    /// Whether this server is in interactive mode.
    bool interactive_;

    /// Store the server thread Id.
    pthread_t thread_id_;

    /// Urbi SDK installation
    UrbiRoot& urbi_root_;

    /// Socket pair used to wake us up
    std::pair<libport::Socket*, libport::Socket*> wake_up_pipe_;
  };

}

// Disable debug traces until we find a better means to do it.
# define DEBUG(Msg) ((void) 0)

# include <kernel/userver.hxx>

#endif // !KERNEL_USERVER_HH
