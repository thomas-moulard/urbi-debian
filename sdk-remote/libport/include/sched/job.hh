/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/**
 ** \file sched/job.hh
 ** \brief Definition of Job.
 */

#ifndef SCHED_JOB_HH
# define SCHED_JOB_HH

# include <iosfwd>
# include <list>

# include <boost/any.hpp>

# include <libport/symbol.hh>
# include <libport/utime.hh>

# include <sched/coroutine.hh>
# include <sched/export.hh>
# include <sched/fwd.hh>
# include <sched/tag.hh>

namespace sched
{

  enum job_state
  {
    to_start,            ///< Job needs to be started
    running,             ///< Job is waiting for the CPU
    sleeping,            ///< Job is sleeping until a specified deadline
    waiting,             ///< Job is waiting for changes to happen
    joining,             ///< Job is waiting for another job to terminate
    zombie,              ///< Job wants to be dead but isn't really yet
  };

  /// State names to string, for debugging purpose.
  const char* name(job_state);

  /// Dump \a s on \a o for debugging purpose.
  std::ostream& operator<<(std::ostream& o, job_state s);

  /// A Job represents a thread of control, implemented using coroutines.
  /// The scheduler decides which job to launch and resumes its execution.
  /// The lifetime of a job is the following
  ///
  /// \li   Job()                  : job creation
  ///
  /// \li   start_job()            : add job to the scheduler; the scheduler
  ///                                will call run(), and the job will
  ///                                yield() itself into the scheduler run
  ///                                queue during the course of work()
  ///
  /// \li   work()                 : this method, which must be overridden,
  ///                                does the real work
  ///
  /// Due to the way coroutines work, a job may not delete itself
  /// as once the coroutine structure has been freed, it is illegal to
  /// continue its execution, even only to switch to another coroutine.
  /// And if a job switches back to the scheduler before terminating its
  /// destructor, the coroutine structure will not be freed. Because of
  /// that, it is necessary for a job to be deleted by another one or
  /// by the scheduler.
  ///
  /// An additional constraint comes from the fact that even when a job
  /// has terminated its work, its structure may not always be deleted.
  /// For example, other jobs may have kept a reference to this job and
  /// expect to retrieve some information (such as a result) after the
  /// job termination.
  ///
  /// We have several ways of dealing with those constraints and not leaking
  /// memory upon a job termination:
  ///
  /// \li 1. Make sure the job spits out all the useful information concerning
  ///        its state by the way of inter-job communication before asking to
  ///        be deleted.
  ///
  /// \li  2. Make sure the job is kept alive as long as there is at least one
  ///         reference onto it, and that it gets deleted from another
  ///         coroutine, or from the main one.

  class SCHED_API Job: public libport::RefCounted
  {
  public:
    /// Create a job from another one.
    ///
    /// \param model The parent job. The scheduler and tags will be inherited
    //         from it.
    ///
    /// \param name The name of the new job, or a name derived from \a model
    ///        if none is provided.
    Job(const Job& model, const libport::Symbol& name);

    /// Create a new job.
    ///
    /// \param scheduler The scheduler to which this job will be attached.
    ///
    /// \param name The name of the new job, or an automatically created
    ///        one if none is provided.
    Job(Scheduler& scheduler, const libport::Symbol& name);

    /// Job destructor.
    ///
    /// The destructor is in charge of unscheduling the job in the
    /// scheduler.
    virtual ~Job();

    /// Get this job scheduler.
    ///
    /// \return A reference on the job scheduler.
    Scheduler& scheduler_get() const;

    /// Dump internal data (for the developper).
    ///
    /// \return \a o
    std::ostream& dump(std::ostream& o) const;

    /// Get the underlying coroutine corresponding to this job.
    ///
    /// \return The coroutine structure.
    Coro* coro_get() const;

    /// Has this job terminated?
    ///
    /// \return True if this job is in the \c zombie state.
    bool terminated() const;

    /// Start job by adding it to the scheduler.
    void start_job();

    /// Run the job. This function is called from the scheduler.
    void run();

    /// Terminate the job. The job will execute its cleanup method
    /// and inform the scheduler that it is ready to be destroyed.
    /// Must not be called by a child to an ancester of his.
    void terminate_now();

    /// Asynchronous termination.  Allows a child to kill its parent.
    /// Will be killed at the next cycle.
    void terminate_asap();

    /// Register this Job on its Scheduler so that it is rescheduled
    /// during the next cycle. This should be called from the
    /// currently scheduled job only but must be kept visible to be
    /// callable from the primitives.
    /// \sa yield_until(), yield_until_terminated(),
    /// yield_until_things_changed()
    void yield();

    /// As yield(), but ask not to be woken up before the deadline.
    /// \sa yield(), yield_until_terminated(), yield_until_things_changed(),
    ///     yield_for().
    void yield_until(libport::utime_t deadline);

    /// As yield(), but ask not to be woken up before the end of the delay.
    /// \sa yield(), yield_until(), yield_until_terminated(),
    ///     yield_until_things_changed()
    void yield_for(libport::utime_t delay);

    /// Wait for another job to terminate before resuming execution of
    /// the current one. If the other job has already terminated, the
    /// caller will continue its execution.
    ///
    /// \param other The job to wait for. It is allowed to specify the
    ///        same Job as the target and the waiting job, in which case
    ///        the Job will only be woken up by an asynchronous exception.
    ///
    /// \sa yield(), yield_for(), yield_until(), yield_until_things_changed()
    void yield_until_terminated(Job& other);

    /// Same as \p yield_until_terminated above(), but wait for every
    /// job in the collection.
    void yield_until_terminated(const jobs_type& jobs);

    /// Wait for any other task to be scheduled.
    /// \sa yield(), yield_for(), yield_until_terminated(), yield_until()
    void yield_until_things_changed();

    /// Mark the current job as side-effect free.
    ///
    /// \param s True if the job is now side-effect free.
    ///
    /// Indicate whether the current state of a job may influence other
    /// parts of the system. This is used by the scheduler to choose
    /// whether other jobs need scheduling or not. The default value
    /// for \c side_effect_free is false.
    void side_effect_free_set(bool s);

    /// Is the current job side-effect free?
    ///
    /// \return True if the job is currently side-effect free.
    bool side_effect_free_get() const;

    /// Raise an exception next time this job will be resumed.
    ///
    /// \param e The exception to throw when the job will be scheduled
    ///        again.
    void async_throw(const exception& e);

    /// Maybe raise a deferred exception. Must be called from the scheduler
    /// while resuming the job execution. For example, StopException
    /// may be raised from here if the job has been blocked by a tag.
    void check_for_pending_exception();

    /// Get the job name
    ///
    /// \return The job name as set from the constructor.
    const libport::Symbol& name_get() const;

    /// Throw an exception if the stack space for this job is near
    /// exhaustion.
    void check_stack_space();

    /// Is the job frozen?
    ///
    /// \return This depends from the job tags state.
    virtual bool frozen() const = 0;

    /// Check if the job holds a tag.
    ///
    /// \return 0 if the job does not hold the tag,
    ///         the position of the tag in the tag stack
    ///         (starting from 1) otherwise.
    virtual size_t has_tag(const Tag& tag, size_t max_depth = (size_t)-1)
      const = 0;

    /// Get the current job state.
    ///
    /// \return The current job state.
    job_state state_get() const;

    /// Set the current job state
    ///
    /// \param state The new job state.
    void state_set(job_state state);

    /// Get this job deadline if it is sleeping.
    ///
    /// \return The date on which this job needs to be awoken.
    ///
    /// This function must not be called unless the job is in the
    /// \c sleeping state.
    libport::utime_t deadline_get() const;

    /// Check if the job can be interrupted.
    ///
    /// \return True if the job cannot be interrupted right now because
    ///         it is executing an atomic operation.
    bool non_interruptible_get() const;

    /// Indicate if the job can be interrupted or not
    ///
    /// \param ni True if the job must not be interrupted until further
    ///        notice.
    void non_interruptible_set(bool ni);

    /// Remember the time we have been frozen since if not remembered
    /// yet.
    ///
    /// \param current_time The current time.
    void notice_frozen(libport::utime_t current_time);

    /// Note that we are not frozen anymore.
    ///
    /// \param current_time The current time.
    void notice_not_frozen(libport::utime_t current_time);

    /// Return the origin since we have been frozen.
    ///
    /// \return 0 if we have not been frozen, the date since we have
    ///         been frozen otherwise.
    libport::utime_t frozen_since_get() const;

    /// Return the current time shift.
    ///
    /// \return The value to subtract from the system time to get the
    ///         unfrozen time of this runner.
    libport::utime_t time_shift_get() const;

    /// Set the current time shift.
    ///
    /// \param ts The new time shift. This should probably only be used
    ///           at runner creation time.
    void time_shift_set(libport::utime_t ts);

    /// Check and maybe register the fact that a tag has been stopped.
    ///
    /// \param tag The tag that has been stopped.
    ///
    /// \param payload The data to embed in the StopException.
    virtual void register_stopped_tag(const Tag& tag,
                                      const boost::any& payload);

    /// Check whether the job has a pending exception.
    ///
    /// \return true if the job has a pending exception.
    bool has_pending_exception() const;

    /// Get the current job priority.
    ///
    /// \return The job priority, as previously computed.
    virtual prio_type prio_get() const = 0;

    /// Ensure proper cleanup.
    virtual void terminate_cleanup();

    /// Number of jobs created and not yet destroyed.
    ///
    /// \return The number of jobs.
    static unsigned int alive_jobs();

    /// Helper to unregister all children upon destruction.
    class SCHED_API Collector: public jobs_type
    {
    public:
      typedef jobs_type super_type;
      /// Construct a child collector.
      ///
      /// \param parent Parent of the child to collect
      ///
      /// \param size Approximative previsional number of children.
      Collector(rJob parent, size_t = 0);
      /// Terminate all children.
      ~Collector();

      /// If there are terminated children, collect them.
      void collect();

      /// Print, for debugging.
      std::ostream& dump(std::ostream& o) const;

    private:
      rJob parent_;
    };

    /// Register a child.
    ///
    /// \param child The child job.
    ///
    /// \param collector Children will be automatically terminated
    ///                  when this object is destroyed.
    void register_child(const rJob& child, Collector& collector);

    /// Terminate child and remove it from our children list.
    void terminate_child(const rJob& child);

    /// Do we have a parent?
    ///
    /// \return True if we have a parent.
    bool child_job() const;

    /// Whether \a this is an ancester of \a that.
    bool ancester_of(const rJob& that) const;

  protected:

    /// Must be implemented to do something useful. If an exception is
    /// raised, it will be propagated to our parent if we have one or
    /// lost otherwise.
    virtual void work() = 0;

    /// Must be overriden. Called if a scheduling error is detected
    /// during the execution course of this job.
    ///
    /// \param msg The explanation of the scheduling error.
    virtual void scheduling_error(const std::string& msg = "") = 0;

  private:
    /// Current job state, to be manipulated only from the job and the
    /// scheduler.
    job_state state_;

    /// Current job deadline. The deadline is only meaningful when the
    /// job state is \c sleeping.
    libport::utime_t deadline_;

    /// The last time we have been frozen (in system time), or 0 if we
    /// are not currently frozen.
    libport::utime_t frozen_since_;

    /// The value we have to deduce from system time because we have
    /// been frozen.
    libport::utime_t time_shift_;

    /// Scheduler in charge of this job. Do not delete.
    Scheduler& scheduler_;

    /// This job name.
    libport::Symbol name_;

    /// Other jobs to wake up when we terminate.
    jobs_type to_wake_up_;

    /// Coro structure corresponding to this job.
    Coro* coro_;

  protected:

    /// Is the current job non-interruptible? If yes, yielding will
    /// do nothing and blocking operations may raise an exception.
    bool non_interruptible_;

  private:

    /// Helper functions for constructors.
    void init_common(const libport::Symbol& name);

    /// Is the current job side-effect free?
    bool side_effect_free_;

    /// The next exception to be propagated if any.
    exception_ptr pending_exception_;

    /// Our parent if any.
    rJob parent_;

    /// Our children.
    jobs_type children_;

    /// Check stack space from time to time.
    bool check_stack_space_;

    /// Number of jobs created and not yet destroyed.
    static unsigned int alive_jobs_;
  };

  SCHED_API
  std::ostream& operator<< (std::ostream& o, const Job& j);

  SCHED_API
  std::ostream& operator<< (std::ostream& o, const jobs_type& js);

  SCHED_API
  std::ostream& operator<< (std::ostream& o, const Job::Collector& c);

  /// This exception will be raised to tell the job that it is currently
  /// stopped and must try to unwind tags from its tag stack until it
  /// is either dead or not stopped anymore.
  struct SCHED_API StopException : public SchedulerException
  {
    StopException(unsigned int, boost::any);
    ~StopException() throw() {}
    ADD_FIELD(unsigned int, depth)
    ADD_FIELD(boost::any, payload)
    COMPLETE_EXCEPTION(StopException)
  };

  /// This exception encapsulates another one, sent by a child job.
  struct SCHED_API ChildException : public SchedulerException
  {
    ChildException(exception_ptr);
    ChildException(const ChildException&);
    ~ChildException() throw() {}
    virtual exception_ptr clone() const;
    ATTRIBUTE_NORETURN void rethrow_child_exception() const;
  protected:
    ATTRIBUTE_NORETURN virtual void rethrow_() const;
  private:
    mutable exception_ptr child_exception_;
  };

  /// Exception used to terminate a job.
  struct SCHED_API TerminateException : public SchedulerException
  {
    COMPLETE_EXCEPTION(TerminateException)
  };


  /// Terminate all jobs present in container and empty it.
  SCHED_API
  void terminate_jobs(jobs_type& jobs);

} // namespace sched

# include <sched/job.hxx>

#endif // !SCHED_JOB_HH
