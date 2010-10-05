/*! \file urbi/ucallbacks.hh
 *******************************************************************************

 Definition of the callback classes.

 This file is part of UObject Component Architecture\n
 Copyright (c) 2006, 2007, 2008, 2009 Gostai S.A.S.

 Permission to use, copy, modify, and redistribute this software for
 non-commercial use is hereby granted.

 This software is provided "as is" without warranty of any kind,
 either expressed or implied, including but not limited to the
 implied warranties of fitness for a particular purpose.

 For more information, comments, bug reports: http://www.urbiforge.com

 **************************************************************************** */

#ifndef URBI_UCALLBACKS_HH
# define URBI_UCALLBACKS_HH

# include <string>

# include <libport/compiler.hh>
# include <libport/meta.hh>
# include <libport/thread-pool.hh>
# include <urbi/export.hh>
# include <urbi/fwd.hh>
# include <urbi/utable.hh>
# include <urbi/uvalue.hh>
# include <urbi/ucontext.hh>
# include <urbi/uvar.hh>

# include <boost/function.hpp>

namespace urbi
{

  namespace impl
  {
    class URBI_SDK_API UGenericCallbackImpl
    {
    public:
      virtual ~UGenericCallbackImpl() = 0;
      virtual void initialize(UGenericCallback* owner, bool owned) = 0;
      virtual void initialize(UGenericCallback* owner) = 0;
      virtual void registerCallback() = 0;
      virtual void clear() = 0;
    };

    inline
    UGenericCallbackImpl::~UGenericCallbackImpl()
    {}
  };

  //! Function and Event storage mechanism
  /// This heavily overloaded class is the only way in C++ to make
  /// life easy from the the interface user point's of view.  */

  class URBI_SDK_API UGenericCallback: public UContext
  {
  public:
    UGenericCallback(UObject& owner,
                     UVar* target,
		     const std::string& type,
		     const std::string& name,
		     int size,
                     impl::UContextImpl* ctx);
    UGenericCallback(UObject& owner,
                     UVar* target,
		     const std::string& type,
		     const std::string& name,
                     impl::UContextImpl* ctx);
    virtual ~UGenericCallback();

    void registerCallback();

    const std::string& getName() const
    {
      return name;
    }

    /// Set this callback to asynchronous mode using \b mode locking mode.
    void setAsync(libport::ThreadPool::rTaskLock lock);

    typedef boost::function2<void, UValue&, const std::exception*> OnDone;
    /** Start evaluation, call onDone with result when done.
     * Synchronous if isSynchronous(), asynchronous else.
     */
    void eval(UList& param, OnDone onDone=0);
    /// Force synchronous evaluation.
    void syncEval(UList& param, OnDone onDone=0);
    virtual UValue __evalcall(UList& param)  = 0;

    /// Period of timers.
    ufloat period;

    /// Nb params of the callbacked function.
    int nbparam;

    /// Name of the UObject that created the callback.
    std::string objname;
    std::string type;
    /// Name of the hooked variable
    std::string name;
    /// Ptr to the hooked UVar, optional.
    UVar* target;
    impl::UGenericCallbackImpl* impl_;
    UObject& owner;
    /// Returns wether the call is synchronous or not.
    bool isSynchronous() const;
    /// Return the thread pool to use
    static libport::ThreadPool& threadPool();
  protected:
    /// TaskLock to use.
    libport::ThreadPool::rTaskLock taskLock;
    /// True if call must be made synchronously.
    bool synchronous_;
    template<typename T>
    static inline impl::UContextImpl* fetchContext(T* ptr, libport::meta::True)
    {
    return ptr->ctx_;
    }

    template<typename T>
    static inline impl::UContextImpl* fetchContext(T*, libport::meta::False)
    {
    return getCurrentContext();
    }
  };


// Support for arbitrary-signature notify callbacks.


template<typename T>
inline
typename uvar_ref_traits<typename uvalue_cast_return_type<T>::type>::type
uvar_uvalue_cast(UValue& v)
{
  if (v.type == DATA_VOID && v.storage)
  {
    UVar* var = reinterpret_cast<UVar*>(v.storage);
    return uvalue_cast<T>(const_cast<UValue&>(var->val()));
  }
  else
    return uvalue_cast<T>(v);
}

template<>
inline
UVar&
uvar_uvalue_cast<UVar&>(UValue& v)
{
  return uvalue_cast<UVar>(v);
}

// TODO: compile-time error if argument is UVar without a ref.

  /*------------------------------------------------.
  | This section is generated. Not for human eyes.  |
  `------------------------------------------------*/


  // non void return type

  template <class OBJ, class R>
  class UCallback0 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr0(Const, IsConst)			\
      UCallback0(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) () Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 0,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 0;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr0 (/**/, false);
    MKUCallBackNotifyCtr0 (const, true);

# undef MKUCallBackNotifyCtr0

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)()
	: ((*obj).*fun)     ();
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ();
    R (OBJ::*funconst) () const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ>
  class UCallbacknotifyend0 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr0(FConst, EConst, IsConst)		\
      UCallbacknotifyend0(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) () FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,0,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 0;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr0 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr0 (/**/, const, true);
    MKUCallBackNotifyEndCtr0 (const ,/**/, false);
    MKUCallBackNotifyEndCtr0 (const, const, true);

# undef MKUCallBackNotifyEndCtr0

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R>
  class UFCallback0 : public UGenericCallback
  {
    public:
    UFCallback0(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) (),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,0, ctx),
      fun(fun)
    {
      nbparam = 0;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)();
      return res;
    }

    private:
    R (*fun) ();
  };

  // callback creation for non-member non void return type

  template <class R>
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) (),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback0<R> (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack0(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R>		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) () Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback0<OBJ,R> (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack0 (/**/, /**/);
  MKUCreateCallBack0 (const, /**/);

# undef MKUCreateCallBack0

  // Special case for eventend notification
# define MKCreateUCallBackEnd0(FConst, EConst)			\
template <class OBJ, class EOBJ>         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) () FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend0<OBJ> (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd0 (/**/, /**/);
  MKCreateUCallBackEnd0 (/**/, const);
  MKCreateUCallBackEnd0 (const, /**/);
  MKCreateUCallBackEnd0 (const, const);

# undef MKCreateUCallBackEnd0


  // non void return type

  template <class OBJ, class R, class P1 >
  class UCallback1 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr1(Const, IsConst)			\
      UCallback1(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 1,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 1;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr1 (/**/, false);
    MKUCallBackNotifyCtr1 (const, true);

# undef MKUCallBackNotifyCtr1

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 );
    R (OBJ::*funconst) ( P1 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 >
  class UCallbacknotifyend1 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr1(FConst, EConst, IsConst)		\
      UCallbacknotifyend1(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,1,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 1;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr1 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr1 (/**/, const, true);
    MKUCallBackNotifyEndCtr1 (const ,/**/, false);
    MKUCallBackNotifyEndCtr1 (const, const, true);

# undef MKUCallBackNotifyEndCtr1

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 >
  class UFCallback1 : public UGenericCallback
  {
    public:
    UFCallback1(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,1, ctx),
      fun(fun)
    {
      nbparam = 1;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback1<R, P1 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack1(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback1<OBJ,R, P1 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack1 (/**/, /**/);
  MKUCreateCallBack1 (const, /**/);

# undef MKUCreateCallBack1

  // Special case for eventend notification
# define MKCreateUCallBackEnd1(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend1<OBJ, P1 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd1 (/**/, /**/);
  MKCreateUCallBackEnd1 (/**/, const);
  MKCreateUCallBackEnd1 (const, /**/);
  MKCreateUCallBackEnd1 (const, const);

# undef MKCreateUCallBackEnd1


  // non void return type

  template <class OBJ, class R, class P1 , class P2 >
  class UCallback2 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr2(Const, IsConst)			\
      UCallback2(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 2,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 2;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr2 (/**/, false);
    MKUCallBackNotifyCtr2 (const, true);

# undef MKUCallBackNotifyCtr2

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 );
    R (OBJ::*funconst) ( P1 , P2 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 >
  class UCallbacknotifyend2 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr2(FConst, EConst, IsConst)		\
      UCallbacknotifyend2(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,2,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 2;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr2 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr2 (/**/, const, true);
    MKUCallBackNotifyEndCtr2 (const ,/**/, false);
    MKUCallBackNotifyEndCtr2 (const, const, true);

# undef MKUCallBackNotifyEndCtr2

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 >
  class UFCallback2 : public UGenericCallback
  {
    public:
    UFCallback2(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,2, ctx),
      fun(fun)
    {
      nbparam = 2;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback2<R, P1 , P2 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack2(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback2<OBJ,R, P1 , P2 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack2 (/**/, /**/);
  MKUCreateCallBack2 (const, /**/);

# undef MKUCreateCallBack2

  // Special case for eventend notification
# define MKCreateUCallBackEnd2(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend2<OBJ, P1 , P2 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd2 (/**/, /**/);
  MKCreateUCallBackEnd2 (/**/, const);
  MKCreateUCallBackEnd2 (const, /**/);
  MKCreateUCallBackEnd2 (const, const);

# undef MKCreateUCallBackEnd2


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 >
  class UCallback3 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr3(Const, IsConst)			\
      UCallback3(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 3,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 3;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr3 (/**/, false);
    MKUCallBackNotifyCtr3 (const, true);

# undef MKUCallBackNotifyCtr3

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 );
    R (OBJ::*funconst) ( P1 , P2 , P3 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 >
  class UCallbacknotifyend3 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr3(FConst, EConst, IsConst)		\
      UCallbacknotifyend3(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,3,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 3;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr3 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr3 (/**/, const, true);
    MKUCallBackNotifyEndCtr3 (const ,/**/, false);
    MKUCallBackNotifyEndCtr3 (const, const, true);

# undef MKUCallBackNotifyEndCtr3

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 >
  class UFCallback3 : public UGenericCallback
  {
    public:
    UFCallback3(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,3, ctx),
      fun(fun)
    {
      nbparam = 3;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback3<R, P1 , P2 , P3 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack3(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback3<OBJ,R, P1 , P2 , P3 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack3 (/**/, /**/);
  MKUCreateCallBack3 (const, /**/);

# undef MKUCreateCallBack3

  // Special case for eventend notification
# define MKCreateUCallBackEnd3(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend3<OBJ, P1 , P2 , P3 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd3 (/**/, /**/);
  MKCreateUCallBackEnd3 (/**/, const);
  MKCreateUCallBackEnd3 (const, /**/);
  MKCreateUCallBackEnd3 (const, const);

# undef MKCreateUCallBackEnd3


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 >
  class UCallback4 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr4(Const, IsConst)			\
      UCallback4(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 4,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 4;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr4 (/**/, false);
    MKUCallBackNotifyCtr4 (const, true);

# undef MKUCallBackNotifyCtr4

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 >
  class UCallbacknotifyend4 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr4(FConst, EConst, IsConst)		\
      UCallbacknotifyend4(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,4,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 4;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr4 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr4 (/**/, const, true);
    MKUCallBackNotifyEndCtr4 (const ,/**/, false);
    MKUCallBackNotifyEndCtr4 (const, const, true);

# undef MKUCallBackNotifyEndCtr4

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 >
  class UFCallback4 : public UGenericCallback
  {
    public:
    UFCallback4(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,4, ctx),
      fun(fun)
    {
      nbparam = 4;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback4<R, P1 , P2 , P3 , P4 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack4(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback4<OBJ,R, P1 , P2 , P3 , P4 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack4 (/**/, /**/);
  MKUCreateCallBack4 (const, /**/);

# undef MKUCreateCallBack4

  // Special case for eventend notification
# define MKCreateUCallBackEnd4(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend4<OBJ, P1 , P2 , P3 , P4 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd4 (/**/, /**/);
  MKCreateUCallBackEnd4 (/**/, const);
  MKCreateUCallBackEnd4 (const, /**/);
  MKCreateUCallBackEnd4 (const, const);

# undef MKCreateUCallBackEnd4


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  class UCallback5 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr5(Const, IsConst)			\
      UCallback5(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 5,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 5;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr5 (/**/, false);
    MKUCallBackNotifyCtr5 (const, true);

# undef MKUCallBackNotifyCtr5

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 >
  class UCallbacknotifyend5 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr5(FConst, EConst, IsConst)		\
      UCallbacknotifyend5(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,5,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 5;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr5 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr5 (/**/, const, true);
    MKUCallBackNotifyEndCtr5 (const ,/**/, false);
    MKUCallBackNotifyEndCtr5 (const, const, true);

# undef MKUCallBackNotifyEndCtr5

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  class UFCallback5 : public UGenericCallback
  {
    public:
    UFCallback5(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,5, ctx),
      fun(fun)
    {
      nbparam = 5;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback5<R, P1 , P2 , P3 , P4 , P5 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack5(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback5<OBJ,R, P1 , P2 , P3 , P4 , P5 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack5 (/**/, /**/);
  MKUCreateCallBack5 (const, /**/);

# undef MKUCreateCallBack5

  // Special case for eventend notification
# define MKCreateUCallBackEnd5(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend5<OBJ, P1 , P2 , P3 , P4 , P5 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd5 (/**/, /**/);
  MKCreateUCallBackEnd5 (/**/, const);
  MKCreateUCallBackEnd5 (const, /**/);
  MKCreateUCallBackEnd5 (const, const);

# undef MKCreateUCallBackEnd5


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  class UCallback6 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr6(Const, IsConst)			\
      UCallback6(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 6,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 6;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr6 (/**/, false);
    MKUCallBackNotifyCtr6 (const, true);

# undef MKUCallBackNotifyCtr6

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  class UCallbacknotifyend6 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr6(FConst, EConst, IsConst)		\
      UCallbacknotifyend6(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,6,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 6;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr6 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr6 (/**/, const, true);
    MKUCallBackNotifyEndCtr6 (const ,/**/, false);
    MKUCallBackNotifyEndCtr6 (const, const, true);

# undef MKUCallBackNotifyEndCtr6

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  class UFCallback6 : public UGenericCallback
  {
    public:
    UFCallback6(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,6, ctx),
      fun(fun)
    {
      nbparam = 6;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback6<R, P1 , P2 , P3 , P4 , P5 , P6 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack6(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback6<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack6 (/**/, /**/);
  MKUCreateCallBack6 (const, /**/);

# undef MKUCreateCallBack6

  // Special case for eventend notification
# define MKCreateUCallBackEnd6(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend6<OBJ, P1 , P2 , P3 , P4 , P5 , P6 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd6 (/**/, /**/);
  MKCreateUCallBackEnd6 (/**/, const);
  MKCreateUCallBackEnd6 (const, /**/);
  MKCreateUCallBackEnd6 (const, const);

# undef MKCreateUCallBackEnd6


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  class UCallback7 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr7(Const, IsConst)			\
      UCallback7(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 7,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 7;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr7 (/**/, false);
    MKUCallBackNotifyCtr7 (const, true);

# undef MKUCallBackNotifyCtr7

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  class UCallbacknotifyend7 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr7(FConst, EConst, IsConst)		\
      UCallbacknotifyend7(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,7,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 7;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr7 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr7 (/**/, const, true);
    MKUCallBackNotifyEndCtr7 (const ,/**/, false);
    MKUCallBackNotifyEndCtr7 (const, const, true);

# undef MKUCallBackNotifyEndCtr7

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  class UFCallback7 : public UGenericCallback
  {
    public:
    UFCallback7(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,7, ctx),
      fun(fun)
    {
      nbparam = 7;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback7<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack7(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback7<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack7 (/**/, /**/);
  MKUCreateCallBack7 (const, /**/);

# undef MKUCreateCallBack7

  // Special case for eventend notification
# define MKCreateUCallBackEnd7(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend7<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd7 (/**/, /**/);
  MKCreateUCallBackEnd7 (/**/, const);
  MKCreateUCallBackEnd7 (const, /**/);
  MKCreateUCallBackEnd7 (const, const);

# undef MKCreateUCallBackEnd7


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  class UCallback8 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr8(Const, IsConst)			\
      UCallback8(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 8,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 8;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr8 (/**/, false);
    MKUCallBackNotifyCtr8 (const, true);

# undef MKUCallBackNotifyCtr8

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  class UCallbacknotifyend8 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr8(FConst, EConst, IsConst)		\
      UCallbacknotifyend8(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,8,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 8;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr8 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr8 (/**/, const, true);
    MKUCallBackNotifyEndCtr8 (const ,/**/, false);
    MKUCallBackNotifyEndCtr8 (const, const, true);

# undef MKUCallBackNotifyEndCtr8

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  class UFCallback8 : public UGenericCallback
  {
    public:
    UFCallback8(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,8, ctx),
      fun(fun)
    {
      nbparam = 8;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback8<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack8(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback8<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack8 (/**/, /**/);
  MKUCreateCallBack8 (const, /**/);

# undef MKUCreateCallBack8

  // Special case for eventend notification
# define MKCreateUCallBackEnd8(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend8<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd8 (/**/, /**/);
  MKCreateUCallBackEnd8 (/**/, const);
  MKCreateUCallBackEnd8 (const, /**/);
  MKCreateUCallBackEnd8 (const, const);

# undef MKCreateUCallBackEnd8


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  class UCallback9 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr9(Const, IsConst)			\
      UCallback9(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 9,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 9;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr9 (/**/, false);
    MKUCallBackNotifyCtr9 (const, true);

# undef MKUCallBackNotifyCtr9

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  class UCallbacknotifyend9 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr9(FConst, EConst, IsConst)		\
      UCallbacknotifyend9(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,9,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 9;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr9 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr9 (/**/, const, true);
    MKUCallBackNotifyEndCtr9 (const ,/**/, false);
    MKUCallBackNotifyEndCtr9 (const, const, true);

# undef MKUCallBackNotifyEndCtr9

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  class UFCallback9 : public UGenericCallback
  {
    public:
    UFCallback9(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,9, ctx),
      fun(fun)
    {
      nbparam = 9;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback9<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack9(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback9<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack9 (/**/, /**/);
  MKUCreateCallBack9 (const, /**/);

# undef MKUCreateCallBack9

  // Special case for eventend notification
# define MKCreateUCallBackEnd9(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend9<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd9 (/**/, /**/);
  MKCreateUCallBackEnd9 (/**/, const);
  MKCreateUCallBackEnd9 (const, /**/);
  MKCreateUCallBackEnd9 (const, const);

# undef MKCreateUCallBackEnd9


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  class UCallback10 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr10(Const, IsConst)			\
      UCallback10(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 10,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 10;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr10 (/**/, false);
    MKUCallBackNotifyCtr10 (const, true);

# undef MKUCallBackNotifyCtr10

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  class UCallbacknotifyend10 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr10(FConst, EConst, IsConst)		\
      UCallbacknotifyend10(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,10,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 10;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr10 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr10 (/**/, const, true);
    MKUCallBackNotifyEndCtr10 (const ,/**/, false);
    MKUCallBackNotifyEndCtr10 (const, const, true);

# undef MKUCallBackNotifyEndCtr10

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  class UFCallback10 : public UGenericCallback
  {
    public:
    UFCallback10(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,10, ctx),
      fun(fun)
    {
      nbparam = 10;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback10<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack10(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback10<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack10 (/**/, /**/);
  MKUCreateCallBack10 (const, /**/);

# undef MKUCreateCallBack10

  // Special case for eventend notification
# define MKCreateUCallBackEnd10(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend10<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd10 (/**/, /**/);
  MKCreateUCallBackEnd10 (/**/, const);
  MKCreateUCallBackEnd10 (const, /**/);
  MKCreateUCallBackEnd10 (const, const);

# undef MKCreateUCallBackEnd10


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >
  class UCallback11 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr11(Const, IsConst)			\
      UCallback11(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 11,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 11;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr11 (/**/, false);
    MKUCallBackNotifyCtr11 (const, true);

# undef MKUCallBackNotifyCtr11

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >
  class UCallbacknotifyend11 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr11(FConst, EConst, IsConst)		\
      UCallbacknotifyend11(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,11,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 11;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr11 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr11 (/**/, const, true);
    MKUCallBackNotifyEndCtr11 (const ,/**/, false);
    MKUCallBackNotifyEndCtr11 (const, const, true);

# undef MKUCallBackNotifyEndCtr11

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >
  class UFCallback11 : public UGenericCallback
  {
    public:
    UFCallback11(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,11, ctx),
      fun(fun)
    {
      nbparam = 11;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback11<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack11(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback11<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack11 (/**/, /**/);
  MKUCreateCallBack11 (const, /**/);

# undef MKUCreateCallBack11

  // Special case for eventend notification
# define MKCreateUCallBackEnd11(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend11<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd11 (/**/, /**/);
  MKCreateUCallBackEnd11 (/**/, const);
  MKCreateUCallBackEnd11 (const, /**/);
  MKCreateUCallBackEnd11 (const, const);

# undef MKCreateUCallBackEnd11


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >
  class UCallback12 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr12(Const, IsConst)			\
      UCallback12(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 12,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 12;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr12 (/**/, false);
    MKUCallBackNotifyCtr12 (const, true);

# undef MKUCallBackNotifyCtr12

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >
  class UCallbacknotifyend12 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr12(FConst, EConst, IsConst)		\
      UCallbacknotifyend12(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,12,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 12;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr12 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr12 (/**/, const, true);
    MKUCallBackNotifyEndCtr12 (const ,/**/, false);
    MKUCallBackNotifyEndCtr12 (const, const, true);

# undef MKUCallBackNotifyEndCtr12

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >
  class UFCallback12 : public UGenericCallback
  {
    public:
    UFCallback12(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,12, ctx),
      fun(fun)
    {
      nbparam = 12;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback12<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack12(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback12<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack12 (/**/, /**/);
  MKUCreateCallBack12 (const, /**/);

# undef MKUCreateCallBack12

  // Special case for eventend notification
# define MKCreateUCallBackEnd12(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend12<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd12 (/**/, /**/);
  MKCreateUCallBackEnd12 (/**/, const);
  MKCreateUCallBackEnd12 (const, /**/);
  MKCreateUCallBackEnd12 (const, const);

# undef MKCreateUCallBackEnd12


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >
  class UCallback13 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr13(Const, IsConst)			\
      UCallback13(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 13,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 13;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr13 (/**/, false);
    MKUCallBackNotifyCtr13 (const, true);

# undef MKUCallBackNotifyCtr13

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >
  class UCallbacknotifyend13 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr13(FConst, EConst, IsConst)		\
      UCallbacknotifyend13(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,13,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 13;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr13 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr13 (/**/, const, true);
    MKUCallBackNotifyEndCtr13 (const ,/**/, false);
    MKUCallBackNotifyEndCtr13 (const, const, true);

# undef MKUCallBackNotifyEndCtr13

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >
  class UFCallback13 : public UGenericCallback
  {
    public:
    UFCallback13(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,13, ctx),
      fun(fun)
    {
      nbparam = 13;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback13<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack13(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback13<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack13 (/**/, /**/);
  MKUCreateCallBack13 (const, /**/);

# undef MKUCreateCallBack13

  // Special case for eventend notification
# define MKCreateUCallBackEnd13(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend13<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd13 (/**/, /**/);
  MKCreateUCallBackEnd13 (/**/, const);
  MKCreateUCallBackEnd13 (const, /**/);
  MKCreateUCallBackEnd13 (const, const);

# undef MKCreateUCallBackEnd13


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >
  class UCallback14 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr14(Const, IsConst)			\
      UCallback14(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 14,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 14;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr14 (/**/, false);
    MKUCallBackNotifyCtr14 (const, true);

# undef MKUCallBackNotifyCtr14

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >
  class UCallbacknotifyend14 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr14(FConst, EConst, IsConst)		\
      UCallbacknotifyend14(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,14,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 14;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr14 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr14 (/**/, const, true);
    MKUCallBackNotifyEndCtr14 (const ,/**/, false);
    MKUCallBackNotifyEndCtr14 (const, const, true);

# undef MKUCallBackNotifyEndCtr14

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >
  class UFCallback14 : public UGenericCallback
  {
    public:
    UFCallback14(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,14, ctx),
      fun(fun)
    {
      nbparam = 14;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback14<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack14(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback14<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack14 (/**/, /**/);
  MKUCreateCallBack14 (const, /**/);

# undef MKUCreateCallBack14

  // Special case for eventend notification
# define MKCreateUCallBackEnd14(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend14<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd14 (/**/, /**/);
  MKCreateUCallBackEnd14 (/**/, const);
  MKCreateUCallBackEnd14 (const, /**/);
  MKCreateUCallBackEnd14 (const, const);

# undef MKCreateUCallBackEnd14


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >
  class UCallback15 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr15(Const, IsConst)			\
      UCallback15(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 15,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 15;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr15 (/**/, false);
    MKUCallBackNotifyCtr15 (const, true);

# undef MKUCallBackNotifyCtr15

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >
  class UCallbacknotifyend15 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr15(FConst, EConst, IsConst)		\
      UCallbacknotifyend15(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,15,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 15;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr15 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr15 (/**/, const, true);
    MKUCallBackNotifyEndCtr15 (const ,/**/, false);
    MKUCallBackNotifyEndCtr15 (const, const, true);

# undef MKUCallBackNotifyEndCtr15

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >
  class UFCallback15 : public UGenericCallback
  {
    public:
    UFCallback15(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,15, ctx),
      fun(fun)
    {
      nbparam = 15;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback15<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack15(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback15<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack15 (/**/, /**/);
  MKUCreateCallBack15 (const, /**/);

# undef MKUCreateCallBack15

  // Special case for eventend notification
# define MKCreateUCallBackEnd15(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend15<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd15 (/**/, /**/);
  MKCreateUCallBackEnd15 (/**/, const);
  MKCreateUCallBackEnd15 (const, /**/);
  MKCreateUCallBackEnd15 (const, const);

# undef MKCreateUCallBackEnd15


  // non void return type

  template <class OBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >
  class UCallback16 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyCtr16(Const, IsConst)			\
      UCallback16(UObject& owner,                                      \
                   UVar* target,                                        \
                   const std::string& type,	                        \
		   OBJ* obj,						\
		   R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ) Const,			\
		   const std::string& funname )                	        \
      : UGenericCallback(owner, target, type, funname, 16,             \
      fetchContext(obj,                                                 \
      typename libport::meta::If<libport::meta::Inherits<OBJ,           \
        UContext>::res>::res())),                                       \
	obj(obj), fun##Const(fun), is_const_ (IsConst)			\
    {									\
      nbparam = 16;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyCtr16 (/**/, false);
    MKUCallBackNotifyCtr16 (const, true);

# undef MKUCallBackNotifyCtr16

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(),
	(is_const_)
	? ((*obj).*funconst)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) , uvar_uvalue_cast<P16>(param[16 - 1]) )
	: ((*obj).*fun)     ( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) , uvar_uvalue_cast<P16>(param[16 - 1]) );
      return res;
    }

  private:
    OBJ* obj;
    R (OBJ::*fun)      ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 );
    R (OBJ::*funconst) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ) const;
    bool is_const_;
  };

  // void return type : special case for notifyend event callbacks
  template <class OBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >
  class UCallbacknotifyend16 : public UGenericCallback
  {
  public:

# define MKUCallBackNotifyEndCtr16(FConst, EConst, IsConst)		\
      UCallbacknotifyend16(UObject& owner, UVar* target,       	\
			    const std::string& type,			\
			    OBJ* obj,					\
			    void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ) FConst,	\
			    void (OBJ::*end)() EConst,			\
			    const std::string& funname)	                \
      : UGenericCallback(owner, target, type, funname,16,              \
      fetchContext(obj,                                                 \
        typename libport::meta::If<libport::meta::Inherits<OBJ,         \
          UContext>::res>::res())),                                     \
	obj(obj), fun(end), is_const_(IsConst)				\
    {									\
      nbparam = 16;                                                    \
      registerCallback();						\
    }

    MKUCallBackNotifyEndCtr16 (/**/, /**/, false);
    MKUCallBackNotifyEndCtr16 (/**/, const, true);
    MKUCallBackNotifyEndCtr16 (const ,/**/, false);
    MKUCallBackNotifyEndCtr16 (const, const, true);

# undef MKUCallBackNotifyEndCtr16

    virtual UValue __evalcall(UList&)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      UValue res;
      res(), (is_const_) ? ((*obj).*func)() : ((*obj).*fun)();
      return res;
    }

  private:
    OBJ* obj;
    void (OBJ::*fun) ();
    void (OBJ::*func) ();
    bool is_const_;
  };

  // non void non-member function
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >
  class UFCallback16 : public UGenericCallback
  {
    public:
    UFCallback16(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ),
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,16, ctx),
      fun(fun)
    {
      nbparam = 16;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), (*fun)( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) , uvar_uvalue_cast<P11>(param[11 - 1]) , uvar_uvalue_cast<P12>(param[12 - 1]) , uvar_uvalue_cast<P13>(param[13 - 1]) , uvar_uvalue_cast<P14>(param[14 - 1]) , uvar_uvalue_cast<P15>(param[15 - 1]) , uvar_uvalue_cast<P16>(param[16 - 1]) );
      return res;
    }

    private:
    R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 );
  };

  // callback creation for non-member non void return type

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  R (*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UFCallback16<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 > (owner, target, type,fun,funname, ctx);
  }

  // callback creation for non void return type

  // Second argument is useless, it exist only for visual c++ compiler
  // which warn at compilation for a call of MKUCreateCallBack()
  // Use a by-ref dynamic_cast to throw an exception in case of failure.
# define MKUCreateCallBack16(Const, Useless)                           \
  template <class OBJ, class EOBJ, class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  R (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ) Const,			\
		  const std::string& funname)	                        \
  {									\
    return new UCallback16<OBJ,R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 > (owner, target, type,       \
       &dynamic_cast<OBJ&>(*obj), fun, funname);			\
  }

  MKUCreateCallBack16 (/**/, /**/);
  MKUCreateCallBack16 (const, /**/);

# undef MKUCreateCallBack16

  // Special case for eventend notification
# define MKCreateUCallBackEnd16(FConst, EConst)			\
template <class OBJ, class EOBJ, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 , class P11 , class P12 , class P13 , class P14 , class P15 , class P16 >         		\
  UGenericCallback*							\
  createUCallback(UObject& owner, UVar* target,                         \
                  const std::string& type,	                        \
		  EOBJ* obj,						\
		  void (OBJ::*fun) ( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 ) FConst,		\
		  void (OBJ::*end)() EConst,				\
		  const std::string& funname)                           \
  {									\
    return new UCallbacknotifyend16<OBJ, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 , P11 , P12 , P13 , P14 , P15 , P16 > (owner, target,	\
	type, &dynamic_cast<OBJ&>(*obj), fun, end,funname);	        \
  }

  MKCreateUCallBackEnd16 (/**/, /**/);
  MKCreateUCallBackEnd16 (/**/, const);
  MKCreateUCallBackEnd16 (const, /**/);
  MKCreateUCallBackEnd16 (const, const);

# undef MKCreateUCallBackEnd16



  template <class R>
  class UBoostFunctionCallback0 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback0(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function0<R > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,0, ctx),
      fun(fun)
    {
      nbparam = 0;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun();
      return res;
    }

    private:
    boost::function0<R > fun;
  };

  template <class R>
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function0<R > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback0<R> (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R>
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)(),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback0<R > (owner, target,
    type,boost::function0<R >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 >
  class UBoostFunctionCallback1 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback1(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function1<R , P1 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,1, ctx),
      fun(fun)
    {
      nbparam = 1;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) );
      return res;
    }

    private:
    boost::function1<R , P1 > fun;
  };

  template <class R, class P1 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function1<R , P1 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback1<R, P1 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback1<R , P1 > (owner, target,
    type,boost::function1<R , P1 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 >
  class UBoostFunctionCallback2 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback2(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function2<R , P1 , P2 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,2, ctx),
      fun(fun)
    {
      nbparam = 2;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) );
      return res;
    }

    private:
    boost::function2<R , P1 , P2 > fun;
  };

  template <class R, class P1 , class P2 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function2<R , P1 , P2 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback2<R, P1 , P2 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback2<R , P1 , P2 > (owner, target,
    type,boost::function2<R , P1 , P2 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 >
  class UBoostFunctionCallback3 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback3(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function3<R , P1 , P2 , P3 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,3, ctx),
      fun(fun)
    {
      nbparam = 3;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) );
      return res;
    }

    private:
    boost::function3<R , P1 , P2 , P3 > fun;
  };

  template <class R, class P1 , class P2 , class P3 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function3<R , P1 , P2 , P3 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback3<R, P1 , P2 , P3 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback3<R , P1 , P2 , P3 > (owner, target,
    type,boost::function3<R , P1 , P2 , P3 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 >
  class UBoostFunctionCallback4 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback4(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function4<R , P1 , P2 , P3 , P4 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,4, ctx),
      fun(fun)
    {
      nbparam = 4;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) );
      return res;
    }

    private:
    boost::function4<R , P1 , P2 , P3 , P4 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function4<R , P1 , P2 , P3 , P4 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback4<R, P1 , P2 , P3 , P4 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback4<R , P1 , P2 , P3 , P4 > (owner, target,
    type,boost::function4<R , P1 , P2 , P3 , P4 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  class UBoostFunctionCallback5 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback5(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function5<R , P1 , P2 , P3 , P4 , P5 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,5, ctx),
      fun(fun)
    {
      nbparam = 5;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) );
      return res;
    }

    private:
    boost::function5<R , P1 , P2 , P3 , P4 , P5 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function5<R , P1 , P2 , P3 , P4 , P5 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback5<R, P1 , P2 , P3 , P4 , P5 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback5<R , P1 , P2 , P3 , P4 , P5 > (owner, target,
    type,boost::function5<R , P1 , P2 , P3 , P4 , P5 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  class UBoostFunctionCallback6 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback6(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function6<R , P1 , P2 , P3 , P4 , P5 , P6 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,6, ctx),
      fun(fun)
    {
      nbparam = 6;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) );
      return res;
    }

    private:
    boost::function6<R , P1 , P2 , P3 , P4 , P5 , P6 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function6<R , P1 , P2 , P3 , P4 , P5 , P6 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback6<R, P1 , P2 , P3 , P4 , P5 , P6 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 , P6 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback6<R , P1 , P2 , P3 , P4 , P5 , P6 > (owner, target,
    type,boost::function6<R , P1 , P2 , P3 , P4 , P5 , P6 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  class UBoostFunctionCallback7 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback7(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function7<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,7, ctx),
      fun(fun)
    {
      nbparam = 7;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) );
      return res;
    }

    private:
    boost::function7<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function7<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback7<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 , P6 , P7 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback7<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 > (owner, target,
    type,boost::function7<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  class UBoostFunctionCallback8 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback8(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function8<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,8, ctx),
      fun(fun)
    {
      nbparam = 8;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) );
      return res;
    }

    private:
    boost::function8<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function8<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback8<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback8<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 > (owner, target,
    type,boost::function8<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  class UBoostFunctionCallback9 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback9(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function9<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,9, ctx),
      fun(fun)
    {
      nbparam = 9;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) );
      return res;
    }

    private:
    boost::function9<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function9<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback9<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback9<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 > (owner, target,
    type,boost::function9<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 >(fun),funname, ctx);
  }
#endif

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  class UBoostFunctionCallback10 : public UGenericCallback
  {
    public:
    UBoostFunctionCallback10(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function10<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > fun,
                  const std::string& funname,
                  impl::UContextImpl* ctx)
      : UGenericCallback(owner, target, type, funname,10, ctx),
      fun(fun)
    {
      nbparam = 10;
      registerCallback();
    }

    virtual UValue __evalcall(UList& param)
    {
      impl::UContextImpl::CleanupStack s_(*ctx_);
      // Avoid warnings when we don't use param.
      (void) param;
      UValue res;
      res(), fun( uvar_uvalue_cast<P1>(param[1 - 1]) , uvar_uvalue_cast<P2>(param[2 - 1]) , uvar_uvalue_cast<P3>(param[3 - 1]) , uvar_uvalue_cast<P4>(param[4 - 1]) , uvar_uvalue_cast<P5>(param[5 - 1]) , uvar_uvalue_cast<P6>(param[6 - 1]) , uvar_uvalue_cast<P7>(param[7 - 1]) , uvar_uvalue_cast<P8>(param[8 - 1]) , uvar_uvalue_cast<P9>(param[9 - 1]) , uvar_uvalue_cast<P10>(param[10 - 1]) );
      return res;
    }

    private:
    boost::function10<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > fun;
  };

  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  UGenericCallback*
  createUCallback(UObject& owner, UVar* target,
                  const std::string& type,
		  boost::function10<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > fun,
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback10<R, P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > (owner, target, type,fun,funname, ctx);
  }

// No stdcall on 64 bit gcc it seems.
#ifndef __x86_64
  template <class R, class P1 , class P2 , class P3 , class P4 , class P5 , class P6 , class P7 , class P8 , class P9 , class P10 >
  UGenericCallback*
  createUCallbackStd(UObject& owner, UVar* target,
                  const std::string& type,
		  // MSVC wants it here.
		  R (ATTRIBUTE_STDCALL *fun)( P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 ),
		  const std::string& funname,
                  impl::UContextImpl* ctx)
  {
    return new UBoostFunctionCallback10<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 > (owner, target,
    type,boost::function10<R , P1 , P2 , P3 , P4 , P5 , P6 , P7 , P8 , P9 , P10 >(fun),funname, ctx);
  }
#endif
} // end namespace urbi

#endif // ! URBI_UCALLBACKS_HH

/// Local Variables:
/// mode: c++
/// End:
