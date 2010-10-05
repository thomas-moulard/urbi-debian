#ifndef TASKIMPL_DEFINED
#define TASKIMPL_DEFINED 1

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdarg.h>

/* Copyright (c) 2005-2006 Russ Cox, MIT; see COPYRIGHT */

#if defined __sun__
#	define __EXTENSIONS__ 1 /* SunOS */
#	if defined __SunOS5_6__ || defined __SunOS5_7__ || defined __SunOS5_8__
		/* NOT USING #define __MAKECONTEXT_V2_SOURCE 1 / * SunOS */
#	else
#		define __MAKECONTEXT_V2_SOURCE 1
#	endif
#endif

//#define USE_UCONTEXT 1

#if defined __OpenBSD__
#	undef USE_UCONTEXT
#	define USE_UCONTEXT 0
#endif

#if defined __APPLE__
#	include <AvailabilityMacros.h>
#	if defined MAC_OS_X_VERSION_10_5
#		undef USE_UCONTEXT
#		define USE_UCONTEXT 0
#	endif
#endif

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#ifndef _WIN32
#	include <unistd.h>
#	include <sys/time.h>
#	include <sys/types.h>
#	ifndef __MINGW32__
#		include <sys/wait.h>
#	endif
#	include <sched.h>
#	include <signal.h>
#	if USE_UCONTEXT
#		include <ucontext.h>
#	endif
#	ifndef __MINGW32__
#		include <sys/utsname.h>
#	endif
#	include <inttypes.h>
#endif

//#include "task.h"

// #define nil ((void*)0)
#define nelem(x) (sizeof(x)/sizeof((x)[0]))

/*
#define ulong task_ulong
#define uint task_uint
#define uchar task_uchar
#define ushort task_ushort
#define uvlong task_uvlong
#define vlong task_vlong

typedef unsigned long ulong;
typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned long long uvlong;
typedef long long vlong;

#define print task_print
#define fprint task_fprint
#define snprint task_snprint
#define seprint task_seprint
#define vprint task_vprint
#define vfprint task_vfprint
#define vsnprint task_vsnprint
#define vseprint task_vseprint
#define strecpy task_strecpy

int print(char*, ...);
int fprint(int, char*, ...);
char *snprint(char*, uint, char*, ...);
char *seprint(char*, char*, char*, ...);
int vprint(char*, va_list);
int vfprint(int, char*, va_list);
char *vsnprint(char*, uint, char*, va_list);
char *vseprint(char*, char*, char*, va_list);
char *strecpy(char*, char*, char*);
*/

#if defined(__FreeBSD__) && __FreeBSD__ < 5
extern	int		getmcontext(mcontext_t*);
extern	void		setmcontext(const mcontext_t*);
#define	setcontext(u)	setmcontext(&(u)->uc_mcontext)
#define	getcontext(u)	getmcontext(&(u)->uc_mcontext)
extern	int		swapcontext(ucontext_t*, const ucontext_t*);
extern	void		makecontext(ucontext_t*, void(*)(), int, ...);
#endif

#if defined(__APPLE__)
#	define mcontext libthread_mcontext
#	define mcontext_t libthread_mcontext_t
#	define ucontext libthread_ucontext
#	define ucontext_t libthread_ucontext_t
#	if defined(__i386__)
#		include "386-ucontext.h"
#	elif defined(__x86_64__)
#		include "amd64-ucontext.h"
#	else
#		include "power-ucontext.h"
#	endif
#endif

#if defined(__OpenBSD__)
#	define mcontext libthread_mcontext
#	define mcontext_t libthread_mcontext_t
#	define ucontext libthread_ucontext
#	define ucontext_t libthread_ucontext_t
#	if defined __i386__
#		include "386-ucontext.h"
#	else
#		include "power-ucontext.h"
#	endif
extern pid_t rfork_thread(int, void*, int(*)(void*), void*);
#endif

#if 0 &&  defined(__sun__)
#	define mcontext libthread_mcontext
#	define mcontext_t libthread_mcontext_t
#	define ucontext libthread_ucontext
#	define ucontext_t libthread_ucontext_t
#	include "sparc-ucontext.h"
#endif

#if defined(__arm__)

/* In some dynamic library loading configurations, the libc implementation of
 * swapcontext and makecontenxt takes precedence over ours. So rename those
 * functions.
 */
#define swapcontext libcoro_swapcontext
#define makecontext libcoro_makecontext


int             swapcontext(ucontext_t*, const ucontext_t*);
void            makecontext(ucontext_t*, void(*)(), int, ...);
int             getmcontext(mcontext_t*);
void            setmcontext(const mcontext_t*);

#define        setcontext(u) \
   setmcontext((mcontext_t*)( (char*)(&(u)->uc_mcontext) + 12))
#define        getcontext(u)  \
   getmcontext((mcontext_t*)((char*)(&(u)->uc_mcontext) + 12))
//#define	setcontext(u)	setmcontext(&(u)->uc_mcontext)
//#define	getcontext(u)	getmcontext(&(u)->uc_mcontext)
#endif

/*
typedef struct Context Context;

enum
{
	STACK = 8192
};

struct Context
{
	ucontext_t	uc;
};

struct Task
{
	char	name[256];	// offset known to acid
	char	state[256];
	Task	*next;
	Task	*prev;
	Task	*allnext;
	Task	*allprev;
	Context	context;
	uvlong	alarmtime;
	uint	id;
	uchar	*stk;
	uint	stksize;
	int	exiting;
	int	alltaskslot;
	int	system;
	int	ready;
	void	(*startfn)(void*);
	void	*startarg;
	void	*udata;
};

void	taskready(Task*);
void	taskswitch(void);

void	addtask(Tasklist*, Task*);
void	deltask(Tasklist*, Task*);

extern Task	*taskrunning;
extern int	taskcount;
*/

#ifdef __cplusplus
} // extern "C"
#endif

#endif
