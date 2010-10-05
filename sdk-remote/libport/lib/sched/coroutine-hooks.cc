/*
 * Copyright (C) 2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <vector>

#include <libport/foreach.hh>

#include <sched/coroutine-hooks.hh>

#define COROUTINE_HANDLER(EVENT)                                        \
static std::vector<coroutine_hook> coroutine_## EVENT ##_hooks;         \
static void handle_coroutine_## EVENT ##_hooks(Coro *);                 \
                                                                        \
void                                                                    \
add_coroutine_## EVENT ##_hook(coroutine_hook hook)                     \
{                                                                       \
  if (coroutine_## EVENT ##_hook != &handle_coroutine_## EVENT ##_hooks \
      && coroutine_## EVENT ##_hook != 0)                               \
    coroutine_## EVENT ##_hooks.push_back(coroutine_## EVENT ##_hook);  \
                                                                        \
  coroutine_## EVENT ##_hook = &handle_coroutine_## EVENT ##_hooks;     \
  coroutine_## EVENT ##_hooks.push_back(hook);                          \
}                                                                       \
                                                                        \
static                                                                  \
void                                                                    \
handle_coroutine_## EVENT ##_hooks(Coro *c)                             \
{                                                                       \
  foreach(coroutine_hook h, coroutine_## EVENT ##_hooks)                \
    h(c);                                                               \
}

COROUTINE_HANDLER(new)
COROUTINE_HANDLER(free)

