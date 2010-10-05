/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#ifndef KERNEL_DEBUG_HH
# define KERNEL_DEBUG_HH

# define DEBUG_OPEN(Flag)                                       \
  do                                                            \
  {								\
    std::cerr << "\x1b\x5b\x33\x33\x6d"                         \
              << Flag << ": "                                   \
              << "\x1b\x5b\x6d"                                 \
              << libport::indent;                               \
    SLEEP(1);							\
  } while (0)


# define DEBUG(Flag, Msg)					\
  do                                                            \
  {								\
    DEBUG_OPEN(Flag);                                           \
    std::cerr << Msg << std::endl;                              \
    SLEEP(1);							\
  } while (0)

# define DEBUGN(Flag, Msg)					\
  do                                                            \
  {								\
    std::cerr << Msg;                                           \
    SLEEP(1);							\
  } while (0)

#endif // !KERNEL_DEBUG_HH
