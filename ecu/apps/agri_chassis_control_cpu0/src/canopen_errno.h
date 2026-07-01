/*
 * Project-local CANopenNode errno compatibility header.
 *
 * HPM SDK 1.11 includes the CANopenNode port header can.h before
 * canopen_errno.h.  can.h pulls in the toolchain errno definitions first, so
 * the SDK copy of canopen_errno.h can redefine EMSGSIZE when SEGGER Embedded
 * Studio uses its bundled RISC-V headers.  Keep this tracked wrapper in the
 * application include path so regenerated CMake/SEGGER projects stay
 * warning-free without editing the ignored SDK environment directory.
 */
#ifndef ECU_CANOPEN_ERRNO_COMPAT_H
#define ECU_CANOPEN_ERRNO_COMPAT_H

#include <errno.h>

/* CANopenNode/HPM code expects the BSD socket-style errno names below.  Define
 * only the names missing from the active toolchain so this header remains safe
 * with both SEGGER and GCC/newlib headers.
 */
#ifndef EIO
#define EIO 5
#endif

#ifndef ENOTSUP
#define ENOTSUP 134
#endif

#ifndef ENETDOWN
#define ENETDOWN 115
#endif

#ifndef ENETUNREACH
#define ENETUNREACH 114
#endif

#ifndef EMSGSIZE
#define EMSGSIZE 122
#endif

#ifndef ENOBUFS
#define ENOBUFS 105
#endif

#endif /* ECU_CANOPEN_ERRNO_COMPAT_H */
