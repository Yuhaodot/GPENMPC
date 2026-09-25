#pragma once
#if defined(__PX4_NUTTX) || defined(__arm__) || defined(__thumb__)
#error HOST declaration compatibility must not enter the target
#endif
// Provide C-linkage declarations for parameters/param.h on the Windows host.
#ifndef __BEGIN_DECLS
#define __BEGIN_DECLS extern "C" {
#define __END_DECLS }
#endif
