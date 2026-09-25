#pragma once
// Provide __EXPORT for host compilation before drv_hrt.h.
#ifndef __EXPORT
#define __EXPORT
#endif
#ifndef __BEGIN_DECLS
#define __BEGIN_DECLS extern "C" {
#endif
#ifndef __END_DECLS
#define __END_DECLS }
#endif
