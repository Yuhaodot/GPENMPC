#pragma once
// Explicit HOST metadata seam, never used by the actual M7 compile.
#include <px4_platform_common/defines.h>
struct orb_metadata {};
extern const orb_metadata mock_original_hil_metadata;
#define ORB_DECLARE(name)
#define ORB_ID(name) (&mock_original_hil_metadata)
