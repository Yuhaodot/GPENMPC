#include "RegisteredCanonicalModuleContext.hpp"
#include "../px4_stream/NativeOffboardBorrow.hpp"

// Compile-only size symbols. Not linked into the application and no object
// constructors run. The real target compiler determines these storage sizes.
extern "C" {
unsigned char gpenmpc_sizeof_registered_context[sizeof(gpenmpc_rfly_px4::RegisteredCanonicalModuleContext)]{};
unsigned char gpenmpc_sizeof_link_registry[sizeof(gpenmpc_rfly_stream::LinkLifetimeRegistry)]{};
unsigned char gpenmpc_sizeof_native_borrow[sizeof(gpenmpc_rfly_stream::NativeOffboardBorrow)]{};
}
