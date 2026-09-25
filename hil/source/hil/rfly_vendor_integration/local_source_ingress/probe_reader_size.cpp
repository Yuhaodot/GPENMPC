#include "Px4OriginalHilReceiptReader.hpp"
// Compile-only size markers; never linked into an application or allocated at
// runtime. Their ELF symbol lengths expose the target ABI sizes without run.
extern "C" {
unsigned char gpenmpc_reader_size[sizeof(gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader)];
unsigned char gpenmpc_reader_receipt_size[sizeof(gpenmpc_hil_endpoint_reader::Receipt)];
}
