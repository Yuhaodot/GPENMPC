#include "Px4SelectedSourceReader.hpp"
extern "C" {
char gpenmpc_selected_reader_size[sizeof(gpenmpc_selected_source::Px4SelectedSourceReader)];
char gpenmpc_selected_receipt_size[sizeof(gpenmpc_selected_source::Receipt)];
char gpenmpc_selected_snapshot_size[sizeof(gpenmpc_odometry::Snapshot)];
}
