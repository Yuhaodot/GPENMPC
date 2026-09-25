#ifndef GPENMPC_CANONICAL_FULL_INNER_ABI_H
#define GPENMPC_CANONICAL_FULL_INNER_ABI_H
/* C/POD numerical ABI. Serialize calls and the process-owned C initializer.
 * Inputs preserve binary64/uint64 bits. The owner supplies publication
 * records; GP18 carries the bound numerical reply. */
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define GPENMPC_FULL_INNER_ABI_VERSION 1u
typedef struct gpenmpc_full_inner_owner gpenmpc_full_inner_owner;
enum gpenmpc_full_inner_result { RFI_OK=0, RFI_ARGUMENT=1, RFI_RETIRED=2,
    RFI_REFERENCE_WINDOW_MISS=3, RFI_STORE_FAILURE=4, RFI_REFERENCE_BITS=5,
    RFI_REFERENCE_IDENTITY=6 };
typedef struct {
    uint32_t abi_version,leg_index;
    uint8_t reference_asset_sha256[32],task_sha256[32],configuration_sha256[32];
    double initial_phase_acceleration,initial_outer_i[3],jerk_limit_mps3;
} gpenmpc_full_inner_configuration;
/* Field mirror. load_window copies fields explicitly, excluding layout padding. */
typedef struct {
    uint32_t schema,capacity,leg_index,source_first_row,source_total_rows,row_count,binding_mode;
    uint8_t reference_asset_sha256[32]; uint64_t window_generation;
    double time_s[256],nominal_jet[3072],nominal_duration_s,total_duration_s,prefix_duration_s;
    double prefix_coefficients[192],ground_jet[12],rest_jet[12],relaunch_duration_s;
    double relaunch_offset_ned_m[3],vertical_frame_offset_ned_m;
} gpenmpc_full_inner_window;
typedef struct {
    uint8_t reference_asset_sha256[32];uint32_t leg_index;
    uint64_t window_generation,query_sequence,source_timestamp_ns,source_generation,reference_generation,outer_generation;
    double progress_s,progress_rate,target_phase_acceleration,target_outer_f[3],dt_s;
} gpenmpc_full_inner_reference_input;
/* Read-only pending query/transition. Echo identity fields to the numerical
 * step and copy actual_reference_pvaj into input36. */
typedef struct {
    uint8_t reference_asset_sha256[32];uint32_t leg_index;
    uint64_t source_timestamp_ns,source_generation,reference_generation,outer_generation;
    uint64_t window_generation,query_sequence,candidate_generation;
    double phase_before_s,phase_after_s,actual_reference_pvaj[12];
    uint8_t control_authority; /* always zero */
} gpenmpc_full_inner_reference_candidate;
typedef struct {
    uint8_t reference_asset_sha256[32];uint32_t leg_index;
    uint64_t window_generation,last_accepted_sequence;
    double query_progress_s,progress_rate,phase_acceleration,outer_i[3];
    uint64_t source_timestamp_ns,source_generation,reference_generation,outer_generation,output_generation,publication_us;
} gpenmpc_full_inner_reference_state;
typedef struct {
    double post_state64[64],kernel61[61],scaffold70[70],request19[19],consumed_reference_pvaj[12];
    uint64_t original_tags2[2],reference_candidate_generation,reference_query_sequence,reference_window_generation;
    uint64_t reference_generation,outer_generation;
    float control16[16];
    uint8_t prior_committed_state_sha256[32],full_inner_input_sha256[32];
    uint8_t control_authority; /* always zero */
} gpenmpc_full_inner_candidate;
typedef struct {
    uint64_t source_timestamp_ns,source_generation,output_generation,original_publication_us;
    float actual_control16[16];uint8_t output_published;
} gpenmpc_full_inner_backend;
typedef struct {
    uint64_t source_timestamp_ns,source_generation,output_generation,original_publication_us;
    float actual_control16[16];uint8_t publication_succeeded,numerical_commit_succeeded;
} gpenmpc_full_inner_numeric_receipt;
typedef struct {
    uint64_t source_timestamp_ns,source_generation,candidate_generation,query_sequence,window_generation;
    uint64_t reference_generation,outer_generation,output_generation,original_publication_us;
    double actual_reference_pvaj[12];uint8_t publication_succeeded,numerical_commit_succeeded;
} gpenmpc_full_inner_reference_receipt;
typedef struct {
    uint32_t abi_failure,numeric_failure,reference_failure,joint_failure;
    uint64_t kernel_calls,numeric_installs,reference_installs,prediction_fills;
    uint64_t joint_attempts,reported_publications,unique_publications,duplicate_reports,regressed_reports,joint_installs,partial_installs;
    gpenmpc_full_inner_backend last_report,last_unique_publication;
    uint8_t prediction_required,prediction_ready,control_authority;
} gpenmpc_full_inner_diagnostics;
typedef struct {
    uint8_t numeric_installed,reference_committed,prediction_required,prediction_ready;
    double state64[64],pending70[70];uint64_t original_installed_tags2[2];
    gpenmpc_full_inner_reference_state reference;
    uint8_t committed_state_sha256[32],loaded_window_sha256[32];
} gpenmpc_full_inner_state;
typedef struct {
    uint8_t generated_source_set_sha256[32],private_archive_sha256[32],facade_source_sha256[32];
} gpenmpc_full_inner_build_identity;
/* Closed records are available after joint publication and installation.
 * Filling the next OPEN prediction preserves closed fields and tags. */
typedef struct {
    double values5[5]; /* available, sample_closed, innovation_available, hard_invalid, trust */
    uint64_t original_installed_tags2[2],original_publication_us;
    uint8_t exported,installed;
    /* Appended actual current-step read-only diagnostics; never inferred
     * from the next GP query. Zero exported means unavailable, not zero GP. */
    double learning12[12];
    uint8_t learning_exported;
} gpenmpc_full_inner_closed_evidence;
size_t gpenmpc_full_inner_storage_bytes(void);
size_t gpenmpc_full_inner_storage_alignment(void);
size_t gpenmpc_full_inner_window_scratch_bytes(void);
/* storage must be zero-initialized, aligned, caller-owned and quiescent.
 * Existing live/retired owner memory cannot be reconstructed by this API. */
int gpenmpc_full_inner_construct(void *storage,size_t bytes,const gpenmpc_full_inner_configuration *configuration,gpenmpc_full_inner_owner **out);
/* Startup/refill-only caller scratch: no 28KB tick stack or duplicate resident
 * reference window. Scratch can be reused after this call returns. */
int gpenmpc_full_inner_load_window(gpenmpc_full_inner_owner *,const gpenmpc_full_inner_window *,void *scratch,size_t scratch_bytes);
// Explicit operator-reference entry uses target_outer_f as WORLD velocity
// only in the separately selected component mode. Not an eNMPC decision.
int gpenmpc_full_inner_prepare_operator_reference(gpenmpc_full_inner_owner *,const gpenmpc_full_inner_reference_input *,
    gpenmpc_full_inner_reference_candidate *);
int gpenmpc_full_inner_prepare_reference(gpenmpc_full_inner_owner *,const gpenmpc_full_inner_reference_input *,
    gpenmpc_full_inner_reference_candidate *);
int gpenmpc_full_inner_prepare_numeric(gpenmpc_full_inner_owner *,const double input36[36],const uint64_t tags2[2],
    const gpenmpc_full_inner_reference_candidate *,gpenmpc_full_inner_candidate *);
/* Compatibility path: exactly one reference prepare followed by the same
 * numeric prepare. New local owners should use the explicit two-step API. */
int gpenmpc_full_inner_prepare(gpenmpc_full_inner_owner *,const double input36[36],const uint64_t tags2[2],
    const gpenmpc_full_inner_reference_input *,gpenmpc_full_inner_candidate *);
int gpenmpc_full_inner_commit(gpenmpc_full_inner_owner *,const gpenmpc_full_inner_backend *,
    const gpenmpc_full_inner_numeric_receipt *,const gpenmpc_full_inner_reference_receipt *);
int gpenmpc_full_inner_fill_gp(gpenmpc_full_inner_owner *,const uint64_t original_tags2[2],const double result18[18]);
int gpenmpc_full_inner_copy_state(const gpenmpc_full_inner_owner *,gpenmpc_full_inner_state *);
int gpenmpc_full_inner_copy_closed_evidence(const gpenmpc_full_inner_owner *,gpenmpc_full_inner_closed_evidence *);
int gpenmpc_full_inner_diagnose(const gpenmpc_full_inner_owner *,gpenmpc_full_inner_diagnostics *);
int gpenmpc_full_inner_retire(gpenmpc_full_inner_owner *); /* no reset or output action */
void gpenmpc_full_inner_identity(gpenmpc_full_inner_build_identity *);
#ifdef __cplusplus
}
#endif
#endif
