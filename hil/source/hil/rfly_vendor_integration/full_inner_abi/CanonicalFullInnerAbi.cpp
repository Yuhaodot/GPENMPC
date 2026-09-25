#include "../CanonicalCombinedSymbolNamespace.h" // only this isolated TU sees private rt* names
#include "CanonicalFullInnerAbi.h"
#include "../CanonicalJointStateInstaller.hpp"
#include "../../px4_full_inner/consumption/CanonicalSha256.hpp"
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize.h"
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_initialize.h"
#else
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#endif
#include "FullInnerBuildIdentity.h" // mechanically derived from actual sources/archive by build helper
#include <new>
#ifndef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
#error Explicit resident SD ABI is mandatory
#endif
namespace num = gpenmpc_local_math;
namespace ref = gpenmpc_reference_math;
namespace joint = gpenmpc_joint_math;
using Sha = gpenmpc_consumption::CanonicalSha256;
static constexpr uint64_t live_marker = UINT64_C(0x3149424149465252),
                          retired_marker = UINT64_C(0x3154455249465252);
static bool process_initialized =
    false; // process owner must serialize construction, as generated API
           // requires
#ifdef GPENMPC_OPERATOR_YAW_REFERENCE
// Same serial numerical owner/call stack as the private generated library.
// Scoped below to one prepare call, never retained across IO or GP waits.
static bool operator_yaw_active{},operator_yaw_fault{};
static double operator_yaw_radians{};
extern "C" double __real_gpenmpcDesiredSe3Command(const double*,const double*,
    const double*,const double*,double,const double*,const double*,double*,double*,double*);
extern "C" double __wrap_gpenmpcDesiredSe3Command(const double *state,const double *p,
    const double *v,const double *a,double mass,const double *wind,const double *augmentation,
    double *raw_force,double *projected_force,double *rotation) {
    const double result=__real_gpenmpcDesiredSe3Command(state,p,v,a,mass,wind,augmentation,
                                                     raw_force,projected_force,rotation);
    if(operator_yaw_active&&!gpenmpc_operator_reference::apply_yaw(operator_yaw_radians,rotation))
        operator_yaw_fault=true;
    return result;
}
extern "C" boolean_T __real_se3WrenchKernel(const double*,const double*,const double*,
    const double*,double,const double*,const double*,const double*,const double*,
    const double*,double*,double*,double*);
extern "C" boolean_T __wrap_se3WrenchKernel(const double*x,const double*p,const double*v,
    const double*a,double mass,const double*wind,const double*augmentation,
    const double*rotation,const double*omega,const double*omega_dot,
    double*wrench,double*rotor,double*diagnostic){
    const bool ok=__real_se3WrenchKernel(x,p,v,a,mass,wind,augmentation,rotation,
                                       omega,omega_dot,wrench,rotor,diagnostic);
    if(!ok||!operator_yaw_active)return ok;
    bool limited=false;
    if(!gpenmpc_operator_reference::allocate_manual(wrench,rotor,limited))return false;
    // Keep the original requested wrench/raw-rotor diagnostic. Record that
    // allocation was limited; never present the requested wrench as achieved.
    diagnostic[50]=limited?1.0:0.0;
    return true;
}
#endif
struct gpenmpc_full_inner_owner {
    uint64_t marker{live_marker};
    num::CanonicalLocalInnerStateStore::Workspace workspace{};
    num::CanonicalLocalInnerStateStore numeric;
    ref::CanonicalReferenceStateStore reference;
    joint::CanonicalJointStateInstaller group;
    gpenmpc_full_inner_configuration configuration;
    unsigned long long installed_tags[2]{};
    uint8_t window_sha[32]{}, prior_sha[32]{}, input_sha[32]{};
    uint32_t failure{};
    bool operator_reference{},committed_operator_reference{};
    double committed_yaw{},pending_yaw{},committed_yaw_rate{},pending_yaw_rate{};
    gpenmpc_full_inner_owner(const gpenmpc_full_inner_configuration &c,
                            const ref::Configuration &r) noexcept
        : numeric(workspace), reference(workspace, r),
          group(numeric, reference), configuration(c) {}
};
#ifdef GPENMPC_ABI_SIZE_PROBE
extern "C" const unsigned char
    gpenmpc_full_inner_sizeof_owner[sizeof(gpenmpc_full_inner_owner)] = {};
extern "C" const unsigned char gpenmpc_full_inner_sizeof_SD[sizeof(
    num::CanonicalLocalInnerStateStore::Workspace)] = {};
extern "C" const unsigned char gpenmpc_full_inner_sizeof_numeric[sizeof(
    num::CanonicalLocalInnerStateStore)] = {};
extern "C" const unsigned char gpenmpc_full_inner_sizeof_reference[sizeof(
    ref::CanonicalReferenceStateStore)] = {};
extern "C" const unsigned char gpenmpc_full_inner_sizeof_joint[sizeof(
    joint::CanonicalJointStateInstaller)] = {};
#endif
static_assert(sizeof(unsigned long long) == sizeof(uint64_t),
              "Exact generated uint64 tags ABI");
static int active(const gpenmpc_full_inner_owner *h) noexcept {
    if (!h)
        return RFI_ARGUMENT;
    if (h->marker == retired_marker)
        return RFI_RETIRED;
    if (h->marker != live_marker)
        return RFI_ARGUMENT;
    return h->failure ? static_cast<int>(h->failure) : RFI_OK;
}
static int fail(gpenmpc_full_inner_owner &h, int reason) noexcept {
    if (!h.failure)
        h.failure = static_cast<uint32_t>(reason);
    return reason;
}
static void bytes(Sha &s, const uint8_t *p, size_t n) noexcept {
    for (size_t i = 0; i < n; ++i)
        s.byte(p[i]);
}
static void reals(Sha &s, const double *p, size_t n) noexcept {
    for (size_t i = 0; i < n; ++i)
        s.real(p[i]);
}
static void domain(Sha &s, const char *p) noexcept {
    while (*p)
        s.byte(static_cast<uint8_t>(*p++));
    s.byte(0);
}
static void finish(Sha &s, uint8_t out[32]) noexcept {
    const auto words = s.finish();
    for (unsigned i = 0; i < 8; ++i)
        for (unsigned j = 0; j < 4; ++j)
            out[4 * i + j] = static_cast<uint8_t>(words[i] >> (24 - 8 * j));
}
static void reference_state(const ref::InstalledState &s,
                            gpenmpc_full_inner_reference_state &o) noexcept {
    o = {};
    std::memcpy(o.reference_asset_sha256, s.query.reference_asset_sha256, 32);
    o.leg_index = s.query.leg_index;
    o.window_generation = s.query.window_generation;
    o.last_accepted_sequence = s.query.last_accepted_sequence;
    o.query_progress_s = s.query_progress_s;
    o.progress_rate = s.progress_rate;
    o.phase_acceleration = s.phase_acceleration;
    std::memcpy(o.outer_i, s.outer_i, sizeof o.outer_i);
    o.source_timestamp_ns = s.source_timestamp_ns;
    o.source_generation = s.source_generation;
    o.reference_generation = s.reference_generation;
    o.outer_generation = s.outer_generation;
    o.output_generation = s.output_generation;
    o.publication_us = s.publication_us;
}
static void hash_reference(Sha &s, const ref::InstalledState &r) noexcept {
    bytes(s, r.query.reference_asset_sha256, 32);
    s.u32(r.query.leg_index);
    s.u64(r.query.window_generation);
    s.u64(r.query.last_accepted_sequence);
    s.real(r.query_progress_s);
    s.real(r.progress_rate);
    s.real(r.phase_acceleration);
    reals(s, r.outer_i, 3);
    s.u64(r.source_timestamp_ns);
    s.u64(r.source_generation);
    s.u64(r.reference_generation);
    s.u64(r.outer_generation);
    s.u64(r.output_generation);
    s.u64(r.publication_us);
}
static void hash_state(const gpenmpc_full_inner_owner &h,
                       uint8_t out[32]) noexcept {
    Sha s;
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
    domain(s, "GPENMPC_FULL_INNER_COMMITTED_STATE_WITH_LEARNING_AUDIT_V3");
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
    domain(s, "GPENMPC_FULL_INNER_COMMITTED_STATE_WITH_CLOSED_EVIDENCE_V2");
#else
    domain(s, "GPENMPC_FULL_INNER_COMMITTED_STATE_V1");
#endif
    const auto *state = h.numeric.installed_state64(),
               *pending = h.numeric.open_pending70();
    s.byte(state ? 1 : 0);
    if (state) {
        reals(s, state, 64);
        reals(s, pending, 70);
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        reals(s, h.numeric.installed_closed5(), 5);
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        reals(s, h.numeric.installed_learning12(), 12);
#endif
        s.u64(h.installed_tags[0]);
        s.u64(h.installed_tags[1]);
    }
    s.byte(h.numeric.prediction_required() ? 1 : 0);
    s.byte(h.numeric.prediction_ready() ? 1 : 0);
    s.byte(h.reference.committed() ? 1 : 0);
    hash_reference(s, h.reference.installed_state());
    // Preparing a reference must not mutate the previously committed state.
    // The pending operator profile becomes committed only with real output.
    if(h.committed_operator_reference){domain(s,"USB_OPERATOR_YAW_REFERENCE");s.real(h.committed_yaw);s.real(h.committed_yaw_rate);}
    finish(s, out);
}
static void hash_window(const gpenmpc_full_inner_window &w,
                        uint8_t out[32]) noexcept {
    Sha s;
    domain(s, "GPENMPC_FULL_INNER_REFERENCE_WINDOW_V1");
    s.u32(w.schema);
    s.u32(w.capacity);
    bytes(s, w.reference_asset_sha256, 32);
    s.u32(w.leg_index);
    s.u64(w.window_generation);
    s.u32(w.source_first_row);
    s.u32(w.source_total_rows);
    s.u32(w.row_count);
    reals(s, w.time_s, 256);
    reals(s, w.nominal_jet, 3072);
    s.real(w.nominal_duration_s);
    s.real(w.total_duration_s);
    s.u32(w.binding_mode);
    s.real(w.prefix_duration_s);
    reals(s, w.prefix_coefficients, 192);
    reals(s, w.ground_jet, 12);
    reals(s, w.rest_jet, 12);
    s.real(w.relaunch_duration_s);
    reals(s, w.relaunch_offset_ned_m, 3);
    s.real(w.vertical_frame_offset_ned_m);
    finish(s, out);
}
static void hash_input(gpenmpc_full_inner_owner &h, const double *input,
                       const uint64_t *tags,
                       const gpenmpc_full_inner_reference_input &r) noexcept {
    hash_state(h, h.prior_sha);
    Sha s;
    domain(s, "GPENMPC_FULL_INNER_INPUT36_STATE64_PENDING70_REFERENCE_V1");
    bytes(s, rfi_generated_source_sha, 32);
    bytes(s, rfi_facade_source_sha, 32);
    bytes(s, h.configuration.task_sha256, 32);
    bytes(s, h.configuration.configuration_sha256, 32);
    bytes(s, h.configuration.reference_asset_sha256, 32);
    s.u32(h.configuration.leg_index);
    s.real(h.configuration.initial_phase_acceleration);
    reals(s, h.configuration.initial_outer_i, 3);
    s.real(h.configuration.jerk_limit_mps3);
    bytes(s, h.window_sha, 32);
    bytes(s, h.prior_sha, 32);
    reals(s, input, 36);
    s.u64(tags[0]);
    s.u64(tags[1]);
    bytes(s, r.reference_asset_sha256, 32);
    s.u32(r.leg_index);
    s.u64(r.window_generation);
    s.u64(r.query_sequence);
    s.u64(r.source_timestamp_ns);
    s.u64(r.source_generation);
    s.u64(r.reference_generation);
    s.u64(r.outer_generation);
    s.real(r.progress_s);
    s.real(r.progress_rate);
    s.real(r.target_phase_acceleration);
    reals(s, r.target_outer_f, 3);
    s.real(r.dt_s);
    finish(s, h.input_sha);
}
static void backend(const joint::BackendPublication &r,
                    gpenmpc_full_inner_backend &o) noexcept {
    o = {};
    o.source_timestamp_ns = r.source_timestamp_ns;
    o.source_generation = r.source_generation;
    o.output_generation = r.output_generation;
    o.original_publication_us = r.original_publication_us;
    std::memcpy(o.actual_control16, r.actual_control16,
                sizeof o.actual_control16);
    o.output_published = r.output_published ? 1 : 0;
}
extern "C" {
size_t gpenmpc_full_inner_storage_bytes(void) {
    return sizeof(gpenmpc_full_inner_owner);
}
size_t gpenmpc_full_inner_storage_alignment(void) {
    return alignof(gpenmpc_full_inner_owner);
}
size_t gpenmpc_full_inner_window_scratch_bytes(void) {
    return sizeof(struct51_T);
}
void gpenmpc_full_inner_identity(gpenmpc_full_inner_build_identity *out) {
    if (!out)
        return;
    std::memcpy(out->generated_source_set_sha256, rfi_generated_source_sha, 32);
    std::memcpy(out->private_archive_sha256, rfi_private_archive_sha, 32);
    std::memcpy(out->facade_source_sha256, rfi_facade_source_sha, 32);
}
int gpenmpc_full_inner_construct(void *storage, size_t n,
                                const gpenmpc_full_inner_configuration *c,
                                gpenmpc_full_inner_owner **out) {
    if (!out)
        return RFI_ARGUMENT;
    *out = nullptr;
    if (!storage || !c || n < sizeof(gpenmpc_full_inner_owner) ||
        reinterpret_cast<uintptr_t>(storage) %
            alignof(gpenmpc_full_inner_owner) ||
        c->abi_version != GPENMPC_FULL_INNER_ABI_VERSION)
        return RFI_ARGUMENT;
    uint64_t marker = 0;
    std::memcpy(&marker, storage, 8);
    if (marker)
        return RFI_ARGUMENT;
    ref::Configuration rc{};
    std::memcpy(rc.reference_asset_sha256, c->reference_asset_sha256, 32);
    rc.leg_index = c->leg_index;
    rc.initial_phase_acceleration = c->initial_phase_acceleration;
    std::memcpy(rc.initial_outer_i, c->initial_outer_i,
                sizeof rc.initial_outer_i);
    rc.jerk_limit_mps3 = c->jerk_limit_mps3;
    if (!process_initialized) {
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize();
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
        gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_initialize();
#else
        gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
#endif
        process_initialized = true;
    }
    auto *h = new (storage) gpenmpc_full_inner_owner(*c, rc);
    *out = h;
    return h->reference.failure() == ref::Failure::None
               ? RFI_OK
               : fail(*h, RFI_STORE_FAILURE);
}
int gpenmpc_full_inner_load_window(gpenmpc_full_inner_owner *h,
                                  const gpenmpc_full_inner_window *w,
                                  void *scratch, size_t n) {
    const int a = active(h);
    if (a)
        return a;
    if (!w || !scratch || n < sizeof(struct51_T) ||
        reinterpret_cast<uintptr_t>(scratch) % alignof(struct51_T) ||
        w->capacity > UINT16_MAX || w->row_count > UINT16_MAX ||
        w->binding_mode > UINT8_MAX)
        return fail(*h, RFI_ARGUMENT);
    // Caller scratch and POD input must be disjoint. Convert fields bytewise.
    const auto p = reinterpret_cast<uintptr_t>(scratch),
               v = reinterpret_cast<uintptr_t>(w);
    const auto owner_start = reinterpret_cast<uintptr_t>(h);
    if (p > UINTPTR_MAX - sizeof(struct51_T) || v > UINTPTR_MAX - sizeof(*w) ||
        (p < v + sizeof(*w) && v < p + sizeof(struct51_T)) ||
        (p < owner_start + sizeof(*h) && owner_start < p + sizeof(struct51_T)))
        return fail(*h, RFI_ARGUMENT);
    auto &d = *new (scratch) struct51_T{};
    d.schema = w->schema;
    d.capacity = static_cast<unsigned short>(w->capacity);
    std::memcpy(d.reference_asset_sha256, w->reference_asset_sha256, 32);
    d.leg_index = w->leg_index;
    d.window_generation = w->window_generation;
    d.source_first_row = w->source_first_row;
    d.source_total_rows = w->source_total_rows;
    d.row_count = static_cast<unsigned short>(w->row_count);
    std::memcpy(d.time_s, w->time_s, sizeof d.time_s);
    std::memcpy(d.nominal_jet, w->nominal_jet, sizeof d.nominal_jet);
    d.nominal_duration_s = w->nominal_duration_s;
    d.total_duration_s = w->total_duration_s;
    d.binding_mode = static_cast<unsigned char>(w->binding_mode);
    d.prefix_duration_s = w->prefix_duration_s;
    std::memcpy(d.prefix_coefficients, w->prefix_coefficients,
                sizeof d.prefix_coefficients);
    std::memcpy(d.ground_jet, w->ground_jet, sizeof d.ground_jet);
    std::memcpy(d.rest_jet, w->rest_jet, sizeof d.rest_jet);
    d.relaunch_duration_s = w->relaunch_duration_s;
    std::memcpy(d.relaunch_offset_ned_m, w->relaunch_offset_ned_m,
                sizeof d.relaunch_offset_ned_m);
    d.vertical_frame_offset_ned_m = w->vertical_frame_offset_ned_m;
    const bool ok = h->reference.load_window(d);
    d.~struct51_T();
    if (!ok)
        return fail(*h, RFI_STORE_FAILURE);
    hash_window(*w, h->window_sha);
    return RFI_OK;
}
static int prepare_reference_impl(
    gpenmpc_full_inner_owner *h, const gpenmpc_full_inner_reference_input *r,
    gpenmpc_full_inner_reference_candidate *out, bool operator_velocity) {
    if (out)
        *out = {};
    const int a = active(h);
    if (a)
        return a;
    if (!r || !out)
        return fail(*h, RFI_ARGUMENT);
    ref::Input ri{};
    std::memcpy(ri.query.reference_asset_sha256, r->reference_asset_sha256, 32);
    ri.query.leg_index = r->leg_index;
    ri.query.window_generation = r->window_generation;
    ri.query.query_sequence = r->query_sequence;
    ri.query.progress_s = r->progress_s;
    ri.source_timestamp_ns = r->source_timestamp_ns;
    ri.source_generation = r->source_generation;
    ri.reference_generation = r->reference_generation;
    ri.outer_generation = r->outer_generation;
    ri.progress_rate = r->progress_rate;
    ri.target_phase_acceleration = r->target_phase_acceleration;
    std::memcpy(ri.target_outer_f, r->target_outer_f, sizeof ri.target_outer_f);
    ri.dt_s = r->dt_s;
    const auto prepared = h->reference.prepare(ri,operator_velocity);
    if (prepared == ref::PrepareResult::WindowMiss)
        return RFI_REFERENCE_WINDOW_MISS;
    if (prepared != ref::PrepareResult::Candidate)
        return fail(*h, RFI_STORE_FAILURE);
    h->operator_reference=operator_velocity;
    const auto &rc = *h->reference.candidate();
    const auto &actual = rc.transition.reference;
    std::memcpy(out->reference_asset_sha256,
                rc.input.query.reference_asset_sha256, 32);
    out->leg_index = rc.input.query.leg_index;
    out->source_timestamp_ns = rc.input.source_timestamp_ns;
    out->source_generation = rc.input.source_generation;
    out->reference_generation = rc.input.reference_generation;
    out->outer_generation = rc.input.outer_generation;
    out->window_generation = rc.input.query.window_generation;
    out->query_sequence = rc.input.query.query_sequence;
    out->candidate_generation = rc.candidate_generation;
    out->phase_before_s = h->reference.installed_state().query_progress_s;
    out->phase_after_s = rc.input.query.progress_s;
    std::memcpy(out->actual_reference_pvaj, actual.position_m, 24);
    std::memcpy(out->actual_reference_pvaj + 3, actual.velocity_mps, 24);
    std::memcpy(out->actual_reference_pvaj + 6, actual.acceleration_mps2, 24);
    std::memcpy(out->actual_reference_pvaj + 9, actual.jerk_mps3, 24);
    return RFI_OK;
}
int gpenmpc_full_inner_prepare_reference(gpenmpc_full_inner_owner *h,
    const gpenmpc_full_inner_reference_input *r,gpenmpc_full_inner_reference_candidate *out) {
    return prepare_reference_impl(h,r,out,false);
}
int gpenmpc_full_inner_prepare_operator_reference(gpenmpc_full_inner_owner *h,
    const gpenmpc_full_inner_reference_input *r,gpenmpc_full_inner_reference_candidate *out) {
    return prepare_reference_impl(h,r,out,true);
}
int gpenmpc_full_inner_prepare_numeric(
    gpenmpc_full_inner_owner *h, const double input[36], const uint64_t tags[2],
    const gpenmpc_full_inner_reference_candidate *r,
    gpenmpc_full_inner_candidate *out) {
    if (out)
        *out = {};
    const int a = active(h);
    if (a)
        return a;
    if (!input || !tags || !r || !out)
        return fail(*h, RFI_ARGUMENT);
    const auto *pending = h->reference.candidate();
    if (!pending)
        return fail(*h, RFI_REFERENCE_IDENTITY);
    const auto &rc = *pending;
    const auto &ri = rc.input;
    const auto &actual = rc.transition.reference;
    const auto &before = h->reference.installed_state().query_progress_s;
    if (std::memcmp(r->reference_asset_sha256, ri.query.reference_asset_sha256,
                    32) ||
        r->leg_index != ri.query.leg_index ||
        r->source_timestamp_ns != ri.source_timestamp_ns ||
        r->source_generation != ri.source_generation ||
        r->reference_generation != ri.reference_generation ||
        r->outer_generation != ri.outer_generation ||
        r->window_generation != ri.query.window_generation ||
        r->query_sequence != ri.query.query_sequence ||
        r->candidate_generation != rc.candidate_generation ||
        r->control_authority != 0 ||
        std::memcmp(&r->phase_before_s, &before, 8) ||
        std::memcmp(&r->phase_after_s, &ri.query.progress_s, 8) ||
        tags[0] != ri.source_timestamp_ns || tags[1] != ri.source_generation)
        return fail(*h, RFI_REFERENCE_IDENTITY);
    if (std::memcmp(r->actual_reference_pvaj, actual.position_m, 24) ||
        std::memcmp(r->actual_reference_pvaj + 3, actual.velocity_mps, 24) ||
        std::memcmp(r->actual_reference_pvaj + 6, actual.acceleration_mps2,
                    24) ||
        std::memcmp(r->actual_reference_pvaj + 9, actual.jerk_mps3, 24))
        return fail(*h, RFI_REFERENCE_BITS);
    if (std::memcmp(input + 19, actual.position_m, 24) ||
        std::memcmp(input + 22, actual.velocity_mps, 24) ||
        std::memcmp(input + 25, actual.acceleration_mps2, 24) ||
        std::memcmp(input + 28, actual.jerk_mps3, 24))
        return fail(*h, RFI_REFERENCE_BITS);
    // Hash the pending query input and current internal numerical/reference state.
    gpenmpc_full_inner_reference_input original{};
    std::memcpy(original.reference_asset_sha256,
                ri.query.reference_asset_sha256, 32);
    original.leg_index = ri.query.leg_index;
    original.window_generation = ri.query.window_generation;
    original.query_sequence = ri.query.query_sequence;
    original.source_timestamp_ns = ri.source_timestamp_ns;
    original.source_generation = ri.source_generation;
    original.reference_generation = ri.reference_generation;
    original.outer_generation = ri.outer_generation;
    original.progress_s = ri.query.progress_s;
    original.progress_rate = ri.progress_rate;
    original.target_phase_acceleration = ri.target_phase_acceleration;
    std::memcpy(original.target_outer_f, ri.target_outer_f,
                sizeof original.target_outer_f);
    original.dt_s = ri.dt_s;
    hash_input(*h, input, tags, original);
    const unsigned long long t[2] = {tags[0], tags[1]};
#ifdef GPENMPC_OPERATOR_YAW_REFERENCE
    if(h->operator_reference){
        // Initialize heading from the estimated attitude.
        const double yaw=h->reference.committed()?h->committed_yaw:
            std::atan2(2*(input[6]*input[9]+input[7]*input[8]),
                       1-2*(input[8]*input[8]+input[9]*input[9]));
        const double yaw_rate=h->reference.committed()?h->committed_yaw_rate:0.0;
        if(!gpenmpc_operator_reference::advance_yaw_reference(yaw,yaw_rate,
                ri.target_phase_acceleration,input[34],h->pending_yaw,h->pending_yaw_rate))
            return fail(*h,RFI_ARGUMENT);
    }
    operator_yaw_active=h->operator_reference;
    operator_yaw_radians=h->pending_yaw;operator_yaw_fault=false;
    const bool numeric_ok=h->numeric.prepare(input,t,h->operator_reference);
    operator_yaw_active=false;
    if(!numeric_ok||operator_yaw_fault)
#else
    if(h->operator_reference&&ri.target_phase_acceleration!=0.0)
        return fail(*h,RFI_ARGUMENT); // Yaw reference is disabled in this build.
    if (!h->numeric.prepare(input, t))
#endif
        return fail(*h, RFI_STORE_FAILURE);
    const auto &nc = *h->numeric.candidate();
    std::memcpy(out->post_state64, nc.post_state64, sizeof out->post_state64);
    std::memcpy(out->kernel61, nc.kernel61, sizeof out->kernel61);
    std::memcpy(out->scaffold70, nc.scaffold70, sizeof out->scaffold70);
    std::memcpy(out->request19, nc.request19, sizeof out->request19);
    std::memcpy(out->consumed_reference_pvaj, nc.consumed_reference_pvaj,
                sizeof out->consumed_reference_pvaj);
    out->original_tags2[0] = nc.original_tags2[0];
    out->original_tags2[1] = nc.original_tags2[1];
    std::memcpy(out->control16, nc.control16, sizeof out->control16);
    out->reference_candidate_generation = rc.candidate_generation;
    out->reference_query_sequence = rc.input.query.query_sequence;
    out->reference_window_generation = rc.input.query.window_generation;
    out->reference_generation = rc.input.reference_generation;
    out->outer_generation = rc.input.outer_generation;
    std::memcpy(out->prior_committed_state_sha256, h->prior_sha, 32);
    std::memcpy(out->full_inner_input_sha256, h->input_sha, 32);
    return RFI_OK;
}
int gpenmpc_full_inner_prepare(gpenmpc_full_inner_owner *h,
                              const double input[36], const uint64_t tags[2],
                              const gpenmpc_full_inner_reference_input *r,
                              gpenmpc_full_inner_candidate *out) {
    if (out)
        *out = {};
    const int a = active(h);
    if (a)
        return a;
    if (!input || !tags || !r || !out || r->source_timestamp_ns != tags[0] ||
        r->source_generation != tags[1])
        return fail(*h, RFI_ARGUMENT);
    gpenmpc_full_inner_reference_candidate pending{};
    const int prepared = gpenmpc_full_inner_prepare_reference(h, r, &pending);
    if (prepared != RFI_OK)
        return prepared;
    return gpenmpc_full_inner_prepare_numeric(h, input, tags, &pending, out);
}
int gpenmpc_full_inner_commit(gpenmpc_full_inner_owner *h,
                             const gpenmpc_full_inner_backend *e,
                             const gpenmpc_full_inner_numeric_receipt *n,
                             const gpenmpc_full_inner_reference_receipt *r) {
    // Even after a facade/store fault, forward a well-formed report to the
    // existing joint ledger so an actual already-published action is not lost.
    if (!h || h->marker != live_marker || !e || !n || !r)
        return RFI_ARGUMENT;
    if (e->output_published > 1 || n->publication_succeeded > 1 ||
        n->numerical_commit_succeeded > 1 || r->publication_succeeded > 1 ||
        r->numerical_commit_succeeded > 1)
        return fail(*h, RFI_ARGUMENT);
    joint::BackendPublication be{};
    num::PublicationReceipt nr{};
    ref::PublicationReceipt rr{};
    be.source_timestamp_ns = e->source_timestamp_ns;
    be.source_generation = e->source_generation;
    be.output_generation = e->output_generation;
    be.original_publication_us = e->original_publication_us;
    std::memcpy(be.actual_control16, e->actual_control16,
                sizeof be.actual_control16);
    be.output_published = e->output_published != 0;
    nr.source_timestamp_ns = n->source_timestamp_ns;
    nr.source_generation = n->source_generation;
    nr.output_generation = n->output_generation;
    nr.original_publication_us = n->original_publication_us;
    std::memcpy(nr.actual_control16, n->actual_control16,
                sizeof nr.actual_control16);
    nr.publication_succeeded = n->publication_succeeded != 0;
    nr.numerical_commit_succeeded = n->numerical_commit_succeeded != 0;
    rr.source_timestamp_ns = r->source_timestamp_ns;
    rr.source_generation = r->source_generation;
    rr.candidate_generation = r->candidate_generation;
    rr.query_sequence = r->query_sequence;
    rr.window_generation = r->window_generation;
    rr.reference_generation = r->reference_generation;
    rr.outer_generation = r->outer_generation;
    rr.output_generation = r->output_generation;
    rr.original_publication_us = r->original_publication_us;
    std::memcpy(rr.actual_reference_pvaj, r->actual_reference_pvaj,
                sizeof rr.actual_reference_pvaj);
    rr.publication_succeeded = r->publication_succeeded != 0;
    rr.numerical_commit_succeeded = r->numerical_commit_succeeded != 0;
    if (h->failure) {
        if (h->numeric.failure() == num::Failure::None)
            h->numeric.reject_joint_publication();
        if (h->reference.failure() == ref::Failure::None)
            h->reference.reject_joint_publication();
    }
    const bool ok = h->group.install_after_publication(be, nr, rr);
    if (ok) {
        h->committed_operator_reference=h->operator_reference;
        if(h->operator_reference){h->committed_yaw=h->pending_yaw;h->committed_yaw_rate=h->pending_yaw_rate;}
        h->installed_tags[0] = nr.source_timestamp_ns;
        h->installed_tags[1] = nr.source_generation;
        return RFI_OK;
    }
    return fail(*h, RFI_STORE_FAILURE);
}
int gpenmpc_full_inner_fill_gp(gpenmpc_full_inner_owner *h,
                              const uint64_t tags[2], const double gp[18]) {
    const int a = active(h);
    if (a)
        return a;
    if (!tags || !gp)
        return fail(*h, RFI_ARGUMENT);
    const unsigned long long t[2] = {tags[0], tags[1]};
    return h->numeric.fill_open_prediction(t, gp) ? RFI_OK
                                                  : fail(*h, RFI_STORE_FAILURE);
}
int gpenmpc_full_inner_copy_state(const gpenmpc_full_inner_owner *h,
                                 gpenmpc_full_inner_state *out) {
    if (!h || !out || (h->marker != live_marker && h->marker != retired_marker))
        return RFI_ARGUMENT;
    *out = {};
    const auto *state = h->numeric.installed_state64();
    out->numeric_installed = state ? 1 : 0;
    if (state) {
        std::memcpy(out->state64, state, sizeof out->state64);
        std::memcpy(out->pending70, h->numeric.open_pending70(),
                    sizeof out->pending70);
        out->original_installed_tags2[0] = h->installed_tags[0];
        out->original_installed_tags2[1] = h->installed_tags[1];
    }
    out->prediction_required = h->numeric.prediction_required() ? 1 : 0;
    out->prediction_ready = h->numeric.prediction_ready() ? 1 : 0;
    out->reference_committed = h->reference.committed() ? 1 : 0;
    reference_state(h->reference.installed_state(), out->reference);
    hash_state(*h, out->committed_state_sha256);
    std::memcpy(out->loaded_window_sha256, h->window_sha, 32);
    return RFI_OK;
}
int gpenmpc_full_inner_copy_closed_evidence(const gpenmpc_full_inner_owner *h,
                                          gpenmpc_full_inner_closed_evidence *out) {
    if (!h || !out || (h->marker != live_marker && h->marker != retired_marker))
        return RFI_ARGUMENT;
    *out = {};
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    out->exported = 1;
    if (const auto *values = h->numeric.installed_closed5()) {
        std::memcpy(out->values5, values, sizeof out->values5);
        out->original_installed_tags2[0] = h->installed_tags[0];
        out->original_installed_tags2[1] = h->installed_tags[1];
        out->original_publication_us = h->reference.installed_state().publication_us;
        out->installed = h->reference.committed() ? 1 : 0;
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        std::memcpy(out->learning12, h->numeric.installed_learning12(), sizeof out->learning12);
        out->learning_exported = 1;
#endif
    }
#endif
    return RFI_OK; // copies history even after failure; never freshness/authority
}
int gpenmpc_full_inner_diagnose(const gpenmpc_full_inner_owner *h,
                               gpenmpc_full_inner_diagnostics *out) {
    if (!h || !out || (h->marker != live_marker && h->marker != retired_marker))
        return RFI_ARGUMENT;
    *out = {};
    const auto &d = h->group.diagnostics();
    out->abi_failure = h->failure;
    out->numeric_failure = static_cast<uint32_t>(h->numeric.failure());
    out->reference_failure = static_cast<uint32_t>(h->reference.failure());
    out->joint_failure = static_cast<uint32_t>(d.failure);
    out->kernel_calls = h->numeric.kernel_calls();
    out->numeric_installs = h->numeric.installed_count();
    out->reference_installs = h->reference.installed_count();
    out->prediction_fills = h->numeric.prediction_fills();
    out->joint_attempts = d.attempts;
    out->reported_publications = d.published_event_reports;
    out->unique_publications = d.unique_publications_reported;
    out->duplicate_reports = d.duplicate_event_reports;
    out->regressed_reports = d.regressed_or_unidentified_reports;
    out->joint_installs = d.joint_installs;
    out->partial_installs = d.unexpected_partial_installs;
    backend(d.last_report, out->last_report);
    backend(d.last_unique_publication, out->last_unique_publication);
    out->prediction_required = h->numeric.prediction_required() ? 1 : 0;
    out->prediction_ready = h->numeric.prediction_ready() ? 1 : 0;
    return RFI_OK;
}
int gpenmpc_full_inner_retire(gpenmpc_full_inner_owner *h) {
    if (!h || h->marker != live_marker)
        return RFI_ARGUMENT;
    h->marker = retired_marker;
    return RFI_OK;
}
}
