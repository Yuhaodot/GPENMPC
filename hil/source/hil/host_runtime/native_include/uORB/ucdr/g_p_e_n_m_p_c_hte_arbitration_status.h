/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/g_p_e_n_m_p_c_hte_arbitration_status.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_hte_arbitration_status()
{
	return 184;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_hte_arbitration_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_hte_arbitration_status_s& topic = *static_cast<const g_p_e_n_m_p_c_hte_arbitration_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.pending_group_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.pending_group_timestamp, sizeof(topic.pending_group_timestamp));
	buf.iterator += sizeof(topic.pending_group_timestamp);
	buf.offset += sizeof(topic.pending_group_timestamp);
	static_assert(sizeof(topic.last_resolved_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_resolved_timestamp, sizeof(topic.last_resolved_timestamp));
	buf.iterator += sizeof(topic.last_resolved_timestamp);
	buf.offset += sizeof(topic.last_resolved_timestamp);
	static_assert(sizeof(topic.last_valid_sample_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_valid_sample_timestamp, sizeof(topic.last_valid_sample_timestamp));
	buf.iterator += sizeof(topic.last_valid_sample_timestamp);
	buf.offset += sizeof(topic.last_valid_sample_timestamp);
	static_assert(sizeof(topic.last_verified_publication_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_verified_publication_timestamp, sizeof(topic.last_verified_publication_timestamp));
	buf.iterator += sizeof(topic.last_verified_publication_timestamp);
	buf.offset += sizeof(topic.last_verified_publication_timestamp);
	static_assert(sizeof(topic.last_verified_sample_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_verified_sample_timestamp, sizeof(topic.last_verified_sample_timestamp));
	buf.iterator += sizeof(topic.last_verified_sample_timestamp);
	buf.offset += sizeof(topic.last_verified_sample_timestamp);
	static_assert(sizeof(topic.last_bounded_dropout_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_bounded_dropout_timestamp, sizeof(topic.last_bounded_dropout_timestamp));
	buf.iterator += sizeof(topic.last_bounded_dropout_timestamp);
	buf.offset += sizeof(topic.last_bounded_dropout_timestamp);
	static_assert(sizeof(topic.last_bounded_dropout_hold_age_us) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_bounded_dropout_hold_age_us, sizeof(topic.last_bounded_dropout_hold_age_us));
	buf.iterator += sizeof(topic.last_bounded_dropout_hold_age_us);
	buf.offset += sizeof(topic.last_bounded_dropout_hold_age_us);
	static_assert(sizeof(topic.last_forwarded_local_position_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_forwarded_local_position_timestamp, sizeof(topic.last_forwarded_local_position_timestamp));
	buf.iterator += sizeof(topic.last_forwarded_local_position_timestamp);
	buf.offset += sizeof(topic.last_forwarded_local_position_timestamp);
	static_assert(sizeof(topic.last_forwarded_local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_forwarded_local_position_timestamp_sample, sizeof(topic.last_forwarded_local_position_timestamp_sample));
	buf.iterator += sizeof(topic.last_forwarded_local_position_timestamp_sample);
	buf.offset += sizeof(topic.last_forwarded_local_position_timestamp_sample);
	static_assert(sizeof(topic.hold_ts) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hold_ts, sizeof(topic.hold_ts));
	buf.iterator += sizeof(topic.hold_ts);
	buf.offset += sizeof(topic.hold_ts);
	static_assert(sizeof(topic.hold_sample_ts) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hold_sample_ts, sizeof(topic.hold_sample_ts));
	buf.iterator += sizeof(topic.hold_sample_ts);
	buf.offset += sizeof(topic.hold_sample_ts);
	static_assert(sizeof(topic.hold_age_us) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hold_age_us, sizeof(topic.hold_age_us));
	buf.iterator += sizeof(topic.hold_age_us);
	buf.offset += sizeof(topic.hold_age_us);
	static_assert(sizeof(topic.last_generation) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.last_generation, sizeof(topic.last_generation));
	buf.iterator += sizeof(topic.last_generation);
	buf.offset += sizeof(topic.last_generation);
	static_assert(sizeof(topic.same_timestamp_valid_winners) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.same_timestamp_valid_winners, sizeof(topic.same_timestamp_valid_winners));
	buf.iterator += sizeof(topic.same_timestamp_valid_winners);
	buf.offset += sizeof(topic.same_timestamp_valid_winners);
	static_assert(sizeof(topic.publications) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.publications, sizeof(topic.publications));
	buf.iterator += sizeof(topic.publications);
	buf.offset += sizeof(topic.publications);
	static_assert(sizeof(topic.generation_gaps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.generation_gaps, sizeof(topic.generation_gaps));
	buf.iterator += sizeof(topic.generation_gaps);
	buf.offset += sizeof(topic.generation_gaps);
	static_assert(sizeof(topic.groups) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.groups, sizeof(topic.groups));
	buf.iterator += sizeof(topic.groups);
	buf.offset += sizeof(topic.groups);
	static_assert(sizeof(topic.valid_winner_groups) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.valid_winner_groups, sizeof(topic.valid_winner_groups));
	buf.iterator += sizeof(topic.valid_winner_groups);
	buf.offset += sizeof(topic.valid_winner_groups);
	static_assert(sizeof(topic.valid_then_dt_small) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.valid_then_dt_small, sizeof(topic.valid_then_dt_small));
	buf.iterator += sizeof(topic.valid_then_dt_small);
	buf.offset += sizeof(topic.valid_then_dt_small);
	static_assert(sizeof(topic.dt_small_then_valid) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.dt_small_then_valid, sizeof(topic.dt_small_then_valid));
	buf.iterator += sizeof(topic.dt_small_then_valid);
	buf.offset += sizeof(topic.dt_small_then_valid);
	static_assert(sizeof(topic.multiple_valid_groups) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.multiple_valid_groups, sizeof(topic.multiple_valid_groups));
	buf.iterator += sizeof(topic.multiple_valid_groups);
	buf.offset += sizeof(topic.multiple_valid_groups);
	static_assert(sizeof(topic.equivalent_valid_duplicates) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.equivalent_valid_duplicates, sizeof(topic.equivalent_valid_duplicates));
	buf.iterator += sizeof(topic.equivalent_valid_duplicates);
	buf.offset += sizeof(topic.equivalent_valid_duplicates);
	static_assert(sizeof(topic.same_time_valid_advances) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.same_time_valid_advances, sizeof(topic.same_time_valid_advances));
	buf.iterator += sizeof(topic.same_time_valid_advances);
	buf.offset += sizeof(topic.same_time_valid_advances);
	static_assert(sizeof(topic.coalesced_soft_transients) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.coalesced_soft_transients, sizeof(topic.coalesced_soft_transients));
	buf.iterator += sizeof(topic.coalesced_soft_transients);
	buf.offset += sizeof(topic.coalesced_soft_transients);
	static_assert(sizeof(topic.invalid_only_groups) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.invalid_only_groups, sizeof(topic.invalid_only_groups));
	buf.iterator += sizeof(topic.invalid_only_groups);
	buf.offset += sizeof(topic.invalid_only_groups);
	static_assert(sizeof(topic.bounded_invalid_only_forwarded) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.bounded_invalid_only_forwarded, sizeof(topic.bounded_invalid_only_forwarded));
	buf.iterator += sizeof(topic.bounded_invalid_only_forwarded);
	buf.offset += sizeof(topic.bounded_invalid_only_forwarded);
	static_assert(sizeof(topic.rep_snap) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rep_snap, sizeof(topic.rep_snap));
	buf.iterator += sizeof(topic.rep_snap);
	buf.offset += sizeof(topic.rep_snap);
	static_assert(sizeof(topic.accepted_dropouts) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.accepted_dropouts, sizeof(topic.accepted_dropouts));
	buf.iterator += sizeof(topic.accepted_dropouts);
	buf.offset += sizeof(topic.accepted_dropouts);
	static_assert(sizeof(topic.reset_epoch_changes) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reset_epoch_changes, sizeof(topic.reset_epoch_changes));
	buf.iterator += sizeof(topic.reset_epoch_changes);
	buf.offset += sizeof(topic.reset_epoch_changes);
	static_assert(sizeof(topic.hard_fault_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hard_fault_count, sizeof(topic.hard_fault_count));
	buf.iterator += sizeof(topic.hard_fault_count);
	buf.offset += sizeof(topic.hard_fault_count);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fault, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.group_pending) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.group_pending, sizeof(topic.group_pending));
	buf.iterator += sizeof(topic.group_pending);
	buf.offset += sizeof(topic.group_pending);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hard_fault, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.hold_hte) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hold_hte, sizeof(topic.hold_hte));
	buf.iterator += sizeof(topic.hold_hte);
	buf.offset += sizeof(topic.hold_hte);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_hte_arbitration_status(ucdrBuffer& buf, g_p_e_n_m_p_c_hte_arbitration_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.pending_group_timestamp) == 8, "size mismatch");
	memcpy(&topic.pending_group_timestamp, buf.iterator, sizeof(topic.pending_group_timestamp));
	buf.iterator += sizeof(topic.pending_group_timestamp);
	buf.offset += sizeof(topic.pending_group_timestamp);
	static_assert(sizeof(topic.last_resolved_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_resolved_timestamp, buf.iterator, sizeof(topic.last_resolved_timestamp));
	buf.iterator += sizeof(topic.last_resolved_timestamp);
	buf.offset += sizeof(topic.last_resolved_timestamp);
	static_assert(sizeof(topic.last_valid_sample_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_valid_sample_timestamp, buf.iterator, sizeof(topic.last_valid_sample_timestamp));
	buf.iterator += sizeof(topic.last_valid_sample_timestamp);
	buf.offset += sizeof(topic.last_valid_sample_timestamp);
	static_assert(sizeof(topic.last_verified_publication_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_verified_publication_timestamp, buf.iterator, sizeof(topic.last_verified_publication_timestamp));
	buf.iterator += sizeof(topic.last_verified_publication_timestamp);
	buf.offset += sizeof(topic.last_verified_publication_timestamp);
	static_assert(sizeof(topic.last_verified_sample_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_verified_sample_timestamp, buf.iterator, sizeof(topic.last_verified_sample_timestamp));
	buf.iterator += sizeof(topic.last_verified_sample_timestamp);
	buf.offset += sizeof(topic.last_verified_sample_timestamp);
	static_assert(sizeof(topic.last_bounded_dropout_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_bounded_dropout_timestamp, buf.iterator, sizeof(topic.last_bounded_dropout_timestamp));
	buf.iterator += sizeof(topic.last_bounded_dropout_timestamp);
	buf.offset += sizeof(topic.last_bounded_dropout_timestamp);
	static_assert(sizeof(topic.last_bounded_dropout_hold_age_us) == 8, "size mismatch");
	memcpy(&topic.last_bounded_dropout_hold_age_us, buf.iterator, sizeof(topic.last_bounded_dropout_hold_age_us));
	buf.iterator += sizeof(topic.last_bounded_dropout_hold_age_us);
	buf.offset += sizeof(topic.last_bounded_dropout_hold_age_us);
	static_assert(sizeof(topic.last_forwarded_local_position_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_forwarded_local_position_timestamp, buf.iterator, sizeof(topic.last_forwarded_local_position_timestamp));
	buf.iterator += sizeof(topic.last_forwarded_local_position_timestamp);
	buf.offset += sizeof(topic.last_forwarded_local_position_timestamp);
	static_assert(sizeof(topic.last_forwarded_local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.last_forwarded_local_position_timestamp_sample, buf.iterator, sizeof(topic.last_forwarded_local_position_timestamp_sample));
	buf.iterator += sizeof(topic.last_forwarded_local_position_timestamp_sample);
	buf.offset += sizeof(topic.last_forwarded_local_position_timestamp_sample);
	static_assert(sizeof(topic.hold_ts) == 8, "size mismatch");
	memcpy(&topic.hold_ts, buf.iterator, sizeof(topic.hold_ts));
	buf.iterator += sizeof(topic.hold_ts);
	buf.offset += sizeof(topic.hold_ts);
	static_assert(sizeof(topic.hold_sample_ts) == 8, "size mismatch");
	memcpy(&topic.hold_sample_ts, buf.iterator, sizeof(topic.hold_sample_ts));
	buf.iterator += sizeof(topic.hold_sample_ts);
	buf.offset += sizeof(topic.hold_sample_ts);
	static_assert(sizeof(topic.hold_age_us) == 8, "size mismatch");
	memcpy(&topic.hold_age_us, buf.iterator, sizeof(topic.hold_age_us));
	buf.iterator += sizeof(topic.hold_age_us);
	buf.offset += sizeof(topic.hold_age_us);
	static_assert(sizeof(topic.last_generation) == 4, "size mismatch");
	memcpy(&topic.last_generation, buf.iterator, sizeof(topic.last_generation));
	buf.iterator += sizeof(topic.last_generation);
	buf.offset += sizeof(topic.last_generation);
	static_assert(sizeof(topic.same_timestamp_valid_winners) == 4, "size mismatch");
	memcpy(&topic.same_timestamp_valid_winners, buf.iterator, sizeof(topic.same_timestamp_valid_winners));
	buf.iterator += sizeof(topic.same_timestamp_valid_winners);
	buf.offset += sizeof(topic.same_timestamp_valid_winners);
	static_assert(sizeof(topic.publications) == 4, "size mismatch");
	memcpy(&topic.publications, buf.iterator, sizeof(topic.publications));
	buf.iterator += sizeof(topic.publications);
	buf.offset += sizeof(topic.publications);
	static_assert(sizeof(topic.generation_gaps) == 4, "size mismatch");
	memcpy(&topic.generation_gaps, buf.iterator, sizeof(topic.generation_gaps));
	buf.iterator += sizeof(topic.generation_gaps);
	buf.offset += sizeof(topic.generation_gaps);
	static_assert(sizeof(topic.groups) == 4, "size mismatch");
	memcpy(&topic.groups, buf.iterator, sizeof(topic.groups));
	buf.iterator += sizeof(topic.groups);
	buf.offset += sizeof(topic.groups);
	static_assert(sizeof(topic.valid_winner_groups) == 4, "size mismatch");
	memcpy(&topic.valid_winner_groups, buf.iterator, sizeof(topic.valid_winner_groups));
	buf.iterator += sizeof(topic.valid_winner_groups);
	buf.offset += sizeof(topic.valid_winner_groups);
	static_assert(sizeof(topic.valid_then_dt_small) == 4, "size mismatch");
	memcpy(&topic.valid_then_dt_small, buf.iterator, sizeof(topic.valid_then_dt_small));
	buf.iterator += sizeof(topic.valid_then_dt_small);
	buf.offset += sizeof(topic.valid_then_dt_small);
	static_assert(sizeof(topic.dt_small_then_valid) == 4, "size mismatch");
	memcpy(&topic.dt_small_then_valid, buf.iterator, sizeof(topic.dt_small_then_valid));
	buf.iterator += sizeof(topic.dt_small_then_valid);
	buf.offset += sizeof(topic.dt_small_then_valid);
	static_assert(sizeof(topic.multiple_valid_groups) == 4, "size mismatch");
	memcpy(&topic.multiple_valid_groups, buf.iterator, sizeof(topic.multiple_valid_groups));
	buf.iterator += sizeof(topic.multiple_valid_groups);
	buf.offset += sizeof(topic.multiple_valid_groups);
	static_assert(sizeof(topic.equivalent_valid_duplicates) == 4, "size mismatch");
	memcpy(&topic.equivalent_valid_duplicates, buf.iterator, sizeof(topic.equivalent_valid_duplicates));
	buf.iterator += sizeof(topic.equivalent_valid_duplicates);
	buf.offset += sizeof(topic.equivalent_valid_duplicates);
	static_assert(sizeof(topic.same_time_valid_advances) == 4, "size mismatch");
	memcpy(&topic.same_time_valid_advances, buf.iterator, sizeof(topic.same_time_valid_advances));
	buf.iterator += sizeof(topic.same_time_valid_advances);
	buf.offset += sizeof(topic.same_time_valid_advances);
	static_assert(sizeof(topic.coalesced_soft_transients) == 4, "size mismatch");
	memcpy(&topic.coalesced_soft_transients, buf.iterator, sizeof(topic.coalesced_soft_transients));
	buf.iterator += sizeof(topic.coalesced_soft_transients);
	buf.offset += sizeof(topic.coalesced_soft_transients);
	static_assert(sizeof(topic.invalid_only_groups) == 4, "size mismatch");
	memcpy(&topic.invalid_only_groups, buf.iterator, sizeof(topic.invalid_only_groups));
	buf.iterator += sizeof(topic.invalid_only_groups);
	buf.offset += sizeof(topic.invalid_only_groups);
	static_assert(sizeof(topic.bounded_invalid_only_forwarded) == 4, "size mismatch");
	memcpy(&topic.bounded_invalid_only_forwarded, buf.iterator, sizeof(topic.bounded_invalid_only_forwarded));
	buf.iterator += sizeof(topic.bounded_invalid_only_forwarded);
	buf.offset += sizeof(topic.bounded_invalid_only_forwarded);
	static_assert(sizeof(topic.rep_snap) == 4, "size mismatch");
	memcpy(&topic.rep_snap, buf.iterator, sizeof(topic.rep_snap));
	buf.iterator += sizeof(topic.rep_snap);
	buf.offset += sizeof(topic.rep_snap);
	static_assert(sizeof(topic.accepted_dropouts) == 4, "size mismatch");
	memcpy(&topic.accepted_dropouts, buf.iterator, sizeof(topic.accepted_dropouts));
	buf.iterator += sizeof(topic.accepted_dropouts);
	buf.offset += sizeof(topic.accepted_dropouts);
	static_assert(sizeof(topic.reset_epoch_changes) == 4, "size mismatch");
	memcpy(&topic.reset_epoch_changes, buf.iterator, sizeof(topic.reset_epoch_changes));
	buf.iterator += sizeof(topic.reset_epoch_changes);
	buf.offset += sizeof(topic.reset_epoch_changes);
	static_assert(sizeof(topic.hard_fault_count) == 4, "size mismatch");
	memcpy(&topic.hard_fault_count, buf.iterator, sizeof(topic.hard_fault_count));
	buf.iterator += sizeof(topic.hard_fault_count);
	buf.offset += sizeof(topic.hard_fault_count);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(&topic.fault, buf.iterator, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.group_pending) == 1, "size mismatch");
	memcpy(&topic.group_pending, buf.iterator, sizeof(topic.group_pending));
	buf.iterator += sizeof(topic.group_pending);
	buf.offset += sizeof(topic.group_pending);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(&topic.hard_fault, buf.iterator, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.hold_hte) == 4, "size mismatch");
	memcpy(&topic.hold_hte, buf.iterator, sizeof(topic.hold_hte));
	buf.iterator += sizeof(topic.hold_hte);
	buf.offset += sizeof(topic.hold_hte);
	return true;
}
