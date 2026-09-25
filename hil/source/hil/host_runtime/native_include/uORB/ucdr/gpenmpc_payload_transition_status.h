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
#include <uORB/topics/gpenmpc_payload_transition_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_payload_transition_status()
{
	return 228;
}

static inline bool ucdr_serialize_gpenmpc_payload_transition_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_payload_transition_status_s& topic = *static_cast<const gpenmpc_payload_transition_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.ht) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ht, sizeof(topic.ht));
	buf.iterator += sizeof(topic.ht);
	buf.offset += sizeof(topic.ht);
	static_assert(sizeof(topic.hs) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hs, sizeof(topic.hs));
	buf.iterator += sizeof(topic.hs);
	buf.offset += sizeof(topic.hs);
	static_assert(sizeof(topic.ha) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ha, sizeof(topic.ha));
	buf.iterator += sizeof(topic.ha);
	buf.offset += sizeof(topic.ha);
	static_assert(sizeof(topic.svt) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.svt, sizeof(topic.svt));
	buf.iterator += sizeof(topic.svt);
	buf.offset += sizeof(topic.svt);
	static_assert(sizeof(topic.hn) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hn, sizeof(topic.hn));
	buf.iterator += sizeof(topic.hn);
	buf.offset += sizeof(topic.hn);
	static_assert(sizeof(topic.hi) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hi, sizeof(topic.hi));
	buf.iterator += sizeof(topic.hi);
	buf.offset += sizeof(topic.hi);
	static_assert(sizeof(topic.tu) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.tu, sizeof(topic.tu));
	buf.iterator += sizeof(topic.tu);
	buf.offset += sizeof(topic.tu);
	static_assert(sizeof(topic.tc) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.tc, sizeof(topic.tc));
	buf.iterator += sizeof(topic.tc);
	buf.offset += sizeof(topic.tc);
	static_assert(sizeof(topic.tx) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.tx, sizeof(topic.tx));
	buf.iterator += sizeof(topic.tx);
	buf.offset += sizeof(topic.tx);
	static_assert(sizeof(topic.ad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ad, sizeof(topic.ad));
	buf.iterator += sizeof(topic.ad);
	buf.offset += sizeof(topic.ad);
	static_assert(sizeof(topic.ar) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ar, sizeof(topic.ar));
	buf.iterator += sizeof(topic.ar);
	buf.offset += sizeof(topic.ar);
	static_assert(sizeof(topic.ato) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ato, sizeof(topic.ato));
	buf.iterator += sizeof(topic.ato);
	buf.offset += sizeof(topic.ato);
	static_assert(sizeof(topic.owr) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.owr, sizeof(topic.owr));
	buf.iterator += sizeof(topic.owr);
	buf.offset += sizeof(topic.owr);
	static_assert(sizeof(topic.irr) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.irr, sizeof(topic.irr));
	buf.iterator += sizeof(topic.irr);
	buf.offset += sizeof(topic.irr);
	static_assert(sizeof(topic.ezr) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ezr, sizeof(topic.ezr));
	buf.iterator += sizeof(topic.ezr);
	buf.offset += sizeof(topic.ezr);
	static_assert(sizeof(topic.sfg) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sfg, sizeof(topic.sfg));
	buf.iterator += sizeof(topic.sfg);
	buf.offset += sizeof(topic.sfg);
	static_assert(sizeof(topic.rpt) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rpt, sizeof(topic.rpt));
	buf.iterator += sizeof(topic.rpt);
	buf.offset += sizeof(topic.rpt);
	static_assert(sizeof(topic.rvs) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rvs, sizeof(topic.rvs));
	buf.iterator += sizeof(topic.rvs);
	buf.offset += sizeof(topic.rvs);
	static_assert(sizeof(topic.hsf) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hsf, sizeof(topic.hsf));
	buf.iterator += sizeof(topic.hsf);
	buf.offset += sizeof(topic.hsf);
	static_assert(sizeof(topic.hcf) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hcf, sizeof(topic.hcf));
	buf.iterator += sizeof(topic.hcf);
	buf.offset += sizeof(topic.hcf);
	static_assert(sizeof(topic.sc) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sc, sizeof(topic.sc));
	buf.iterator += sizeof(topic.sc);
	buf.offset += sizeof(topic.sc);
	static_assert(sizeof(topic.sq) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sq, sizeof(topic.sq));
	buf.iterator += sizeof(topic.sq);
	buf.offset += sizeof(topic.sq);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fault, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.av) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.av, sizeof(topic.av));
	buf.iterator += sizeof(topic.av);
	buf.offset += sizeof(topic.av);
	static_assert(sizeof(topic.er) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.er, sizeof(topic.er));
	buf.iterator += sizeof(topic.er);
	buf.offset += sizeof(topic.er);
	static_assert(sizeof(topic.hlr) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hlr, sizeof(topic.hlr));
	buf.iterator += sizeof(topic.hlr);
	buf.offset += sizeof(topic.hlr);
	static_assert(sizeof(topic.hv) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hv, sizeof(topic.hv));
	buf.iterator += sizeof(topic.hv);
	buf.offset += sizeof(topic.hv);
	static_assert(sizeof(topic.ea) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ea, sizeof(topic.ea));
	buf.iterator += sizeof(topic.ea);
	buf.offset += sizeof(topic.ea);
	static_assert(sizeof(topic.earmed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.earmed, sizeof(topic.earmed));
	buf.iterator += sizeof(topic.earmed);
	buf.offset += sizeof(topic.earmed);
	static_assert(sizeof(topic.eia) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eia, sizeof(topic.eia));
	buf.iterator += sizeof(topic.eia);
	buf.offset += sizeof(topic.eia);
	static_assert(sizeof(topic.eld) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eld, sizeof(topic.eld));
	buf.iterator += sizeof(topic.eld);
	buf.offset += sizeof(topic.eld);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.edt) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.edt, sizeof(topic.edt));
	buf.iterator += sizeof(topic.edt);
	buf.offset += sizeof(topic.edt);
	static_assert(sizeof(topic.eso) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eso, sizeof(topic.eso));
	buf.iterator += sizeof(topic.eso);
	buf.offset += sizeof(topic.eso);
	static_assert(sizeof(topic.ta) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ta, sizeof(topic.ta));
	buf.iterator += sizeof(topic.ta);
	buf.offset += sizeof(topic.ta);
	static_assert(sizeof(topic.cd) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.cd, sizeof(topic.cd));
	buf.iterator += sizeof(topic.cd);
	buf.offset += sizeof(topic.cd);
	static_assert(sizeof(topic.rr) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.rr, sizeof(topic.rr));
	buf.iterator += sizeof(topic.rr);
	buf.offset += sizeof(topic.rr);
	static_assert(sizeof(topic.te) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.te, sizeof(topic.te));
	buf.iterator += sizeof(topic.te);
	buf.offset += sizeof(topic.te);
	static_assert(sizeof(topic.hfd) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hfd, sizeof(topic.hfd));
	buf.iterator += sizeof(topic.hfd);
	buf.offset += sizeof(topic.hfd);
	static_assert(sizeof(topic.afa) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.afa, sizeof(topic.afa));
	buf.iterator += sizeof(topic.afa);
	buf.offset += sizeof(topic.afa);
	static_assert(sizeof(topic.daa) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.daa, sizeof(topic.daa));
	buf.iterator += sizeof(topic.daa);
	buf.offset += sizeof(topic.daa);
	static_assert(sizeof(topic.dac) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.dac, sizeof(topic.dac));
	buf.iterator += sizeof(topic.dac);
	buf.offset += sizeof(topic.dac);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.aat) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.aat, sizeof(topic.aat));
	buf.iterator += sizeof(topic.aat);
	buf.offset += sizeof(topic.aat);
	static_assert(sizeof(topic.ads) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ads, sizeof(topic.ads));
	buf.iterator += sizeof(topic.ads);
	buf.offset += sizeof(topic.ads);
	static_assert(sizeof(topic.rs) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.rs, sizeof(topic.rs));
	buf.iterator += sizeof(topic.rs);
	buf.offset += sizeof(topic.rs);
	static_assert(sizeof(topic.lrv) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.lrv, sizeof(topic.lrv));
	buf.iterator += sizeof(topic.lrv);
	buf.offset += sizeof(topic.lrv);
	static_assert(sizeof(topic.hlf) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hlf, sizeof(topic.hlf));
	buf.iterator += sizeof(topic.hlf);
	buf.offset += sizeof(topic.hlf);
	static_assert(sizeof(topic.hcl) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.hcl, sizeof(topic.hcl));
	buf.iterator += sizeof(topic.hcl);
	buf.offset += sizeof(topic.hcl);
	static_assert(sizeof(topic.hte) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hte, sizeof(topic.hte));
	buf.iterator += sizeof(topic.hte);
	buf.offset += sizeof(topic.hte);
	static_assert(sizeof(topic.hvv) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hvv, sizeof(topic.hvv));
	buf.iterator += sizeof(topic.hvv);
	buf.offset += sizeof(topic.hvv);
	static_assert(sizeof(topic.hp) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hp, sizeof(topic.hp));
	buf.iterator += sizeof(topic.hp);
	buf.offset += sizeof(topic.hp);
	static_assert(sizeof(topic.htg) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.htg, sizeof(topic.htg));
	buf.iterator += sizeof(topic.htg);
	buf.offset += sizeof(topic.htg);
	static_assert(sizeof(topic.mr) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.mr, sizeof(topic.mr));
	buf.iterator += sizeof(topic.mr);
	buf.offset += sizeof(topic.mr);
	static_assert(sizeof(topic.dad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.dad, sizeof(topic.dad));
	buf.iterator += sizeof(topic.dad);
	buf.offset += sizeof(topic.dad);
	static_assert(sizeof(topic.aad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.aad, sizeof(topic.aad));
	buf.iterator += sizeof(topic.aad);
	buf.offset += sizeof(topic.aad);
	static_assert(sizeof(topic.acr) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acr, sizeof(topic.acr));
	buf.iterator += sizeof(topic.acr);
	buf.offset += sizeof(topic.acr);
	static_assert(sizeof(topic.hma) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hma, sizeof(topic.hma));
	buf.iterator += sizeof(topic.hma);
	buf.offset += sizeof(topic.hma);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_payload_transition_status(ucdrBuffer& buf, gpenmpc_payload_transition_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.ht) == 8, "size mismatch");
	memcpy(&topic.ht, buf.iterator, sizeof(topic.ht));
	buf.iterator += sizeof(topic.ht);
	buf.offset += sizeof(topic.ht);
	static_assert(sizeof(topic.hs) == 8, "size mismatch");
	memcpy(&topic.hs, buf.iterator, sizeof(topic.hs));
	buf.iterator += sizeof(topic.hs);
	buf.offset += sizeof(topic.hs);
	static_assert(sizeof(topic.ha) == 8, "size mismatch");
	memcpy(&topic.ha, buf.iterator, sizeof(topic.ha));
	buf.iterator += sizeof(topic.ha);
	buf.offset += sizeof(topic.ha);
	static_assert(sizeof(topic.svt) == 8, "size mismatch");
	memcpy(&topic.svt, buf.iterator, sizeof(topic.svt));
	buf.iterator += sizeof(topic.svt);
	buf.offset += sizeof(topic.svt);
	static_assert(sizeof(topic.hn) == 4, "size mismatch");
	memcpy(&topic.hn, buf.iterator, sizeof(topic.hn));
	buf.iterator += sizeof(topic.hn);
	buf.offset += sizeof(topic.hn);
	static_assert(sizeof(topic.hi) == 4, "size mismatch");
	memcpy(&topic.hi, buf.iterator, sizeof(topic.hi));
	buf.iterator += sizeof(topic.hi);
	buf.offset += sizeof(topic.hi);
	static_assert(sizeof(topic.tu) == 4, "size mismatch");
	memcpy(&topic.tu, buf.iterator, sizeof(topic.tu));
	buf.iterator += sizeof(topic.tu);
	buf.offset += sizeof(topic.tu);
	static_assert(sizeof(topic.tc) == 4, "size mismatch");
	memcpy(&topic.tc, buf.iterator, sizeof(topic.tc));
	buf.iterator += sizeof(topic.tc);
	buf.offset += sizeof(topic.tc);
	static_assert(sizeof(topic.tx) == 4, "size mismatch");
	memcpy(&topic.tx, buf.iterator, sizeof(topic.tx));
	buf.iterator += sizeof(topic.tx);
	buf.offset += sizeof(topic.tx);
	static_assert(sizeof(topic.ad) == 4, "size mismatch");
	memcpy(&topic.ad, buf.iterator, sizeof(topic.ad));
	buf.iterator += sizeof(topic.ad);
	buf.offset += sizeof(topic.ad);
	static_assert(sizeof(topic.ar) == 4, "size mismatch");
	memcpy(&topic.ar, buf.iterator, sizeof(topic.ar));
	buf.iterator += sizeof(topic.ar);
	buf.offset += sizeof(topic.ar);
	static_assert(sizeof(topic.ato) == 4, "size mismatch");
	memcpy(&topic.ato, buf.iterator, sizeof(topic.ato));
	buf.iterator += sizeof(topic.ato);
	buf.offset += sizeof(topic.ato);
	static_assert(sizeof(topic.owr) == 4, "size mismatch");
	memcpy(&topic.owr, buf.iterator, sizeof(topic.owr));
	buf.iterator += sizeof(topic.owr);
	buf.offset += sizeof(topic.owr);
	static_assert(sizeof(topic.irr) == 4, "size mismatch");
	memcpy(&topic.irr, buf.iterator, sizeof(topic.irr));
	buf.iterator += sizeof(topic.irr);
	buf.offset += sizeof(topic.irr);
	static_assert(sizeof(topic.ezr) == 4, "size mismatch");
	memcpy(&topic.ezr, buf.iterator, sizeof(topic.ezr));
	buf.iterator += sizeof(topic.ezr);
	buf.offset += sizeof(topic.ezr);
	static_assert(sizeof(topic.sfg) == 4, "size mismatch");
	memcpy(&topic.sfg, buf.iterator, sizeof(topic.sfg));
	buf.iterator += sizeof(topic.sfg);
	buf.offset += sizeof(topic.sfg);
	static_assert(sizeof(topic.rpt) == 4, "size mismatch");
	memcpy(&topic.rpt, buf.iterator, sizeof(topic.rpt));
	buf.iterator += sizeof(topic.rpt);
	buf.offset += sizeof(topic.rpt);
	static_assert(sizeof(topic.rvs) == 4, "size mismatch");
	memcpy(&topic.rvs, buf.iterator, sizeof(topic.rvs));
	buf.iterator += sizeof(topic.rvs);
	buf.offset += sizeof(topic.rvs);
	static_assert(sizeof(topic.hsf) == 4, "size mismatch");
	memcpy(&topic.hsf, buf.iterator, sizeof(topic.hsf));
	buf.iterator += sizeof(topic.hsf);
	buf.offset += sizeof(topic.hsf);
	static_assert(sizeof(topic.hcf) == 4, "size mismatch");
	memcpy(&topic.hcf, buf.iterator, sizeof(topic.hcf));
	buf.iterator += sizeof(topic.hcf);
	buf.offset += sizeof(topic.hcf);
	static_assert(sizeof(topic.sc) == 4, "size mismatch");
	memcpy(&topic.sc, buf.iterator, sizeof(topic.sc));
	buf.iterator += sizeof(topic.sc);
	buf.offset += sizeof(topic.sc);
	static_assert(sizeof(topic.sq) == 4, "size mismatch");
	memcpy(&topic.sq, buf.iterator, sizeof(topic.sq));
	buf.iterator += sizeof(topic.sq);
	buf.offset += sizeof(topic.sq);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(&topic.fault, buf.iterator, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.av) == 1, "size mismatch");
	memcpy(&topic.av, buf.iterator, sizeof(topic.av));
	buf.iterator += sizeof(topic.av);
	buf.offset += sizeof(topic.av);
	static_assert(sizeof(topic.er) == 1, "size mismatch");
	memcpy(&topic.er, buf.iterator, sizeof(topic.er));
	buf.iterator += sizeof(topic.er);
	buf.offset += sizeof(topic.er);
	static_assert(sizeof(topic.hlr) == 1, "size mismatch");
	memcpy(&topic.hlr, buf.iterator, sizeof(topic.hlr));
	buf.iterator += sizeof(topic.hlr);
	buf.offset += sizeof(topic.hlr);
	static_assert(sizeof(topic.hv) == 1, "size mismatch");
	memcpy(&topic.hv, buf.iterator, sizeof(topic.hv));
	buf.iterator += sizeof(topic.hv);
	buf.offset += sizeof(topic.hv);
	static_assert(sizeof(topic.ea) == 1, "size mismatch");
	memcpy(&topic.ea, buf.iterator, sizeof(topic.ea));
	buf.iterator += sizeof(topic.ea);
	buf.offset += sizeof(topic.ea);
	static_assert(sizeof(topic.earmed) == 1, "size mismatch");
	memcpy(&topic.earmed, buf.iterator, sizeof(topic.earmed));
	buf.iterator += sizeof(topic.earmed);
	buf.offset += sizeof(topic.earmed);
	static_assert(sizeof(topic.eia) == 1, "size mismatch");
	memcpy(&topic.eia, buf.iterator, sizeof(topic.eia));
	buf.iterator += sizeof(topic.eia);
	buf.offset += sizeof(topic.eia);
	static_assert(sizeof(topic.eld) == 1, "size mismatch");
	memcpy(&topic.eld, buf.iterator, sizeof(topic.eld));
	buf.iterator += sizeof(topic.eld);
	buf.offset += sizeof(topic.eld);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.edt) == 4, "size mismatch");
	memcpy(&topic.edt, buf.iterator, sizeof(topic.edt));
	buf.iterator += sizeof(topic.edt);
	buf.offset += sizeof(topic.edt);
	static_assert(sizeof(topic.eso) == 1, "size mismatch");
	memcpy(&topic.eso, buf.iterator, sizeof(topic.eso));
	buf.iterator += sizeof(topic.eso);
	buf.offset += sizeof(topic.eso);
	static_assert(sizeof(topic.ta) == 1, "size mismatch");
	memcpy(&topic.ta, buf.iterator, sizeof(topic.ta));
	buf.iterator += sizeof(topic.ta);
	buf.offset += sizeof(topic.ta);
	static_assert(sizeof(topic.cd) == 1, "size mismatch");
	memcpy(&topic.cd, buf.iterator, sizeof(topic.cd));
	buf.iterator += sizeof(topic.cd);
	buf.offset += sizeof(topic.cd);
	static_assert(sizeof(topic.rr) == 1, "size mismatch");
	memcpy(&topic.rr, buf.iterator, sizeof(topic.rr));
	buf.iterator += sizeof(topic.rr);
	buf.offset += sizeof(topic.rr);
	static_assert(sizeof(topic.te) == 1, "size mismatch");
	memcpy(&topic.te, buf.iterator, sizeof(topic.te));
	buf.iterator += sizeof(topic.te);
	buf.offset += sizeof(topic.te);
	static_assert(sizeof(topic.hfd) == 1, "size mismatch");
	memcpy(&topic.hfd, buf.iterator, sizeof(topic.hfd));
	buf.iterator += sizeof(topic.hfd);
	buf.offset += sizeof(topic.hfd);
	static_assert(sizeof(topic.afa) == 1, "size mismatch");
	memcpy(&topic.afa, buf.iterator, sizeof(topic.afa));
	buf.iterator += sizeof(topic.afa);
	buf.offset += sizeof(topic.afa);
	static_assert(sizeof(topic.daa) == 1, "size mismatch");
	memcpy(&topic.daa, buf.iterator, sizeof(topic.daa));
	buf.iterator += sizeof(topic.daa);
	buf.offset += sizeof(topic.daa);
	static_assert(sizeof(topic.dac) == 4, "size mismatch");
	memcpy(&topic.dac, buf.iterator, sizeof(topic.dac));
	buf.iterator += sizeof(topic.dac);
	buf.offset += sizeof(topic.dac);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.aat) == 8, "size mismatch");
	memcpy(&topic.aat, buf.iterator, sizeof(topic.aat));
	buf.iterator += sizeof(topic.aat);
	buf.offset += sizeof(topic.aat);
	static_assert(sizeof(topic.ads) == 8, "size mismatch");
	memcpy(&topic.ads, buf.iterator, sizeof(topic.ads));
	buf.iterator += sizeof(topic.ads);
	buf.offset += sizeof(topic.ads);
	static_assert(sizeof(topic.rs) == 8, "size mismatch");
	memcpy(&topic.rs, buf.iterator, sizeof(topic.rs));
	buf.iterator += sizeof(topic.rs);
	buf.offset += sizeof(topic.rs);
	static_assert(sizeof(topic.lrv) == 8, "size mismatch");
	memcpy(&topic.lrv, buf.iterator, sizeof(topic.lrv));
	buf.iterator += sizeof(topic.lrv);
	buf.offset += sizeof(topic.lrv);
	static_assert(sizeof(topic.hlf) == 8, "size mismatch");
	memcpy(&topic.hlf, buf.iterator, sizeof(topic.hlf));
	buf.iterator += sizeof(topic.hlf);
	buf.offset += sizeof(topic.hlf);
	static_assert(sizeof(topic.hcl) == 8, "size mismatch");
	memcpy(&topic.hcl, buf.iterator, sizeof(topic.hcl));
	buf.iterator += sizeof(topic.hcl);
	buf.offset += sizeof(topic.hcl);
	static_assert(sizeof(topic.hte) == 4, "size mismatch");
	memcpy(&topic.hte, buf.iterator, sizeof(topic.hte));
	buf.iterator += sizeof(topic.hte);
	buf.offset += sizeof(topic.hte);
	static_assert(sizeof(topic.hvv) == 4, "size mismatch");
	memcpy(&topic.hvv, buf.iterator, sizeof(topic.hvv));
	buf.iterator += sizeof(topic.hvv);
	buf.offset += sizeof(topic.hvv);
	static_assert(sizeof(topic.hp) == 4, "size mismatch");
	memcpy(&topic.hp, buf.iterator, sizeof(topic.hp));
	buf.iterator += sizeof(topic.hp);
	buf.offset += sizeof(topic.hp);
	static_assert(sizeof(topic.htg) == 4, "size mismatch");
	memcpy(&topic.htg, buf.iterator, sizeof(topic.htg));
	buf.iterator += sizeof(topic.htg);
	buf.offset += sizeof(topic.htg);
	static_assert(sizeof(topic.mr) == 4, "size mismatch");
	memcpy(&topic.mr, buf.iterator, sizeof(topic.mr));
	buf.iterator += sizeof(topic.mr);
	buf.offset += sizeof(topic.mr);
	static_assert(sizeof(topic.dad) == 4, "size mismatch");
	memcpy(&topic.dad, buf.iterator, sizeof(topic.dad));
	buf.iterator += sizeof(topic.dad);
	buf.offset += sizeof(topic.dad);
	static_assert(sizeof(topic.aad) == 4, "size mismatch");
	memcpy(&topic.aad, buf.iterator, sizeof(topic.aad));
	buf.iterator += sizeof(topic.aad);
	buf.offset += sizeof(topic.aad);
	static_assert(sizeof(topic.acr) == 4, "size mismatch");
	memcpy(&topic.acr, buf.iterator, sizeof(topic.acr));
	buf.iterator += sizeof(topic.acr);
	buf.offset += sizeof(topic.acr);
	static_assert(sizeof(topic.hma) == 4, "size mismatch");
	memcpy(&topic.hma, buf.iterator, sizeof(topic.hma));
	buf.iterator += sizeof(topic.hma);
	buf.offset += sizeof(topic.hma);
	return true;
}
