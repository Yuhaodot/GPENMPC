#pragma once
// Host-test broker using generated topic structures and production Io/Pump code.
#include <cstdint>
#include <cstddef>
#ifndef __EXPORT
#define __EXPORT
#endif
struct orb_metadata{std::uint8_t id;std::size_t size;unsigned queue;};
#define ORB_DECLARE(name) extern const orb_metadata __orb_##name
#define ORB_ID(name) (&__orb_##name)
// Use the compile-time range from platforms/common/uORB/uORB.h.
#if defined(CONSTRAINED_MEMORY)
#define ORB_MULTI_MAX_INSTANCES 4
#else
#define ORB_MULTI_MAX_INSTANCES 10
#endif
bool gpenmpc_test_topic_update(const orb_metadata*,void*,std::uint32_t&,std::uint8_t)noexcept;
bool gpenmpc_test_topic_publish(const orb_metadata*,const void*)noexcept;
