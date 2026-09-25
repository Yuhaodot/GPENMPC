# Add the lifetime registry once when the runtime target exists.
if(TARGET gpenmpc_rfly_runtime_components)
  get_target_property(_gpenmpc_runtime_sources gpenmpc_rfly_runtime_components SOURCES)
  set(_gpenmpc_lifetime_source "${GPENMPC_RFLY}/px4_stream/LinkLifetimeRegistry.cpp")
  list(FIND _gpenmpc_runtime_sources "${_gpenmpc_lifetime_source}" _gpenmpc_lifetime_index)
  if(_gpenmpc_lifetime_index LESS 0)
    target_sources(gpenmpc_rfly_runtime_components PRIVATE "${_gpenmpc_lifetime_source}")
  endif()
endif()
