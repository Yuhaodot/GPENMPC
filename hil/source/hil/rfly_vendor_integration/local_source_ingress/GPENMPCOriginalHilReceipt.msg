# Read-only record of the ORIGINAL HIL_SENSOR receiver call. It changes no
# sensor or estimator value. Timestamp is the existing receiver HRT, not a
# new clock sample, wire-time conversion, measurement or control authority.
uint64 timestamp
uint64 wire_time_usec
uint64 receiver_address
uint64 link_address
uint64 original_event_sequence
uint64 prior_publication_failures
int32 receiver_instance
int32 channel
uint32 fields_updated
# Actual sensor publisher identity, read AFTER the existing update. Unknown
# instances remain -1 and IDs 0; matching sample timestamps alone is not proof
# that the selected estimator input came from this receiver's publishers.
int16 gyro_topic_instance
int16 accel_topic_instance
uint32 gyro_device_id
uint32 accel_device_id
uint8 system_id
uint8 component_id
uint8 mavlink_sequence
uint8 payload_length
uint8 sensor_id
uint8[65] original_payload
bool gyro_update_called
bool accel_update_called

# Engineering queue capacity: missed entries remain detectable by generation
# and per-receiver sequence; this is not a new source freshness allowance.
uint8 ORB_QUEUE_LENGTH = 16
