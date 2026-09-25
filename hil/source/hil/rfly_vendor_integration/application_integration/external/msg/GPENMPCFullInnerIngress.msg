# Receive-only, local experimental MAVLink TUNNEL ingress. Not control authority.
uint64 timestamp # Original board HRT at entry to the parsed-frame handler; never dequeue/publication time
uint64 reception_sequence # Per receiver lifetime, validated arrivals including queue-full drops
uint8 receiver_instance
uint8 source_system # MAVLink header routing ID, NOT authentication
uint8 source_component # MAVLink header routing ID, NOT authentication
uint8 target_system
uint8 target_component
uint8 mavlink_sequence # Transport sequence; wraps and includes unrelated MAVLink messages
uint8 wire_payload_length # Actual MAVLink 2 payload length, possibly zero-trimmed
uint16 payload_type
uint8 payload_length # 1..128 meaningful opaque bytes
uint8[128] payload # Bytes after payload_length are zeroed
uint8 ingress_first_fault # Latched for receiver lifetime; zero means none
uint64 ingress_first_fault_hrt
uint8 ingress_last_fault
uint64 ingress_last_fault_hrt
uint64 ingress_rejected_total
uint64 ingress_queue_overflows
uint64 ingress_publication_failures
uint8 ORB_QUEUE_LENGTH = 16 # One existing prearm RWW burst; finite queue, consumers MUST reject reception_sequence/uORB generation gaps
# TOPICS gpenmpc_full_inner_ingress
