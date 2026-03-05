std_msgs/Header header

# System-wide resources
float32 cpu_percent             # 0-100% averaged across all cores
uint8 cpu_count                 # Number of CPU cores
float32 memory_used_mb
float32 memory_total_mb

# Per-node health
system_supervisor/NodeHealth[] nodes

# System state
uint8 system_state
uint8 supervisor_state
string system_message
bool autonomous_enabled         # False after cat3 — only teleop works

uint8 NOMINAL=0
uint8 DEGRADED=1
uint8 LOCALIZATION_LOST=2
uint8 EMERGENCY_STOP=3

# supervisor states
uint8 SV_IDLE=0
uint8 SV_ACTIVATING=1
uint8 SV_ACTIVE=2
uint8 SV_DEACTIVATING=3
