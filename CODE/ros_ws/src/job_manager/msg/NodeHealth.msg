string name
uint8 status
float32 cpu_percent         # 0 to N*100% where N=cores (matches htop)
float32 memory_mb           # RSS memory in MB
float32 last_heartbeat_age  # Seconds since last sign of life
uint32 restart_count
uint32 error_count
string last_error
string message

uint8 WAITING=0
uint8 RUNNING=1
uint8 STALE=2
uint8 DEAD=3
uint8 RESTARTING=4
uint8 SHUTDOWN=5
uint8 OFF=6
