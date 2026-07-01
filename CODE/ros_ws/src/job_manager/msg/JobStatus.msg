std_msgs/Header header

string job_id
string pickup_room
string dropoff_room
uint8 priority

uint8 state
string state_name          # Human-readable state
string message             # Current status detail

# State constants
uint8 QUEUED=0
uint8 WAITING_FOR_NAV=1    # Waiting for supervisor to bring up nav stack
uint8 PICKUP_NAV=2         # Navigating to pickup room
uint8 PICKUP_ARRIVED=3     # At pickup location (waiting for confirmation/timeout)
uint8 DROPOFF_NAV=4        # Navigating to dropoff room
uint8 DROPOFF_ARRIVED=5    # At dropoff location (waiting for confirmation/timeout)
uint8 RETURNING=6          # Returning to home
uint8 COMPLETE=7
uint8 FAILED=8
uint8 CANCELLED=9
