# Information about a single Aruco marker in the swarm system

uint32 marker_id

# Marker state: 0=FREE, 1=RESERVED, 2=LANDED
uint8 state
uint8 FREE=0
uint8 RESERVED=1
uint8 LANDED=2

# Marker type: 0=VICTIM, 1=FIRE
uint8 type
uint8 VICTIM=0
uint8 FIRE=1

# Owner drone identifier (empty if FREE)
string owner

# Time since last update in seconds
float64 time_since_update
