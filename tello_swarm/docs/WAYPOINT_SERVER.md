# Waypoint Server

Centralized waypoint reservation server for drone swarm coordination. Prevents collisions by managing mutual exclusion for waypoints during autonomous navigation.

## Overview

The Waypoint Server provides a **traffic management system** for multi-drone missions. Drones must reserve waypoints before navigating to them, ensuring only one drone occupies a waypoint at a time.

### Key Features

- **Mutual Exclusion**: Only one drone can reserve a waypoint at a time
- **Heartbeat-Based Renewal**: Drones send periodic heartbeats to maintain reservations
- **Automatic Expiration**: Stale reservations expire after 8 seconds without heartbeat
- **Idempotent Operations**: Safe retry logic for network failures
- **Real-time Monitoring**: Publishes unavailable waypoints for dashboard visualization

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Waypoint Server                            │
│  - Manages N waypoints (configurable, default: 100)             │
│  - States: FREE | RESERVED                                      │
│  - Tracks owner (drone_id) and last_update timestamp            │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         │ Services           │ Subscriptions      │ Publishers
         ▼                    ▼                    ▼
   /reserve_waypoint    /waypoint_heartbeat   /unavailable_waypoints
   /unreserve_waypoint
```

### Waypoint State Machine

```
    ┌─────────┐
    │  FREE   │◄──────────────┐
    └────┬────┘               │
         │                    │
         │ reserve_waypoint   │ unreserve_waypoint
         │                    │ OR expiration (8s)
         ▼                    │
    ┌──────────┐              │
    │ RESERVED ├──────────────┘
    └──────────┘
         │
         │ heartbeat renewal
         │ (every 2s)
         ▼
```

## ROS2 Interface

### Services

#### `/reserve_waypoint` (swarm_interfaces/srv/ReserveWaypoint)

Reserve a waypoint for exclusive access.

**Request:**
```
int32 waypoint_index   # Waypoint sequence index (0-based)
string drone_id        # Drone identifier (e.g., "tello1")
```

**Response:**
```
bool success           # True if reserved, false if occupied
string message         # Human-readable status
```

**Examples:**
```bash
# Reserve waypoint 2
ros2 service call /reserve_waypoint swarm_interfaces/srv/ReserveWaypoint \
  "{waypoint_index: 2, drone_id: 'tello1'}"

# Response (success):
# success: true
# message: "Waypoint reserved successfully"

# Response (already reserved):
# success: false
# message: "Waypoint reserved by tello2"
```

#### `/unreserve_waypoint` (swarm_interfaces/srv/ReserveWaypoint)

Release a waypoint reservation.

**Request/Response:** Same as reserve_waypoint

**Examples:**
```bash
ros2 service call /unreserve_waypoint swarm_interfaces/srv/ReserveWaypoint \
  "{waypoint_index: 2, drone_id: 'tello1'}"
```

### Topics

#### `/waypoint_heartbeat` (swarm_interfaces/msg/WaypointHeartbeat)

**Publisher:** Drones (every 2 seconds)  
**Purpose:** Renew reservation to prevent expiration

**Message:**
```
std_msgs/Header header
string drone_id
int32 waypoint_index
```

**Example:**
```bash
ros2 topic pub /waypoint_heartbeat swarm_interfaces/msg/WaypointHeartbeat \
  "{drone_id: 'tello1', waypoint_index: 2}"
```

#### `/unavailable_waypoints` (std_msgs/msg/Int32MultiArray)

**Publisher:** Waypoint Server (every 500ms)  
**QoS:** Transient Local (late joiners receive last state)  
**Purpose:** Monitor which waypoints are currently reserved

**Message:**
```
int32[] data   # List of reserved waypoint indices
```

**Example:**
```bash
ros2 topic echo /unavailable_waypoints
# Output: data: [2, 5, 7]  # Waypoints 2, 5, 7 are reserved
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_waypoints` | int | 100 | Maximum waypoint index to support (0 to N-1) |
| `expiration_duration_s` | double | 8.0 | Seconds without heartbeat before expiration |

## Usage

### Starting the Server

**Basic:**
```bash
ros2 run tello_swarm waypoint_server
```

**With Launch File:**
```bash
ros2 launch tello_swarm waypoint_server_launch.py
```

**Custom Configuration:**
```bash
ros2 launch tello_swarm waypoint_server_launch.py \
  max_waypoints:=50 \
  expiration_duration_s:=10.0
```

### Monitoring

**Check server status:**
```bash
ros2 node info /waypoint_server
```

**Monitor unavailable waypoints:**
```bash
ros2 topic echo /unavailable_waypoints
```

**Watch service calls:**
```bash
ros2 topic echo --flow-style /rosout
```

### Manual Testing

**Reserve waypoint 3:**
```bash
ros2 service call /reserve_waypoint swarm_interfaces/srv/ReserveWaypoint \
  "{waypoint_index: 3, drone_id: 'test_drone'}"
```

**Send heartbeat (prevents expiration):**
```bash
ros2 topic pub --rate 0.5 /waypoint_heartbeat swarm_interfaces/msg/WaypointHeartbeat \
  "{drone_id: 'test_drone', waypoint_index: 3}"
```

**Unreserve waypoint 3:**
```bash
ros2 service call /unreserve_waypoint swarm_interfaces/srv/ReserveWaypoint \
  "{waypoint_index: 3, drone_id: 'test_drone'}"
```

## Integration with Mission Control

### Client-Side Implementation

Drones should follow this pattern:

```python
# 1. Request reservation before navigation
request = ReserveWaypoint.Request()
request.waypoint_index = current_waypoint_index
request.drone_id = self.drone_id

future = self.reserve_client.call_async(request)

# 2. Wait for reservation (non-blocking)
if future.done():
    response = future.result()
    if response.success:
        # Start navigation and heartbeat
        self.start_navigation()
        self.start_heartbeat_timer()
    else:
        # Waypoint occupied - hover and retry later
        self.hover()
        self.schedule_retry()

# 3. Publish heartbeat while navigating (every 2s)
heartbeat = WaypointHeartbeat()
heartbeat.drone_id = self.drone_id
heartbeat.waypoint_index = current_waypoint_index
self.heartbeat_pub.publish(heartbeat)

# 4. Unreserve after reaching waypoint
unreserve_request = ReserveWaypoint.Request()
unreserve_request.waypoint_index = current_waypoint_index
unreserve_request.drone_id = self.drone_id
self.unreserve_client.call_async(unreserve_request)
```

### Recommended Timing

- **Heartbeat Frequency:** 2 Hz (every 500ms)
- **Expiration Duration:** 8 seconds (allows 16 missed heartbeats)
- **Retry Interval:** 2 seconds (if waypoint occupied)
- **Timeout:** 30 seconds (skip waypoint if never available)

## Failure Modes & Recovery

### Drone Crashes

**Symptom:** Waypoint remains reserved indefinitely  
**Recovery:** Automatic expiration after 8 seconds without heartbeat  
**Monitoring:** Server logs warning when expiring stale reservation

### Server Crashes

**Symptom:** Drones cannot reserve waypoints  
**Recovery:** Drones should detect service unavailability and proceed in single-drone mode  
**Monitoring:** Implement `service_is_ready()` check with fallback logic

### Network Delays

**Symptom:** Heartbeat arrives late, reservation expires  
**Recovery:** Drone re-reserves waypoint (idempotent operation)  
**Mitigation:** Increase `expiration_duration_s` parameter

### Deadlock Prevention

**Scenario:** Multiple drones waiting for same waypoint indefinitely  
**Prevention:** Implement timeout (30s max wait) → skip waypoint if unavailable

## Best Practices

### 1. Always Unreserve
```cpp
// Even on timeout/error, unreserve before exiting
if (navigation_failed || timed_out) {
    unreserve_waypoint(current_index);
}
```

### 2. Handle Service Unavailability
```cpp
if (!reserve_client->service_is_ready()) {
    // Fallback: proceed without reservation (single-drone mode)
    RCLCPP_WARN(node->get_logger(), "Waypoint server unavailable");
}
```

### 3. Idempotent Retry Logic
```cpp
// Safe to retry reservation - server handles duplicate requests
while (!reserved && !timeout) {
    response = reserve_waypoint(index);
    if (!response.success) {
        sleep(2s);  // Wait before retry
    }
}
```

### 4. Monitor Unavailable Waypoints
```cpp
// Subscribe to /unavailable_waypoints for traffic awareness
void unavailable_callback(const Int32MultiArray& msg) {
    // Visualize occupied waypoints on dashboard
    // Adjust route planning based on congestion
}
```

## Debugging

### Common Issues

**Issue:** Reservation expires immediately  
**Cause:** Heartbeat not publishing or wrong waypoint_index  
**Fix:** Verify `/waypoint_heartbeat` topic with `ros2 topic echo`

**Issue:** Cannot reserve waypoint (always occupied)  
**Cause:** Previous drone didn't unreserve  
**Fix:** Wait 8 seconds for expiration or restart server

**Issue:** Service call times out  
**Cause:** Server not running  
**Fix:** Check `ros2 node list`, restart with launch file

### Diagnostic Commands

```bash
# Check if server is running
ros2 node list | grep waypoint_server

# Monitor service calls
ros2 service list | grep waypoint

# Check heartbeat frequency
ros2 topic hz /waypoint_heartbeat

# See current reservations
ros2 topic echo /unavailable_waypoints

# View server logs
ros2 topic echo /rosout | grep waypoint_server
```

## Performance Considerations

- **Scalability:** Tested with 10 drones, 100 waypoints
- **Latency:** Service calls < 5ms on localhost
- **Memory:** ~1KB per waypoint (100 waypoints = 100KB)
- **CPU:** < 1% on Raspberry Pi 4

## Future Enhancements

- [ ] Priority-based reservation (VIP drones)
- [ ] Queue management for occupied waypoints
- [ ] Dynamic waypoint addition/removal
- [ ] Reservation statistics dashboard
- [ ] Integration with path planning (A* with occupied waypoints)

## See Also

- [Marker Manager](../README.md) - Similar pattern for ArUco marker coordination
- [Mission Control UWB](../../tello_nav/mission_control/mission_control_uwb/) - Client implementation
- [Swarm Interfaces](../swarm_interfaces/) - Message/service definitions
