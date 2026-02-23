# tello_swarm

Centralized swarm coordination servers for multi-drone Tello missions. Manages global resource conflicts — ArUco marker reservations, waypoint mutual exclusion, and synchronized takeoff/landing — so multiple drones can operate safely in a shared environment.

## Repository Structure

```
tello_swarm/
├── swarm_interfaces/          # Custom ROS 2 messages & services
│   ├── msg/
│   │   ├── MarkerHeartbeat.msg
│   │   ├── MarkerInfo.msg
│   │   ├── MarkerRegistry.msg
│   │   └── WaypointHeartbeat.msg
│   └── srv/
│       ├── MarkLanded.srv
│       ├── ReserveMarker.srv
│       └── ReserveWaypoint.srv
└── tello_swarm/               # C++ server nodes
    ├── config/
    │   └── swarm_params.yaml
    ├── docs/
    │   └── WAYPOINT_SERVER.md
    ├── include/tello_swarm/
    │   ├── marker_manager.hpp
    │   ├── waypoint_server.hpp
    │   ├── swarm_trigger_base.hpp
    │   ├── takeoff_server.hpp
    │   └── landing_server.hpp
    ├── launch/
    │   ├── swarm_servers_launch.py
    │   └── waypoint_server_launch.py
    └── src/
        ├── marker_manager.cpp
        ├── waypoint_server.cpp
        ├── swarm_trigger_base.cpp
        ├── takeoff_server.cpp
        └── landing_server.cpp
```

## Architecture Overview

All servers run as **global singletons** (one instance per swarm, not per drone). Individual drones interact with them via ROS 2 services and topics.

```
                          ┌──────────────────────┐
                          │   swarm_servers      │
                          │   (single process or │
                          │    separate nodes)   │
                          └──────────┬───────────┘
           ┌──────────────┬──────────┼─────────────────┐
           ▼              ▼          ▼                 ▼
   ┌──────────────┐ ┌───────────┐ ┌──────────┐ ┌───────────┐
   │ marker_      │ │ waypoint_ │ │ takeoff_ │ │ landing_  │
   │ manager      │ │ server    │ │ server   │ │ server    │
   └──────┬───────┘ └─────┬─────┘ └────┬─────┘ └─────┬─────┘
          │               │            │             │
    /reserve_marker  /reserve_waypoint  /takeoff_all   /land_all
    /unreserve_marker /unreserve_waypoint
    /mark_landed      /waypoint_heartbeat
    /marker_heartbeat /unavailable_waypoints
    /unavailable_markers
    /marker_registry
```

## Nodes

### marker_manager

Manages ArUco marker reservations with a THREE-state lifecycle. Prevents multiple drones from targeting or landing on the same marker.

| Marker IDs | Type     | Purpose            |
|------------|----------|--------------------|
| 1–8        | `VICTIM` | Victim markers     |
| 9–14       | `FIRE`   | Fire markers       |

**Marker State Machine:**

```
    ┌─────────┐                     ┌─────────────┐
    │  FREE   │── reserve_marker ──▶│  RESERVED   │
    └─────────┘                     └──────┬──────┘
         ▲                                 │
         │ unreserve_marker                │ mark_landed
         │ OR expiration (~6s)             ▼
         │                          ┌─────────────┐
         └──────────────────────────│   LANDED    │
                                    └─────────────┘
                                   (permanent, no expiry)
```

- **FREE** → available for any drone to reserve
- **RESERVED** → exclusively held; requires heartbeat renewal every ~6 seconds or expires back to FREE
- **LANDED** → permanently occupied; a drone has confirmed landing on this marker (does not expire)

**ROS 2 Interface:**

| Type | Name | Interface | Description |
|------|------|-----------|-------------|
| Service | `/reserve_marker` | `swarm_interfaces/srv/ReserveMarker` | Reserve a marker (or renew if same owner) |
| Service | `/unreserve_marker` | `swarm_interfaces/srv/ReserveMarker` | Release a reservation (owner only) |
| Service | `/mark_landed` | `swarm_interfaces/srv/MarkLanded` | Confirm landing on a reserved marker (owner only) |
| Subscriber | `/marker_heartbeat` | `swarm_interfaces/msg/MarkerHeartbeat` | Heartbeat to renew RESERVED state |
| Publisher | `/unavailable_markers` | `std_msgs/msg/Int32MultiArray` | IDs of non-FREE markers (transient local QoS) |
| Publisher | `/marker_registry` | `swarm_interfaces/msg/MarkerRegistry` | Full registry snapshot (every 500 ms) |

---

### waypoint_server

Manages mutual exclusion for waypoint indices. Ensures only one drone navigates to any given waypoint at a time.

**Waypoint State Machine:**

```
    ┌─────────┐                        ┌─────────────┐
    │  FREE   │── reserve_waypoint ──▶ │  RESERVED   │
    └─────────┘                        └──────┬──────┘
         ▲                                    │
         │ unreserve_waypoint                 │ heartbeat (renew)
         │ OR expiration (default 8s)         ▼
         └────────────────────────────────────┘
```

**ROS 2 Interface:**

| Type | Name | Interface | Description |
|------|------|-----------|-------------|
| Service | `/reserve_waypoint` | `swarm_interfaces/srv/ReserveWaypoint` | Reserve a waypoint by index |
| Service | `/unreserve_waypoint` | `swarm_interfaces/srv/ReserveWaypoint` | Release a waypoint reservation |
| Subscriber | `/waypoint_heartbeat` | `swarm_interfaces/msg/WaypointHeartbeat` | Heartbeat to renew reservation |
| Publisher | `/unavailable_waypoints` | `std_msgs/msg/Int32MultiArray` | Reserved waypoint indices (transient local QoS) |

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_waypoints` | `int` | `100` | Number of waypoint slots (indices 0 to N−1) |
| `expiration_duration_s` | `double` | `8.0` | Seconds without heartbeat before forced expiration |

See [docs/WAYPOINT_SERVER.md](tello_swarm/docs/WAYPOINT_SERVER.md) for detailed usage, failure modes, and integration examples.

---

### takeoff_server

Triggers synchronized takeoff across all configured drones. Calls each drone's individual `/takeoff` service in parallel and waits for all responses.

**ROS 2 Interface:**

| Type | Name | Interface | Description |
|------|------|-----------|-------------|
| Service | `/takeoff_all` | `std_srvs/srv/Trigger` | Trigger takeoff for every configured drone |

---

### landing_server

Triggers synchronized landing across all configured drones. Same pattern as `takeoff_server`.

**ROS 2 Interface:**

| Type | Name | Interface | Description |
|------|------|-----------|-------------|
| Service | `/land_all` | `std_srvs/srv/Trigger` | Trigger landing for every configured drone |

---

### SwarmTriggerBase (shared base class)

Abstract C++ base that `takeoff_server` and `landing_server` both inherit from. Provides:

- Parameter-driven drone list (`drone_ids`)
- Pre-created persistent service clients per drone
- Parallel async service dispatch with configurable timeout
- Reentrant callback group + `MultiThreadedExecutor` for concurrent handling

**Parameters (shared by takeoff_server / landing_server):**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `drone_ids` | `string[]` | `["tello1"]` | List of drone namespace IDs |
| `takeoff_service_name` / `landing_service_name` | `string` | `"takeoff"` / `"land"` | Per-drone service to call |
| `client_timeout_sec` | `double` | `5.0` | Timeout waiting for each drone's response |

## Custom Interfaces (swarm_interfaces)

### Messages

| Message | Fields | Purpose |
|---------|--------|---------|
| `MarkerHeartbeat` | `string drone_id`, `int32 marker_id` | Keep marker reservation alive |
| `WaypointHeartbeat` | `Header header`, `string drone_id`, `int32 waypoint_index` | Keep waypoint reservation alive |
| `MarkerInfo` | `uint32 marker_id`, `uint8 state`, `uint8 type`, `string owner`, `float64 time_since_update` | Single marker status |
| `MarkerRegistry` | `Header header`, `MarkerInfo[] markers` | Full registry snapshot |

### Services

| Service | Request | Response | Purpose |
|---------|---------|----------|---------|
| `ReserveMarker` | `int32 marker_id`, `string drone_id` | `bool success`, `string message` | Reserve/unreserve a marker |
| `MarkLanded` | `int32 marker_id`, `string drone_id` | `bool success`, `string message` | Confirm landing on marker |
| `ReserveWaypoint` | `int32 waypoint_index`, `string drone_id` | `bool success`, `string message` | Reserve/unreserve a waypoint |

## Heartbeat Pattern

Both `marker_manager` and `waypoint_server` use a **heartbeat-based lease** to prevent stale reservations from permanently blocking resources:

1. **Drone reserves** a resource via service call.
2. **Drone publishes heartbeats** every ~2 seconds on `/marker_heartbeat` or `/waypoint_heartbeat`.
3. **Server refreshes** the `last_update` timestamp on each heartbeat.
4. **Server expires** reservations that exceed the cooldown duration (~6 s for markers, ~8 s for waypoints) without a heartbeat renewal.

This guarantees that if a drone crashes, disconnects, or fails silently, its reservations are automatically released.

## Configuration

### swarm_params.yaml

```yaml
takeoff_server:
  ros__parameters:
    drone_ids: ["tello1", "tello2"]
    takeoff_service_name: "takeoff"
    client_timeout_sec: 5.0
```

Adjust `drone_ids` to match the namespaces of all drones in your swarm.

## Usage

### Launch All Swarm Servers

```bash
ros2 launch tello_bringup swarm_servers_launch.py
```

### Debugging

```bash
# Current marker reservations
ros2 topic echo /unavailable_markers

# Full marker registry with state, type, owner, and timing
ros2 topic echo /marker_registry

# Currently reserved waypoints
ros2 topic echo /unavailable_waypoints

# Verify services are available
ros2 service list | grep -E "reserve|landed|takeoff|land"

# Trigger synchronized takeoff/landing manually
ros2 service call /takeoff_all std_srvs/srv/Trigger
ros2 service call /land_all std_srvs/srv/Trigger

# Manual marker reservation test
ros2 service call /reserve_marker swarm_interfaces/srv/ReserveMarker \
  "{marker_id: 3, drone_id: 'test_drone'}"

# Manual waypoint reservation test
ros2 service call /reserve_waypoint swarm_interfaces/srv/ReserveWaypoint \
  "{waypoint_index: 0, drone_id: 'test_drone'}"
```

## Building

```bash
cd ~/tello_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swarm_interfaces tello_swarm
source install/setup.bash
```

> **Note:** `swarm_interfaces` must be built before `tello_swarm` (colcon handles this automatically via declared dependencies).

## Dependencies

| Dependency | Type | Purpose |
|------------|------|---------|
| `rclcpp` | ROS 2 | C++ client library |
| `std_srvs` | ROS 2 | `Trigger` service for takeoff/landing |
| `std_msgs` | ROS 2 | `Int32MultiArray` for unavailable lists, `Header` |
| `swarm_interfaces` | Local | Custom messages and services |
| `yaml-cpp` | System | YAML parsing (waypoint_server) |

## License

Apache-2.0