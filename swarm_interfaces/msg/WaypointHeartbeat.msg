# Heartbeat message for waypoint reservation renewal
# Published periodically by drones to keep their waypoint reservations active

std_msgs/Header header

# Drone identifier
string drone_id

# Reserved waypoint index (0-based sequence)
int32 waypoint_index
