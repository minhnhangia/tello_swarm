#ifndef TELLO_SWARM__TAKEOFF_SERVER__HPP_
#define TELLO_SWARM__TAKEOFF_SERVER__HPP_

#include "tello_swarm/swarm_trigger_base.hpp"

namespace tello_swarm
{

/**
 * @brief Takeoff server for coordinating swarm takeoff operations
 * 
 * Thin wrapper around SwarmTriggerBase that provides takeoff-specific
 * configuration. Exposes /takeoff_all service that triggers /takeoff on all drones.
 */
class TakeoffServer : public SwarmTriggerBase
{
public:
    TakeoffServer();
};

} // namespace tello_swarm

#endif // TELLO_SWARM__TAKEOFF_SERVER__HPP_