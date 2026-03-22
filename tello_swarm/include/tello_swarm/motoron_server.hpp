#ifndef TELLO_SWARM__MOTORON_SERVER__HPP_
#define TELLO_SWARM__MOTORON_SERVER__HPP_

#include "tello_swarm/swarm_trigger_base.hpp"

namespace tello_swarm
{

/**
 * @brief Motoron server for coordinating swarm motoron operations
 * 
 * Thin wrapper around SwarmTriggerBase that provides motoron-specific
 * configuration. Exposes /motoron_all service that triggers /motoron on all drones.
 */
class MotoronServer : public SwarmTriggerBase
{
public:
    MotoronServer();
};

} // namespace tello_swarm

#endif // TELLO_SWARM__MOTORON_SERVER__HPP_