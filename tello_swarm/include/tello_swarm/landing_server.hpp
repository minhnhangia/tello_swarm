#ifndef TELLO_SWARM__LANDING_SERVER__HPP_
#define TELLO_SWARM__LANDING_SERVER__HPP_

#include "tello_swarm/swarm_trigger_base.hpp"

namespace tello_swarm
{

/**
 * @brief Landing server for coordinating swarm landing operations
 * 
 * Thin wrapper around SwarmTriggerBase that provides landing-specific
 * configuration. Exposes /land_all service that triggers /land on all drones.
 */
class LandingServer : public SwarmTriggerBase
{
public:
    LandingServer();
};

} // namespace tello_swarm

#endif // TELLO_SWARM__LANDING_SERVER__HPP_
