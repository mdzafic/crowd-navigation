#pragma once

#include <cstddef>
#include "agents.hpp"

namespace crowd
{
    std::size_t overlapCount(const Agents& state, float agent_radius);
    float minPairDistance(const Agents& state);
    std::size_t goalReachedCount(const Agents& state, float eps);
    float maxSpeed(const Agents& state);
    float averageSpeed(const Agents& state);
    float averageDistanceToGoal(const Agents& state);
}
