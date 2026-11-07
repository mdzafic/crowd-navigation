#include "metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crowd
{
    std::size_t overlapCount(const Agents& state, float agent_radius)
    {
        std::size_t count = 0;
        const float diameter2 = 4.0f * agent_radius * agent_radius;
        for (std::size_t i = 0; i < state.size(); ++i)
            for (std::size_t j = i + 1; j < state.size(); ++j)
            {
                const float dx = state.x[i] - state.x[j];
                const float dy = state.y[i] - state.y[j];
                if (dx * dx + dy * dy < diameter2)
                    ++count;
            }
        return count;
    }

    float minPairDistance(const Agents& state)
    {
        if (state.size() < 2)
            return std::numeric_limits<float>::infinity();

        float min_distance2 = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < state.size(); ++i)
            for (std::size_t j = i + 1; j < state.size(); ++j)
            {
                const float dx = state.x[i] - state.x[j];
                const float dy = state.y[i] - state.y[j];
                min_distance2 = std::min(min_distance2, dx * dx + dy * dy);
            }
        return std::sqrt(min_distance2);
    }

    std::size_t goalReachedCount(const Agents& state, float eps)
    {
        const float eps2 = eps * eps;
        std::size_t count = 0;
        for (std::size_t i = 0; i < state.size(); ++i)
        {
            const float dx = state.gx[i] - state.x[i];
            const float dy = state.gy[i] - state.y[i];
            if (dx * dx + dy * dy <= eps2)
                ++count;
        }
        return count;
    }

    float maxSpeed(const Agents& state)
    {
        float max_speed2 = 0.0f;
        for (std::size_t i = 0; i < state.size(); ++i)
            max_speed2 = std::max(max_speed2,
                                  state.vx[i] * state.vx[i] + state.vy[i] * state.vy[i]);
        return std::sqrt(max_speed2);
    }

    float averageSpeed(const Agents& state)
    {
        if (state.size() == 0)
            return 0.0f;

        float total_speed = 0.0f;
        for (std::size_t i = 0; i < state.size(); ++i)
            total_speed += std::sqrt(state.vx[i] * state.vx[i] + state.vy[i] * state.vy[i]);
        return total_speed / static_cast<float>(state.size());
    }

    float averageDistanceToGoal(const Agents& state)
    {
        if (state.size() == 0)
            return 0.0f;

        float total_distance = 0.0f;
        for (std::size_t i = 0; i < state.size(); ++i)
        {
            const float dx = state.gx[i] - state.x[i];
            const float dy = state.gy[i] - state.y[i];
            total_distance += std::sqrt(dx * dx + dy * dy);
        }
        return total_distance / static_cast<float>(state.size());
    }
}
