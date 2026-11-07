#include <cmath>
#include <cstddef>
#include <cstdio>
#include "scenario.hpp"
#include "sim.hpp"

int main()
{
    crowd::Params single_agent_params;
    crowd::Agents single_agent;
    single_agent.resize(1);
    single_agent.x[0] = 0.f;
    single_agent.y[0] = 0.f;
    single_agent.gx[0] = 10.f;
    single_agent.gy[0] = 0.f;
    const crowd::World single_agent_world{-10.f, -10.f, 10.f, 10.f};
    crowd::Sim single_agent_sim(single_agent, single_agent_world, single_agent_params);
    single_agent_sim.step();
    const crowd::Agents& single_agent_state = single_agent_sim.state();
    if (std::fabs(single_agent_state.vx[0] - single_agent_params.v_max) > 1.0e-5f ||
        std::fabs(single_agent_state.vy[0]) > 1.0e-5f ||
        std::fabs(single_agent_state.x[0] - single_agent_params.v_max * single_agent_params.dt) > 1.0e-5f ||
        std::fabs(single_agent_state.y[0]) > 1.0e-5f)
    {
        std::puts("FAIL: single agent with ORCA should move directly toward its goal");
        return 1;
    }

    crowd::Params p;
    p.max_neighbors = 10;
    const crowd::Scenario scenario = crowd::make_antipodal_disk(200, p, 0.6f);
    crowd::Sim sim(scenario.agents, scenario.world, p);

    sim.step();

    const float radius2 = p.interaction_radius() * p.interaction_radius();
    for (std::size_t i = 0; i < scenario.agents.size(); ++i)
    {
        if (sim.neighbor_count(i) > p.max_neighbors) {
            std::printf("count failure i=%zu count=%zu\n", i, sim.neighbor_count(i));
            return 1;
        }
        const std::int32_t* ids = sim.neighbors(i);
        float previous_d2 = -1.f;
        for (std::size_t k = 0; k < sim.neighbor_count(i); ++k)
        {
            const std::size_t j = static_cast<std::size_t>(ids[k]);
            if (j >= scenario.agents.size() || j == i) {
                std::printf("id failure i=%zu k=%zu j=%zu\n", i, k, j);
                return 1;
            }
            const float dx = scenario.agents.x[j] - scenario.agents.x[i];
            const float dy = scenario.agents.y[j] - scenario.agents.y[i];
            const float d2 = dx * dx + dy * dy;
            if (d2 > radius2 || d2 < previous_d2) {
                std::printf("distance failure i=%zu k=%zu d2=%f prev=%f radius2=%f\n",
                            i, k, d2, previous_d2, radius2);
                return 1;
            }
            previous_d2 = d2;
        }
    }
    std::puts("OK sim_neighbors");
    return 0;
}
