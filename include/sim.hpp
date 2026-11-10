#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include "agents.hpp"
#include "grid.hpp"
#include "orca.hpp"
#include "params.hpp"
#include "navfield.hpp"
#include "map.hpp"

namespace crowd
{
    // Vrijeme faza jednog koraka (ms); popunjava se samo ako je zadan pokazivac.
    struct StepTimes
    {
        double grid_ms = 0, topk_ms = 0, orca_ms = 0;
    };

    // Tanak interfejs izmedju simulacije i vizualizatora.
    class Sim
    {
    public:
        Sim(Agents initial, World world, const Params& p);

        const Agents& state() const noexcept { return cur_; }
        void step(StepTimes* times = nullptr);   // jedan korak od p.dt
        std::size_t steps() const noexcept { return steps_; }
        std::size_t reached() const noexcept; // broj agenata na cilju
        std::size_t neighbor_count(std::size_t i) const noexcept { return neighbor_counts_[i]; }
        const std::int32_t* neighbors(std::size_t i) const noexcept
        {
            return neighbor_ids_.data() + i * p_.max_neighbors;
        }
        Sim(Agents initial, World world, const Params& p, const Map* map);  // novi konstruktor

        const NavFields& nav() const noexcept { return nav_; }
        int agent_field(std::size_t i) const noexcept { return agent_field_[i]; }

    private:
        Params p_;
        Agents cur_, next_;                   // dvostruko baferovanje
        Grid grid_;
        std::vector<std::int32_t> neighbor_ids_;
        std::vector<float> neighbor_distances_;
        std::vector<std::size_t> neighbor_counts_;
        std::vector<Line> orca_line_buffers_;
        std::size_t steps_ = 0;
        const Map* map_ = nullptr;
        NavFields nav_;
        std::vector<int> agent_field_;
        std::vector<float> goal_x_, goal_y_;   // pravi ciljevi (cur_.gx se u koraku privremeno mijenja)
        std::size_t line_stride_ = 0;
    };
}