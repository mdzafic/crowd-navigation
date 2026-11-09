#pragma once
#include <vector>
#include "map.hpp"

namespace crowd
{
    struct NavField
    {
        int w = 0, h = 0;
        float cell = 1.f;
        int goal_cell = -1;
        std::vector<float> cost;          // 1e30 = nedostizno / zid
        std::vector<float> dirx, diry;    // jedinièni smjer prema cilju

        void build(const Map& m, int goal_cx, int goal_cy,
                   const std::vector<float>& wall_dist, float margin);
        // Bilinearno glaðen smjer; false ako nema validnih susjednih æelija.
        bool sample(float x, float y, float& ox, float& oy) const;
    };

    class NavFields
    {
    public:
        // Gradi po jedno polje za svaku jedinstvenu ciljnu æeliju.
        // Vraæa indeks polja za svakog agenta.
        std::vector<int> build(const Map& m, const std::vector<float>& gx,
                               const std::vector<float>& gy, float radius);
        const NavField& at(int i) const { return fields_[static_cast<std::size_t>(i)]; }
    private:
        std::vector<NavField> fields_;
    };
}