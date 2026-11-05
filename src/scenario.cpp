#include "scenario.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace crowd
{
    Scenario make_antipodal_disk(std::size_t n, const Params& p, float spacing, std::uint64_t seed)
    {
        // Resetka m x m, dovoljno velika da sadrzi n tacaka unutar diska.
        const int m = static_cast<int>(std::ceil(std::sqrt(4.0 * static_cast<double>(n) / 3.14159265358979))) + 3;
        const float half = 0.5f * static_cast<float>(m - 1);

        struct Pt { float x, y, d2; };
        std::vector<Pt> pts;
        pts.reserve(static_cast<std::size_t>(m) * static_cast<std::size_t>(m));
        for (int j = 0; j < m; ++j)
            for (int i = 0; i < m; ++i)
            {
                const float px = (static_cast<float>(i) - half) * spacing;
                const float py = (static_cast<float>(j) - half) * spacing;
                pts.push_back({px, py, px * px + py * py});
            }

        // n tacaka najblizih centru (= disk), deterministicki.
        std::nth_element(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(n), pts.end(),
                         [](const Pt& a, const Pt& b) { return a.d2 < b.d2; });
        pts.resize(n);

        std::mt19937_64 rng(seed);
        std::uniform_real_distribution<float> jit(-0.05f * spacing, 0.05f * spacing);

        Scenario s;
        s.agents.resize(n);
        float rmax = 0.f;
        for (std::size_t i = 0; i < n; ++i)
        {
            const float px = pts[i].x + jit(rng);
            const float py = pts[i].y + jit(rng);
            s.agents.x[i] = px;   s.agents.y[i] = py;
            s.agents.gx[i] = -px; s.agents.gy[i] = -py;
            rmax = std::max(rmax, std::sqrt(px * px + py * py));
        }

        // Svijet = disk + margina.
        const float L = rmax + 2.0f * p.interaction_radius();
        s.world = {-L, -L, L, L};
        return s;
    }
}
