// Faza 0-3: scenarij + prostorna mreza. Mjeri gradnju mreze i pretragu susjeda.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "grid.hpp"
#include "scenario.hpp"

using clk = std::chrono::steady_clock;

template <class F>
static double median_ms(int repeats, F&& f)
{
    f(); // warmup
    std::vector<double> t;
    for (int r = 0; r < repeats; ++r)
    {
        const auto a = clk::now();
        f();
        const auto b = clk::now();
        t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}

int main(int argc, char** argv)
{
    const std::size_t n = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 10000;
    const int repeats = argc > 2 ? std::atoi(argv[2]) : 20;

    crowd::Params p;
    crowd::Scenario s = crowd::make_antipodal_disk(n, p);
    crowd::Agents& a = s.agents;
    const float R = p.interaction_radius();
    crowd::Grid grid(s.world, R, n);
    std::printf("N=%zu  R=%.2f  mreza %dx%d (%zu celija)\n", n, R,
                grid.width(), grid.height(),
                static_cast<std::size_t>(grid.width()) * static_cast<std::size_t>(grid.height()));

    const double t_build = median_ms(repeats, [&] { grid.build(a.x.data(), a.y.data(), n); });

    // Pretraga: za svakog agenta prodji 3x3 celije i broji prave susjede (d <= R).
    std::size_t total_cand = 0, total_nb = 0;
    const double t_query = median_ms(repeats, [&] {
        std::size_t cand = 0, nb = 0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const float xi = a.x[i], yi = a.y[i];
            grid.for_each_candidate(xi, yi, [&](int) {
                ++cand;
            });
            grid.for_each_neighbor(i, a.x.data(), a.y.data(), R,
                                   [&](int) { ++nb; });
        }
        total_cand = cand; total_nb = nb;
    });

    std::printf("gradnja mreze : %8.3f ms\n", t_build);
    std::printf("pretraga 3x3  : %8.3f ms  (kandidata/agent %.1f, susjeda u R/agent %.1f)\n",
                t_query, double(total_cand) / double(n), double(total_nb) / double(n));
    return 0;
}
