// Baseline: mjeri cijeli Sim::step() po fazama (mreza, top-k, ORCA).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "scenario.hpp"
#include "sim.hpp"

using namespace crowd;

static double median(std::vector<double> v)
{
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

int main(int argc, char** argv)
{
    const int repeats = argc > 1 ? std::atoi(argv[1]) : 7;
    const int warmup_steps = 20;
    const int measure_steps = 100;

    const std::size_t sizes[] = { 1000, 5000, 10000, 20000 };

    std::printf("k=%zu, zagrijavanje %d koraka, mjereni koraci %d..%d, %d ponavljanja (medijana)\n",
        Params{}.max_neighbors, warmup_steps, warmup_steps + 1,
        warmup_steps + measure_steps, repeats);
    std::printf("%7s %10s %10s %10s %10s %10s\n",
        "N", "mreza ms", "top-k ms", "ORCA ms", "ukupno ms", "us/agent");

    for (std::size_t n : sizes)
    {
        std::vector<double> grid, topk, orca, total;
        for (int r = 0; r < repeats; ++r)
        {
            Params p;
            const Scenario sc = make_antipodal_disk(n, p);
            Sim sim(sc.agents, sc.world, p);

            for (int s = 0; s < warmup_steps; ++s) sim.step();

            double g = 0, t = 0, o = 0;
            StepTimes st;
            for (int s = 0; s < measure_steps; ++s)
            {
                sim.step(&st);
                g += st.grid_ms; t += st.topk_ms; o += st.orca_ms;
            }
            grid.push_back(g / measure_steps);
            topk.push_back(t / measure_steps);
            orca.push_back(o / measure_steps);
            total.push_back((g + t + o) / measure_steps);
        }
        const double tot = median(total);
        std::printf("%7zu %10.3f %10.3f %10.3f %10.3f %10.3f\n",
            n, median(grid), median(topk), median(orca), tot,
            tot * 1000.0 / static_cast<double>(n));
    }
    return 0;
}