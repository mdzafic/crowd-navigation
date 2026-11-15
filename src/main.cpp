// Baseline: mjeri cijeli Sim::step() po fazama (mreza, top-k, ORCA).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "map.hpp"
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
    bool map_mode = false;
    Map map;
    std::string map_path;
    std::size_t map_agents = 0;
    int repeats = 7;

    if (argc > 1 && std::string(argv[1]) == "--map")
    {
        if (argc < 3)
        {
            std::fprintf(stderr, "Upotreba: crowd_bench --map <fajl> [N] [ponavljanja]\n");
            return 2;
        }
        map_mode = true;
        map_path = argv[2];
        if (argc > 3) map_agents = std::strtoull(argv[3], nullptr, 10);
        if (argc > 4) repeats = std::atoi(argv[4]);
        if (repeats <= 0 || (argc > 3 && map_agents == 0))
        {
            std::fprintf(stderr, "N i broj ponavljanja moraju biti pozitivni.\n");
            return 2;
        }

        std::string err;
        if (!Map::load(map_path, map, err))
        {
            std::fprintf(stderr, "Greska pri ucitavanju mape: %s\n", err.c_str());
            return 1;
        }
    }
    else
    {
        if (argc > 1) repeats = std::atoi(argv[1]);
        if (repeats <= 0)
        {
            std::fprintf(stderr, "Broj ponavljanja mora biti pozitivan.\n");
            return 2;
        }
    }

    const int warmup_steps = 20;
    const int measure_steps = 100;

    std::vector<std::size_t> sizes;
    if (map_mode)
    {
        const std::size_t free_cells = static_cast<std::size_t>(
            std::count(map.wall.begin(), map.wall.end(), std::uint8_t{0}));
        const std::vector<std::size_t> requested = argc > 3
            ? std::vector<std::size_t>{map_agents}
            : std::vector<std::size_t>{
                std::max<std::size_t>(1, free_cells / 8),
                std::max<std::size_t>(1, free_cells / 4),
                std::max<std::size_t>(1, free_cells / 2),
                free_cells};
        for (const std::size_t requested_agents : requested)
        {
            const std::size_t actual_agents = std::min(requested_agents, free_cells);
            if (actual_agents > 0 &&
                std::find(sizes.begin(), sizes.end(), actual_agents) == sizes.end())
                sizes.push_back(actual_agents);
        }
    }
    else
    {
        sizes = {1000, 5000, 10000, 20000};
    }

    std::printf("scenarij=%s, k=%zu, zagrijavanje %d koraka, mjereni koraci %d..%d, "
                "%d ponavljanja (medijana)\n",
        map_mode ? map_path.c_str() : "antipodalni disk",
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
            const Scenario sc = map_mode
                ? make_map_scenario(map, n, p)
                : make_antipodal_disk(n, p);
            Sim sim = map_mode
                ? Sim(sc.agents, sc.world, p, &map)
                : Sim(sc.agents, sc.world, p);

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