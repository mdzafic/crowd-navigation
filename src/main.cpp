// Baseline: mjeri cijeli Sim::step() po fazama (mreza, top-k, ORCA).
// Opcije: --csv <fajl> (dodaje redove), --label <oznaka eksperimenta>
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
static double vmin(const std::vector<double>& v) { return *std::min_element(v.begin(), v.end()); }
static double vmax(const std::vector<double>& v) { return *std::max_element(v.begin(), v.end()); }

int main(int argc, char** argv)
{
    // Izdvoji opcije --csv i --label; ostalo su pozicioni argumenti.
    std::string csv_path, label = "none";
    std::size_t force_n = 0;
    std::vector<std::string> a;
    a.push_back(argv[0]);
    for (int i = 1; i < argc; ++i)
    {
        const std::string s = argv[i];
        if (s == "--csv" && i + 1 < argc)
            csv_path = argv[++i];
        else if (s == "--label" && i + 1 < argc)
            label = argv[++i];
        else if (s == "--n" && i + 1 < argc)
            force_n = std::strtoull(argv[++i], nullptr, 10);
        else
            a.push_back(s);
    }
    const int ac = static_cast<int>(a.size());

    bool map_mode = false;
    Map map;
    std::string map_path;
    std::size_t map_agents = 0;
    int repeats = 7;

    if (ac > 1 && a[1] == "--map")
    {
        if (ac < 3)
        {
            std::fprintf(stderr, "Upotreba: crowd_bench --map <fajl> [N] [ponavljanja] [--csv f] [--label x]\n");
            return 2;
        }
        map_mode = true;
        map_path = a[2];
        if (ac > 3) map_agents = std::strtoull(a[3].c_str(), nullptr, 10);
        if (ac > 4) repeats = std::atoi(a[4].c_str());
        if (repeats <= 0 || (ac > 3 && map_agents == 0))
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
        if (ac > 1) repeats = std::atoi(a[1].c_str());
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
            std::count(map.wall.begin(), map.wall.end(), std::uint8_t{ 0 }));
        const std::vector<std::size_t> requested = ac > 3
            ? std::vector<std::size_t>{map_agents}
        : std::vector<std::size_t>{
            std::max<std::size_t>(1, free_cells / 8),
            std::max<std::size_t>(1, free_cells / 4),
            std::max<std::size_t>(1, free_cells / 2),
            free_cells };
        for (const std::size_t ra : requested)
        {
            const std::size_t actual = std::min(ra, free_cells);
            if (actual > 0 && std::find(sizes.begin(), sizes.end(), actual) == sizes.end())
                sizes.push_back(actual);
        }
    }
    else
    {
        sizes = { 1000, 5000, 10000, 20000 };
    }

    if (force_n > 0)
        sizes = { force_n };

    const std::string scen_name = map_mode ? map_path : "antipodal_disk";

    // CSV: zaglavlje samo ako je fajl prazan/nov.
    std::FILE* csv = nullptr;
    if (!csv_path.empty())
    {
        csv = std::fopen(csv_path.c_str(), "ab+");
        if (!csv) { std::fprintf(stderr, "Ne mogu otvoriti %s\n", csv_path.c_str()); return 1; }
        std::fseek(csv, 0, SEEK_END);
        if (std::ftell(csv) == 0)
            std::fprintf(csv, "label,scenario,k,warmup,steps,N,repeat,grid_ms,topk_ms,orca_ms,"
                "total_ms,us_per_agent,avg_neighbors,frac_at_k,reached\n");
    }

    std::printf("label=%s scenarij=%s, k=%zu, zagrijavanje %d koraka, mjereni koraci %d..%d, "
        "%d ponavljanja (medijana)\n",
        label.c_str(), scen_name.c_str(), Params{}.max_neighbors,
        warmup_steps, warmup_steps + 1, warmup_steps + measure_steps, repeats);
    std::printf("%7s %10s %10s %10s %10s %10s %10s %8s %8s\n",
        "N", "mreza ms", "top-k ms", "ORCA ms", "ukupno ms", "min", "max", "us/agent", "avg_nb");

    for (std::size_t n : sizes)
    {
        std::vector<double> grid, topk, orca, total, avgnb;
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
            const double gm = g / measure_steps, tm = t / measure_steps, om = o / measure_steps;
            grid.push_back(gm); topk.push_back(tm); orca.push_back(om);
            total.push_back(gm + tm + om);

            // Opterecenje na kraju mjerenja (stanje nakon posljednjeg koraka).
            double sum_nb = 0; std::size_t at_k = 0;
            for (std::size_t i = 0; i < n; ++i)
            {
                const std::size_t c = sim.neighbor_count(i);
                sum_nb += static_cast<double>(c);
                if (c >= p.max_neighbors) ++at_k;
            }
            const double nb = sum_nb / static_cast<double>(n);
            const double fk = static_cast<double>(at_k) / static_cast<double>(n);
            avgnb.push_back(nb);

            if (csv)
                std::fprintf(csv, "%s,%s,%zu,%d,%d,%zu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.4f,%zu\n",
                    label.c_str(), scen_name.c_str(), p.max_neighbors, warmup_steps,
                    measure_steps, n, r, gm, tm, om, gm + tm + om,
                    (gm + tm + om) * 1000.0 / static_cast<double>(n), nb, fk, sim.reached());
        }
        const double tot = median(total);
        std::printf("%7zu %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f %8.3f %8.2f\n",
            n, median(grid), median(topk), median(orca), tot, vmin(total), vmax(total),
            tot * 1000.0 / static_cast<double>(n), median(avgnb));
    }
    if (csv) std::fclose(csv);

#ifdef CROWD_DIAG
    std::printf("DIAG ukupno: kandidata=%llu, u_radijusu=%llu, ubaceno=%llu\n",
        g_cand, g_inradius, g_inserted);
#endif

    return 0;
}