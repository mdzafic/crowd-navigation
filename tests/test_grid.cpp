// Provjera: susjedi iz mreze (3x3 celije + filtar d <= R) moraju biti
// IDENTICNI skupu iz brute-force O(N^2) pretrage.
#include <algorithm>
#include <cstdio>
#include <vector>
#include "grid.hpp"
#include "scenario.hpp"

using namespace crowd;

static bool check(std::size_t n, float spacing, bool shuffle_to_edges)
{
    Params p;
    Scenario s = make_antipodal_disk(n, p, spacing);
    Agents& a = s.agents;
    const float R = p.interaction_radius();

    if (shuffle_to_edges)
    {
        // Rubni slucaj: dio agenata van granica svijeta -> clamp mora raditi.
        for (std::size_t i = 0; i < n; i += 7) { a.x[i] += 3.0f * s.world.max_x; a.y[i] -= 3.0f * s.world.max_y; }
    }

    Grid grid(s.world, R, n);
    grid.build(a.x.data(), a.y.data(), n);

    std::size_t pairs = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
        std::vector<int> from_grid, brute;
        grid.for_each_neighbor(i, a.x.data(), a.y.data(), R,
                               [&](int j) { from_grid.push_back(j); });
        for (std::size_t j = 0; j < n; ++j)
        {
            if (j == i) continue;
            const float dx = a.x[j] - a.x[i], dy = a.y[j] - a.y[i];
            if (dx * dx + dy * dy <= R * R) brute.push_back(static_cast<int>(j));
        }
        std::sort(from_grid.begin(), from_grid.end());
        if (from_grid != brute)
        {
            std::printf("GRESKA: agent %zu, grid=%zu brute=%zu susjeda\n", i, from_grid.size(), brute.size());
            return false;
        }
        pairs += brute.size();
    }

    // k_nearest: za k = 1, 10, 64 i subdiv = 1..4 udaljenosti moraju biti iste
    // kao k najmanjih iz brute-force pretrage, sortirane, bez samog agenta.
    for (int subdiv = 1; subdiv <= 4; ++subdiv)
    {
        Grid g(s.world, R, n, subdiv);
        g.build(a.x.data(), a.y.data(), n);
        for (std::size_t k : { std::size_t{ 1 }, std::size_t{ 10 }, std::size_t{ 64 } })
        {
            std::vector<std::int32_t> ids(k);
            std::vector<float> d2s(k);
            for (std::size_t i = 0; i < n; ++i)
            {
                std::vector<float> brute;
                for (std::size_t j = 0; j < n; ++j)
                {
                    if (j == i) continue;
                    const float dx = a.x[j] - a.x[i], dy = a.y[j] - a.y[i];
                    const float d2 = dx * dx + dy * dy;
                    if (d2 <= R * R) brute.push_back(d2);
                }
                std::sort(brute.begin(), brute.end());
                if (brute.size() > k) brute.resize(k);

                const std::size_t c = g.k_nearest(i, R, k, ids.data(), d2s.data());
                bool ok = c == brute.size();
                for (std::size_t t = 0; ok && t < c; ++t)
                {
                    const std::size_t j = static_cast<std::size_t>(ids[t]);
                    const float dx = a.x[j] - a.x[i], dy = a.y[j] - a.y[i];
                    ok = j != i && d2s[t] == brute[t] && dx * dx + dy * dy == d2s[t];
                }
                if (!ok)
                {
                    std::printf("GRESKA k_nearest: agent %zu, k=%zu subdiv=%d (dobijeno %zu, ocekivano %zu)\n",
                                i, k, subdiv, c, brute.size());
                    return false;
                }
            }
        }
    }

    // Svaki agent mora biti u sorted_ids tacno jednom.
    std::vector<int> seen(n, 0);
    for (std::int32_t id : grid.sorted_ids()) if (id >= 0 && static_cast<std::size_t>(id) < n) ++seen[id];
    for (std::size_t i = 0; i < n; ++i)
        if (seen[i] != 1) { std::printf("GRESKA: agent %zu zastupljen %d puta\n", i, seen[i]); return false; }

    std::printf("OK  n=%zu spacing=%.1f edges=%d  (ukupno %zu parova)\n", n, spacing, shuffle_to_edges, pairs);
    return true;
}

int main()
{
    bool ok = true;
    ok &= check(1, 1.2f, false);
    ok &= check(50, 1.2f, false);
    ok &= check(800, 1.2f, false);
    ok &= check(800, 0.6f, false);   // vrlo gusto
    ok &= check(800, 3.0f, false);   // rijetko
    ok &= check(800, 1.2f, true);    // agenti van granica
    std::puts(ok ? "SVI TESTOVI PROSLI" : "TESTOVI PAO");
    return ok ? 0 : 1;
}
