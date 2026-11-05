#pragma once
#include <cstddef>
#include <cstdint>
#include "agents.hpp"
#include "params.hpp"

namespace crowd
{
    struct World
    {
        float min_x, min_y, max_x, max_y;
    };

    struct Scenario
    {
        Agents agents;
        World world;   // granice u kojima se agenti krecu (+ margina)
    };

    // Referentni scenarij: N agenata na kvadratnoj resetki unutar diska
    // (gustina odredjena razmakom `spacing`), mali slucajni pomak (jitter)
    // da se razbije savrsena simetrija, cilj svakog agenta = tacka
    // suprotna kroz centar (-x, -y). Svi agenti prolaze kroz sredinu.
    Scenario make_antipodal_disk(std::size_t n, const Params& p,
                                 float spacing = 1.2f,
                                 std::uint64_t seed = 42);
}
