#pragma once
#include <cstddef>

namespace crowd
{
    // Parametri simulacije (default vrijednosti slicne RVO2 primjerima).
    struct Params
    {
        float radius = 0.5f;         // r_i, isti za sve agente
        float v_max = 1.4f;          // m/s
        float horizon_T = 2.0f;      // vremenski horizont T [s]
        float dt = 0.25f;            // korak simulacije [s]
        std::size_t max_neighbors = 10; // VAZNO: od ukuonog broja susjeda bira se 10 samo zbog kompleksnosti ORCA, ali mozda nece biti dovoljno u situacijama velike guzve, pa je moguce da ce kasnije biti potrebno optimizovati        

        // Radijus interakcije: R >= v_max * T + r_i + r_j
        // Ovo je i stranica kvadratne celije prostorne mreze.
        float interaction_radius() const noexcept
        {
            return v_max * horizon_T + 2.0f * radius;
        }
    };
}
