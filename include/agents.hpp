#pragma once
#include <cstddef>
#include <vector>

namespace crowd
{
    // Structure of Arrays: svaki atribut u svom nizu (cache i SIMD friendly).
    struct Agents
    {
        std::vector<float> x, y;      // pozicija
        std::vector<float> vx, vy;    // trenutna brzina
        std::vector<float> gx, gy;    // cilj

        std::size_t size() const noexcept { return x.size(); }

        void resize(std::size_t n)
        {
            x.resize(n);  y.resize(n);
            vx.assign(n, 0.f); vy.assign(n, 0.f);
            gx.resize(n); gy.resize(n);
        }
    };
}
