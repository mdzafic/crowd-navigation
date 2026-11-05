#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "scenario.hpp"

namespace crowd
{
    // Uniformna mreza sa celijama stranice R, gradjena counting sortom.
    //   cell_start[c] .. cell_start[c+1]-1  =  pozicije u sorted_ids za celiju c
    //   sorted_ids[k]                       =  ID agenta
    // Svi baferi se alociraju jednom (u konstruktoru), build() ih samo prepisuje.
    class Grid
    {
    public:
        Grid(const World& world, float cell_size, std::size_t max_agents);

        void build(const float* x, const float* y, std::size_t n);

        int width()  const noexcept { return W_; }
        int height() const noexcept { return H_; }
        float cell_size() const noexcept { return R_; }

        int cell_x(float x) const noexcept { return clamp_x(static_cast<int>(std::floor((x - min_x_) * inv_R_))); }
        int cell_y(float y) const noexcept { return clamp_y(static_cast<int>(std::floor((y - min_y_) * inv_R_))); }
        int cell_index(int cx, int cy) const noexcept { return cy * W_ + cx; }

        // Poziva f(j) za svakog kandidata j (ukljucujuci i samog agenta)
        // iz 3x3 celija oko pozicije (x,y).
        // Optimizacija: tri susjedne celije u istom redu su uzastopne u
        // memoriji, pa su to samo 3 raspona umjesto 9.
        template <class F>
        void for_each_candidate(float x, float y, F&& f) const
        {
            const int cx = cell_x(x), cy = cell_y(y);
            const int x0 = std::max(cx - 1, 0), x1 = std::min(cx + 1, W_ - 1);
            const int y0 = std::max(cy - 1, 0), y1 = std::min(cy + 1, H_ - 1);
            for (int yy = y0; yy <= y1; ++yy)
            {
                const std::int32_t b = cell_start_[yy * W_ + x0];
                const std::int32_t e = cell_start_[yy * W_ + x1 + 1];
                for (std::int32_t k = b; k < e; ++k) f(sorted_ids_[k]);
            }
        }

        // Poziva f(j) samo za agente j != self koji su stvarno unutar
        // radijusa. Mreza daje kandidate, a ovaj filter provjerava udaljenost.
        template <class F>
        void for_each_neighbor(std::size_t self, const float* x, const float* y,
                               float radius, F&& f) const
        {
            const float xi = x[self], yi = y[self];
            const float radius2 = radius * radius;
            for_each_candidate(xi, yi, [&](std::int32_t j) {
                if (static_cast<std::size_t>(j) == self) return;
                const float dx = x[j] - xi, dy = y[j] - yi;
                if (dx * dx + dy * dy <= radius2) f(j);
            });
        }

        const std::vector<std::int32_t>& cell_start() const noexcept { return cell_start_; }
        const std::vector<std::int32_t>& sorted_ids() const noexcept { return sorted_ids_; }

    private:
        int clamp_x(int c) const noexcept { return c < 0 ? 0 : (c >= W_ ? W_ - 1 : c); }
        int clamp_y(int c) const noexcept { return c < 0 ? 0 : (c >= H_ ? H_ - 1 : c); }

        float min_x_, min_y_, R_, inv_R_;
        int W_, H_;
        std::vector<std::int32_t> cell_of_;     // [n]     celija svakog agenta
        std::vector<std::int32_t> cell_start_;  // [W*H+1] prefix sum
        std::vector<std::int32_t> cursor_;      // [W*H]   radni bafer za scatter
        std::vector<std::int32_t> sorted_ids_;  // [n]
    };
}
