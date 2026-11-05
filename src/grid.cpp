#include "grid.hpp"
#include <algorithm>
#include <cmath>

namespace crowd
{
    Grid::Grid(const World& world, float cell_size, std::size_t max_agents)
        : min_x_(world.min_x), min_y_(world.min_y), R_(cell_size), inv_R_(1.0f / cell_size)
    {
        W_ = std::max(1, static_cast<int>(std::ceil((world.max_x - world.min_x) * inv_R_)));
        H_ = std::max(1, static_cast<int>(std::ceil((world.max_y - world.min_y) * inv_R_)));
        const std::size_t cells = static_cast<std::size_t>(W_) * static_cast<std::size_t>(H_);
        cell_of_.resize(max_agents);
        sorted_ids_.resize(max_agents);
        cell_start_.assign(cells + 1, 0);
        cursor_.assign(cells, 0);
    }

    void Grid::build(const float* x, const float* y, std::size_t n)
    {
        const std::size_t cells = static_cast<std::size_t>(W_) * static_cast<std::size_t>(H_);

        // 1) indeks celije svakog agenta
        for (std::size_t i = 0; i < n; ++i)
            cell_of_[i] = cell_index(cell_x(x[i]), cell_y(y[i]));

        // 2) histogram (cell_start_[c+1] privremeno drzi broj agenata u c)
        std::fill(cell_start_.begin(), cell_start_.end(), 0);
        for (std::size_t i = 0; i < n; ++i)
            ++cell_start_[cell_of_[i] + 1];

        // 3) prefix sum -> pocetak svake celije; kraj = pocetak sljedece
        for (std::size_t c = 0; c < cells; ++c)
            cell_start_[c + 1] += cell_start_[c];

        // 4) scatter: pozicija = pocetak[celija] + redni broj u celiji
        std::copy(cell_start_.begin(), cell_start_.begin() + static_cast<std::ptrdiff_t>(cells), cursor_.begin());
        for (std::size_t i = 0; i < n; ++i)
            sorted_ids_[cursor_[cell_of_[i]]++] = static_cast<std::int32_t>(i);
    }
}
