#include "grid.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace crowd
{
    Grid::Grid(const World& world, float cell_size, std::size_t max_agents, int subdiv)
        : min_x_(world.min_x), min_y_(world.min_y),
        R_(cell_size / static_cast<float>(subdiv)),
        inv_R_(static_cast<float>(subdiv) / cell_size),
        reach_(std::clamp(subdiv, 1, MAX_REACH))
    {
        W_ = std::max(1, static_cast<int>(std::ceil((world.max_x - world.min_x) * inv_R_)));
        H_ = std::max(1, static_cast<int>(std::ceil((world.max_y - world.min_y) * inv_R_)));
        const std::size_t cells = static_cast<std::size_t>(W_) * static_cast<std::size_t>(H_);
        cell_of_.resize(max_agents);
        sorted_ids_.resize(max_agents);
        pos_of_.resize(max_agents);
        sx_.resize(max_agents);
        sy_.resize(max_agents);
        cell_start_.assign(cells + 1, 0);
        cursor_.assign(cells, 0);

        // Rezerva za donju granicu udaljenosti (zaokruzivanje floor((x-min)*inv_R)
        // naspram min + c*R). Samo malo smanjuje odsijecanje, nikad ne mijenja rezultat.
        slack_ = 1.0e-4f * R_ + 1.0e-5f * std::max({ std::fabs(world.min_x), std::fabs(world.max_x),
                                                     std::fabs(world.min_y), std::fabs(world.max_y) });

        // Pomaci (2*reach+1)^2 celija, sortirani po prstenu pa po udaljenosti centra.
        for (int dy = -reach_; dy <= reach_; ++dy)
            for (int dx = -reach_; dx <= reach_; ++dx)
                order_.push_back({ static_cast<std::int8_t>(dx), static_cast<std::int8_t>(dy) });
        std::stable_sort(order_.begin(), order_.end(), [](Offset a, Offset b) {
            const int ra = std::max(std::abs(a.dx), std::abs(a.dy));
            const int rb = std::max(std::abs(b.dx), std::abs(b.dy));
            if (ra != rb) return ra < rb;
            return a.dx * a.dx + a.dy * a.dy < b.dx * b.dx + b.dy * b.dy;
        });
        ring_begin_.assign(static_cast<std::size_t>(reach_) + 2, 0);
        for (const Offset& o : order_)
            ++ring_begin_[static_cast<std::size_t>(std::max(std::abs(o.dx), std::abs(o.dy))) + 1];
        for (std::size_t r = 1; r < ring_begin_.size(); ++r)
            ring_begin_[r] += ring_begin_[r - 1];
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
        //    + kopija pozicija u istom redoslijedu (uzastopno citanje u k_nearest)
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::int32_t p = cursor_[cell_of_[i]]++;
            sorted_ids_[p] = static_cast<std::int32_t>(i);
            pos_of_[i] = p;
            sx_[p] = x[i];
            sy_[p] = y[i];
        }
    }
}
