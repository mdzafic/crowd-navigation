#include "sim.hpp"
#include <cmath>
#include <utility>

namespace crowd
{
    Sim::Sim(Agents initial, World world, const Params& p)
        : p_(p), cur_(std::move(initial)),
          grid_(world, p.interaction_radius(), cur_.size()),
          neighbor_ids_(cur_.size() * p.max_neighbors, -1),
          neighbor_distances_(cur_.size() * p.max_neighbors, 0.f),
          neighbor_counts_(cur_.size(), 0),
          orca_line_buffers_(cur_.size() * p.max_neighbors * 2)
    {
        next_ = cur_;
    }

    static void insert_neighbor(std::int32_t id, float distance2,
                                std::int32_t* ids, float* distances,
                                std::size_t& count, std::size_t limit)
    {
        if (limit == 0 || (count == limit && distance2 >= distances[count - 1]))
            return;

        const std::size_t new_count = count < limit ? count + 1 : count;
        std::size_t pos = new_count - 1;
        while (pos > 0 && distance2 < distances[pos - 1])
        {
            distances[pos] = distances[pos - 1];
            ids[pos] = ids[pos - 1];
            --pos;
        }
        distances[pos] = distance2;
        ids[pos] = id;
        count = new_count;
    }

    void Sim::step()
    {
        const std::size_t n = cur_.size();
        const float dt = p_.dt;
        const float vmax = p_.v_max;
        const std::size_t line_buffer_stride = p_.max_neighbors * 2;

        grid_.build(cur_.x.data(), cur_.y.data(), n);
        for (std::size_t i = 0; i < n; ++i)
        {
            std::int32_t* ids = neighbor_ids_.data() + i * p_.max_neighbors;
            float* distances = neighbor_distances_.data() + i * p_.max_neighbors;
            std::size_t count = 0;
            grid_.for_each_neighbor(i, cur_.x.data(), cur_.y.data(), p_.interaction_radius(),
                                    [&](std::int32_t j) {
                                        const float dx = cur_.x[j] - cur_.x[i];
                                        const float dy = cur_.y[j] - cur_.y[i];
                                        insert_neighbor(j, dx * dx + dy * dy,
                                                        ids, distances, count,
                                                        p_.max_neighbors);
                                    });
            neighbor_counts_[i] = count;

            float vx, vy;
            if (p_.enable_orca)
            {
                Line* line_buffer = count == 0 ? nullptr :
                    orca_line_buffers_.data() + i * line_buffer_stride;
                const Vec2 velocity = computeNewVelocity(
                    static_cast<int>(i), cur_, ids, static_cast<int>(count), line_buffer, p_);
                vx = velocity.x;
                vy = velocity.y;
                next_.x[i] = cur_.x[i] + vx * dt;
                next_.y[i] = cur_.y[i] + vy * dt;
            }
            else
            {
                const float dx = cur_.gx[i] - cur_.x[i];
                const float dy = cur_.gy[i] - cur_.y[i];
                const float distance2 = dx * dx + dy * dy;
                const float stepLength = vmax * dt;
                vx = 0.f;
                vy = 0.f;
                if (distance2 > stepLength * stepLength)
                {
                    const float inverseDistance = vmax / std::sqrt(distance2);
                    vx = dx * inverseDistance;
                    vy = dy * inverseDistance;
                    next_.x[i] = cur_.x[i] + vx * dt;
                    next_.y[i] = cur_.y[i] + vy * dt;
                }
                else
                {
                    vx = 0.f;
                    vy = 0.f;
                    next_.x[i] = cur_.gx[i];
                    next_.y[i] = cur_.gy[i];
                }
            }
            next_.vx[i] = vx;
            next_.vy[i] = vy;
            next_.gx[i] = cur_.gx[i];
            next_.gy[i] = cur_.gy[i];
        }
        std::swap(cur_, next_);   // vektori se zamijene u O(1)
        ++steps_;
    }

    std::size_t Sim::reached() const noexcept
    {
        std::size_t c = 0;
        for (std::size_t i = 0; i < cur_.size(); ++i)
            if (cur_.x[i] == cur_.gx[i] && cur_.y[i] == cur_.gy[i]) ++c;
        return c;
    }
}