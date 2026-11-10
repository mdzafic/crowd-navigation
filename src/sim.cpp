#include "sim.hpp"
#include <cmath>
#include <utility>
#include <chrono>

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
        line_stride_ = p.max_neighbors * 2;
    }
    Sim::Sim(Agents initial, World world, const Params& p, const Map* map)
    : Sim(std::move(initial), world, p)
    {
        if (!map) return;
        map_ = map;
        line_stride_ = 2 * (static_cast<std::size_t>(kMaxWallLines) + p_.max_neighbors);
        orca_line_buffers_.assign(cur_.size() * line_stride_, Line{});
        goal_x_ = cur_.gx;
        goal_y_ = cur_.gy;
        agent_field_ = nav_.build(*map_, goal_x_, goal_y_, p_.radius);
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

    void Sim::step(StepTimes* times)
    {
        using clk = std::chrono::steady_clock;
        auto ms = [](clk::time_point a, clk::time_point b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
            };

        const std::size_t n = cur_.size();
        const float dt = p_.dt;
        const float vmax = p_.v_max;
        const std::size_t line_buffer_stride = line_stride_;

        const auto t0 = clk::now();
        grid_.build(cur_.x.data(), cur_.y.data(), n);
        const auto t1 = clk::now();

        // Faza 2: top-k susjedi
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
        }
        const auto t2 = clk::now();

        // Faza 3: ORCA + pomak
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t count = neighbor_counts_[i];
            const std::int32_t* ids = neighbor_ids_.data() + i * p_.max_neighbors;

            if (map_)
            {
                const float cell = map_->cell;
                const int cx = static_cast<int>(std::floor(cur_.x[i] / cell));
                const int cy = static_cast<int>(std::floor(cur_.y[i] / cell));
                const NavField& f = nav_.at(agent_field_[i]);
                const float look = 3.f * vmax * dt;
                float ox, oy;
                if (cy * map_->w + cx == f.goal_cell || !f.sample(cur_.x[i], cur_.y[i], ox, oy))
                {
                    cur_.gx[i] = goal_x_[i];                 // u ciljnoj æeliji: ravno na cilj
                    cur_.gy[i] = goal_y_[i];
                }
                else
                {
                    cur_.gx[i] = cur_.x[i] + ox * look;      // waypoint iz polja
                    cur_.gy[i] = cur_.y[i] + oy * look;
                }
            }

            float vx, vy;
            if (p_.enable_orca)
            {
                Line* line_buffer = (map_ || count > 0)
                    ? orca_line_buffers_.data() + i * line_buffer_stride : nullptr;
                const Vec2 velocity = computeNewVelocity(
                        static_cast<int>(i), cur_, ids, static_cast<int>(count), line_buffer, p_, map_);
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
                    next_.x[i] = cur_.gx[i];
                    next_.y[i] = cur_.gy[i];
                }
            }
            if (map_)   // klizanje: odbaci komponentu pomaka koja ulazi u zid
            {
                const float wr = kWallRadiusFactor * p_.radius;
                const float ox = cur_.x[i], oy = cur_.y[i];
                float nx = ox, ny = oy;
                if (!map_->blocked(next_.x[i], oy, wr)) nx = next_.x[i];
                if (!map_->blocked(nx, next_.y[i], wr)) ny = next_.y[i];
                next_.x[i] = nx; next_.y[i] = ny;
                vx = (nx - ox) / dt;  vy = (ny - oy) / dt;   // stvarna brzina za susjede
            }
            next_.vx[i] = vx;
            next_.vy[i] = vy;
            next_.gx[i] = map_ ? goal_x_[i] : cur_.gx[i];
            next_.gy[i] = map_ ? goal_y_[i] : cur_.gy[i];
        }
        const auto t3 = clk::now();

        std::swap(cur_, next_);
        ++steps_;

        if (times)
        {
            times->grid_ms = ms(t0, t1);
            times->topk_ms = ms(t1, t2);
            times->orca_ms = ms(t2, t3);
        }
    }

    std::size_t Sim::reached() const noexcept
    {
        std::size_t c = 0;
        for (std::size_t i = 0; i < cur_.size(); ++i)
            if (cur_.x[i] == cur_.gx[i] && cur_.y[i] == cur_.gy[i]) ++c;
        return c;
    }
}