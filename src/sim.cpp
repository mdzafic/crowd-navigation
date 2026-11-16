
#include "sim.hpp"
#include <cmath>
#include <utility>
#include <chrono>

#ifdef CROWD_DIAG
namespace crowd
{
    unsigned long long g_cand = 0;
    unsigned long long g_inradius = 0;
    unsigned long long g_inserted = 0;
}
#endif

static int subdiv_from_env() {
    const char* s = std::getenv("CROWD_SUBDIV");
    return s ? std::max(1, std::atoi(s)) : 2;
}

namespace crowd
{
    Sim::Sim(Agents initial, World world, const Params& p)
        : p_(p), cur_(std::move(initial)),
        grid_(world, p.interaction_radius(), cur_.size(), 3),
        neighbor_ids_(cur_.size()* p.max_neighbors, -1),
        neighbor_distances_(cur_.size()* p.max_neighbors, 0.f),
        neighbor_counts_(cur_.size(), 0),
        orca_line_buffers_(cur_.size()* p.max_neighbors * 2)
    {
        next_ = cur_;
    }

    Sim::Sim(Agents initial, World world, const Params& p, const Map* map)
        : Sim(std::move(initial), world, p)
    {
        if (!map) return;
        map_ = map;
        goal_x_ = cur_.gx;
        goal_y_ = cur_.gy;
        agent_field_ = nav_.build(*map_, goal_x_, goal_y_, p_.radius);
        orca_line_buffers_.resize(cur_.size() *
            static_cast<std::size_t>(lineBufferSize(
                static_cast<int>(p_.max_neighbors + map_->segments.size()))));
        wall_lines_.resize(map_->segments.size());
    }

    static bool insert_neighbor(std::int32_t id, float distance2,
        std::int32_t* ids, float* distances,
        std::size_t& count, std::size_t limit)
    {
        if (limit == 0 || (count == limit && distance2 >= distances[count - 1]))
            return false;

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

        return true;
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

        const std::size_t line_buffer_stride = static_cast<std::size_t>(
            lineBufferSize(static_cast<int>(p_.max_neighbors +
                (map_ ? map_->segments.size() : 0))));

        const auto t0 = clk::now();
        grid_.build(cur_.x.data(), cur_.y.data(), n);
        const auto t1 = clk::now();


        // Lokalni baferi kandidata (alociraju se jednom po koraku, ne po agentu).
// Ne smiju biti static/thread_local ako kasnije paraleliziramo.
        std::vector<std::int32_t> cand_id(512);
        std::vector<float> cand_d2(512);
        const float rad = p_.interaction_radius();
        const float r2 = rad * rad;
        const float* px = cur_.x.data();
        const float* py = cur_.y.data();

        for (std::size_t i = 0; i < n; ++i)
        {
            std::int32_t* ids = neighbor_ids_.data() + i * p_.max_neighbors;
            float* distances = neighbor_distances_.data() + i * p_.max_neighbors;
            const float xi = px[i], yi = py[i];
            std::size_t count = 0;
            std::size_t m = 0;

#ifdef CROWD_DIAG
            grid_.for_each_candidate(xi, yi, [&](std::int32_t) { ++g_cand; });
#endif

            // 1) Kandidati: d2 racunamo za sve, upisujemo uvijek,
            //    a brojac m povecavamo maskom (bez grananja).
            grid_.for_each_candidate(xi, yi, [&](std::int32_t j) {
                if (m + 1 >= cand_id.size())          // rijetko i predvidivo
                {
                    cand_id.resize(cand_id.size() * 2);
                    cand_d2.resize(cand_d2.size() * 2);
                }
                const float dx = px[j] - xi, dy = py[j] - yi;
                const float d2 = dx * dx + dy * dy;
                cand_id[m] = j;
                cand_d2[m] = d2;
                m += static_cast<std::size_t>(
                    (d2 <= r2) & (static_cast<std::size_t>(j) != i));
                });

            // 2) Umetanje u sortirani top-k (isti redoslijed kao prije).
            for (std::size_t c = 0; c < m; ++c)
            {
                [[maybe_unused]] const bool ins = insert_neighbor(
                    cand_id[c], cand_d2[c],
                    ids, distances, count, p_.max_neighbors);
#ifdef CROWD_DIAG
                ++g_inradius;
                g_inserted += ins;
#endif
            }

            neighbor_counts_[i] = count;
        }

        const auto t2 = clk::now();

        // Faza 3: ORCA + pomak
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t count = neighbor_counts_[i];
            const std::int32_t* ids = neighbor_ids_.data() + i * p_.max_neighbors;
            int wallCount = 0;

            if (map_)
            {
                const float cell = map_->cell;
                const int cx = static_cast<int>(std::floor(cur_.x[i] / cell));
                const int cy = static_cast<int>(std::floor(cur_.y[i] / cell));

                const NavField& f = nav_.at(agent_field_[i]);
                const float look = 3.f * vmax * dt;

                float ox, oy;

                if (cy * map_->w + cx == f.goal_cell ||
                    !f.sample(cur_.x[i], cur_.y[i], ox, oy))
                {
                    cur_.gx[i] = goal_x_[i];
                    cur_.gy[i] = goal_y_[i];
                }
                else
                {
                    cur_.gx[i] = cur_.x[i] + ox * look;
                    cur_.gy[i] = cur_.y[i] + oy * look;
                }

                const Vec2 position{ cur_.x[i], cur_.y[i] };
                const Vec2 velocity{ cur_.vx[i], cur_.vy[i] };

                const float wallRange = p_.radius + p_.v_max * p_.horizon_T;
                const float wallRangeSq = wallRange * wallRange;

                for (const Segment& segment : map_->segments)
                {
                    const Vec2 edge = segment.b - segment.a;
                    const float edgeLengthSq = lengthSq(edge);

                    float t = 0.0f;

                    if (edgeLengthSq > 1.0e-12f)
                        t = std::clamp(
                            dot(position - segment.a, edge) / edgeLengthSq,
                            0.0f, 1.0f);

                    const Vec2 closest = segment.a + edge * t;

                    if (lengthSq(position - closest) <= wallRangeSq)
                    {
                        wall_lines_[static_cast<std::size_t>(wallCount++)] =
                            makeWallOrcaLine(
                                position, velocity, p_.radius,
                                segment.a, segment.b,
                                p_.horizon_T, p_.dt);
                    }
                }
            }

            float vx, vy;

            if (p_.enable_orca)
            {
                Line* line_buffer = (count == 0 && wallCount == 0) ? nullptr :
                    orca_line_buffers_.data() + i * line_buffer_stride;

                const Vec2 velocity = computeNewVelocity(
                    static_cast<int>(i), cur_, ids,
                    static_cast<int>(count),
                    wall_lines_.data(), wallCount,
                    line_buffer, p_);

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

            if (map_)
            {
                const float wr = 0.9f * p_.radius;
                const float ox = cur_.x[i], oy = cur_.y[i];

                float nx = ox, ny = oy;

                if (!map_->blocked(next_.x[i], oy, wr))
                    nx = next_.x[i];

                if (!map_->blocked(nx, next_.y[i], wr))
                    ny = next_.y[i];

                next_.x[i] = nx;
                next_.y[i] = ny;

                vx = (nx - ox) / dt;
                vy = (ny - oy) / dt;
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
            if (cur_.x[i] == cur_.gx[i] &&
                cur_.y[i] == cur_.gy[i])
                ++c;

        return c;
    }
}
