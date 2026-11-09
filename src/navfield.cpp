#include "navfield.hpp"
#include <cmath>
#include <queue>
#include <utility>

namespace crowd
{
    namespace
    {
        constexpr float INF = 1e30f;
        const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        // k >= 4 je dijagonala: zabrani rezanje kroz ugao zida
        bool move_ok(const Map& m, int x, int y, int k)
        {
            const int nx = x + DX[k], ny = y + DY[k];
            if (m.isWall(nx, ny)) return false;
            if (k >= 4 && (m.isWall(nx, y) || m.isWall(x, ny))) return false;
            return true;
        }

        // Udaljenost (u svjetskim jedinicama) od centra æelije do najbliže æelije zida.
        std::vector<float> wall_distance(const Map& m)
        {
            const int W = m.w, H = m.h;
            std::vector<int> d(static_cast<std::size_t>(W) * H, -1);
            std::queue<int> q;
            for (int i = 0; i < W * H; ++i)
                if (m.wall[i]) { d[i] = 0; q.push(i); }
            while (!q.empty())
            {
                const int i = q.front(); q.pop();
                const int x = i % W, y = i / W;
                for (int k = 0; k < 8; ++k)
                {
                    const int nx = x + DX[k], ny = y + DY[k];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    const int j = ny * W + nx;
                    if (d[j] >= 0) continue;
                    d[j] = d[i] + 1;
                    q.push(j);
                }
            }
            std::vector<float> out(d.size());
            for (std::size_t i = 0; i < d.size(); ++i)
                out[i] = static_cast<float>(d[i]) * m.cell;
            return out;
        }
    }

    void NavField::build(const Map& m, int gcx, int gcy,
                         const std::vector<float>& wd, float margin)
    {
        w = m.w; h = m.h; cell = m.cell;
        goal_cell = gcy * w + gcx;
        const std::size_t n = static_cast<std::size_t>(w) * h;
        cost.assign(n, INF);
        dirx.assign(n, 0.f);
        diry.assign(n, 0.f);

        using QE = std::pair<float, int>;
        std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
        cost[goal_cell] = 0.f;
        pq.push({0.f, goal_cell});

        // penalizacija æelija blizu zida (udaljenost do LICA zida)
        auto pen = [&](int idx) {
            const float d = wd[idx] - 0.5f * cell;
            return d >= margin ? 1.f : 1.f + 2.f * (1.f - d / margin);
        };

        while (!pq.empty())
        {
            const auto [c, i] = pq.top(); pq.pop();
            if (c > cost[i]) continue;
            const int x = i % w, y = i / w;
            for (int k = 0; k < 8; ++k)
            {
                if (!move_ok(m, x, y, k)) continue;
                const int j = (y + DY[k]) * w + (x + DX[k]);
                const float step = (k < 4 ? 1.f : 1.41421356f) * cell;
                const float nc = c + step * pen(j);
                if (nc < cost[j]) { cost[j] = nc; pq.push({nc, j}); }
            }
        }

        // smjer = ponderisan zbir jediniènih vektora prema jeftinijim susjedima
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
            {
                const int i = y * w + x;
                if (cost[i] >= INF || i == goal_cell) continue;
                float sx = 0.f, sy = 0.f;
                for (int k = 0; k < 8; ++k)
                {
                    if (!move_ok(m, x, y, k)) continue;
                    const int j = (y + DY[k]) * w + (x + DX[k]);
                    const float gain = cost[i] - cost[j];
                    if (gain <= 0.f) continue;
                    const float len = (k < 4) ? 1.f : 1.41421356f;
                    sx += gain * DX[k] / len;
                    sy += gain * DY[k] / len;
                }
                const float l = std::sqrt(sx * sx + sy * sy);
                if (l > 1e-6f) { dirx[i] = sx / l; diry[i] = sy / l; }
            }
    }

    bool NavField::sample(float x, float y, float& ox, float& oy) const
    {
        const float fx = x / cell - 0.5f, fy = y / cell - 0.5f;
        const int x0 = static_cast<int>(std::floor(fx));
        const int y0 = static_cast<int>(std::floor(fy));
        const float tx = fx - x0, ty = fy - y0;
        float sx = 0.f, sy = 0.f, ws = 0.f;
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i)
            {
                const int cx = x0 + i, cy = y0 + j;
                if (cx < 0 || cy < 0 || cx >= w || cy >= h) continue;
                const int idx = cy * w + cx;
                if (cost[idx] >= INF) continue;          // zid / nedostižno
                const float wt = (i ? tx : 1.f - tx) * (j ? ty : 1.f - ty);
                sx += dirx[idx] * wt; sy += diry[idx] * wt; ws += wt;
            }
        const float l = std::sqrt(sx * sx + sy * sy);
        if (ws <= 0.f || l < 1e-6f) return false;
        ox = sx / l; oy = sy / l;
        return true;
    }

    std::vector<int> NavFields::build(const Map& m, const std::vector<float>& gx,
                                      const std::vector<float>& gy, float radius)
    {
        fields_.clear();
        const std::vector<float> wd = wall_distance(m);
        std::vector<int> cell_to_field(static_cast<std::size_t>(m.w) * m.h, -1);
        std::vector<int> per_agent(gx.size(), -1);
        for (std::size_t a = 0; a < gx.size(); ++a)
        {
            const int cx = static_cast<int>(std::floor(gx[a] / m.cell));
            const int cy = static_cast<int>(std::floor(gy[a] / m.cell));
            const int c = cy * m.w + cx;
            if (cell_to_field[c] < 0)
            {
                cell_to_field[c] = static_cast<int>(fields_.size());
                fields_.emplace_back();
                fields_.back().build(m, cx, cy, wd, 3.f * radius);
            }
            per_agent[a] = cell_to_field[c];
        }
        return per_agent;
    }
}