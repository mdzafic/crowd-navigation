#include "map.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace crowd
{
    bool Map::isWall(int cx, int cy) const noexcept
    {
        if (cx < 0 || cy < 0 || cx >= w || cy >= h)
            return true;
        return wall[static_cast<std::size_t>(cy) * w + cx] != 0;
    }

    bool Map::blocked(float x, float y) const noexcept
    {
        return isWall(static_cast<int>(std::floor(x / cell)),
                      static_cast<int>(std::floor(y / cell)));
    }
    bool Map::blocked(float x, float y, float r) const noexcept
    {
        const int x0 = static_cast<int>(std::floor((x - r) / cell));
        const int x1 = static_cast<int>(std::floor((x + r) / cell));
        const int y0 = static_cast<int>(std::floor((y - r) / cell));
        const int y1 = static_cast<int>(std::floor((y + r) / cell));
        for (int cy = y0; cy <= y1; ++cy)
            for (int cx = x0; cx <= x1; ++cx)
            {
                if (!isWall(cx, cy)) continue;
                const float qx = std::fmin(std::fmax(x, cx * cell), (cx + 1) * cell);
                const float qy = std::fmin(std::fmax(y, cy * cell), (cy + 1) * cell);
                const float dx = x - qx, dy = y - qy;
                if (dx * dx + dy * dy < r * r) return true;
            }
        return false;
    }

    namespace
    {
        void buildSegments(Map& m)
        {
            // TODO: spajanje kolinearnih segmenata (smanjuje broj za 5-10x).
            m.segments.clear();
            const float c = m.cell;
            for (int cy = 0; cy < m.h; ++cy)
                for (int cx = 0; cx < m.w; ++cx)
                {
                    if (!m.isWall(cx, cy)) continue;
                    const float x0 = cx * c, x1 = (cx + 1) * c;
                    const float y0 = cy * c, y1 = (cy + 1) * c;

                    // slobodan susjed = unutar mape i nije zid
                    const auto freeCell = [&](int x, int y) {
                        return x >= 0 && y >= 0 && x < m.w && y < m.h && !m.isWall(x, y);
                    };
                    // orijentacija: det(b-a, p-a) > 0 za slobodnu stranu
                    if (freeCell(cx, cy - 1)) m.segments.push_back({{x1, y0}, {x0, y0}}); // gore
                    if (freeCell(cx, cy + 1)) m.segments.push_back({{x0, y1}, {x1, y1}}); // dolje
                    if (freeCell(cx - 1, cy)) m.segments.push_back({{x0, y0}, {x0, y1}}); // lijevo
                    if (freeCell(cx + 1, cy)) m.segments.push_back({{x1, y1}, {x1, y0}}); // desno
                }
        }
    }

    bool Map::parse(const std::string& text, Map& out, std::string& err, float cell)
    {
        std::vector<std::string> rows;
        std::istringstream in(text);
        std::string line;
        while (std::getline(in, line))
        {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            rows.push_back(line);
        }
        while (!rows.empty() && rows.back().empty()) rows.pop_back();

        if (rows.empty()) { err = "mapa je prazna"; return false; }
        const std::size_t width = rows[0].size();
        if (width == 0) { err = "prvi red je prazan"; return false; }

        Map m;
        m.w = static_cast<int>(width);
        m.h = static_cast<int>(rows.size());
        m.cell = cell;
        const std::size_t n = width * rows.size();
        m.wall.assign(n, 0);
        m.spawn.assign(n, 0);
        m.goal.assign(n, 0);

        std::size_t spawnCount = 0, goalCount = 0;
        for (std::size_t y = 0; y < rows.size(); ++y)
        {
            if (rows[y].size() != width)
            {
                err = "red " + std::to_string(y + 1) + " ima drugaèiju dužinu";
                return false;
            }
            for (std::size_t x = 0; x < width; ++x)
            {
                const std::size_t i = y * width + x;
                switch (rows[y][x])
                {
                    case '#': m.wall[i] = 1; break;
                    case '.': break;
                    case 'S': m.spawn[i] = 1; ++spawnCount; break;
                    case 'G': m.goal[i] = 1; ++goalCount; break;
                    default:
                        err = std::string("nepoznat znak '") + rows[y][x] + "' u redu " +
                              std::to_string(y + 1) + ", koloni " + std::to_string(x + 1);
                        return false;
                }
            }
        }
        if (spawnCount == 0) { err = "mapa nema S æelija"; return false; }
        if (goalCount == 0)  { err = "mapa nema G æelija"; return false; }

        buildSegments(m);
        out = std::move(m);
        return true;
    }

    bool Map::load(const std::string& path, Map& out, std::string& err, float cell)
    {
        std::ifstream f(path);
        if (!f) { err = "ne mogu otvoriti " + path; return false; }
        std::stringstream ss;
        ss << f.rdbuf();
        return parse(ss.str(), out, err, cell);
    }
}