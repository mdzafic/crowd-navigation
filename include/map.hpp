#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "orca.hpp"   // Vec2

namespace crowd
{
    // Koordinate: x udesno, y PREMA DOLJE (red 0 u fajlu je gore).
    // Æelija (cx, cy) zauzima [cx*cell, (cx+1)*cell] x [cy*cell, (cy+1)*cell].
    //
    // Segment a->b: slobodan prostor je uvijek na strani gdje je
    // det(b - a, p - a) > 0 (isti det kao u orca.cpp).
    struct Segment
    {
        Vec2 a, b;
    };

    struct Map
    {
        int w = 0, h = 0;
        float cell = 1.0f;
        std::vector<std::uint8_t> wall;     // w*h, 1 = zid
        std::vector<std::uint8_t> spawn;    // w*h, 1 = S
        std::vector<std::uint8_t> goal;     // w*h, 1 = G
        std::vector<Segment> segments;      // granice zidova

        bool isWall(int cx, int cy) const noexcept;     // izvan mape = zid
        bool blocked(float x, float y) const noexcept;  // koordinate -> æelija

        float widthWorld()  const noexcept { return static_cast<float>(w) * cell; }
        float heightWorld() const noexcept { return static_cast<float>(h) * cell; }

        // Vraæa false i popunjava err ako je mapa neispravna.
        static bool parse(const std::string& text, Map& out, std::string& err,
                          float cell = 1.0f);
        static bool load(const std::string& path, Map& out, std::string& err,
                         float cell = 1.0f);
    };
}