#include <cstdio>
#include <cstdlib>
#include <string>
#include "map.hpp"

using namespace crowd;

#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); std::exit(1); } } while (0)

static Map mustParse(const std::string& text)
{
    Map m; std::string err;
    if (!Map::parse(text, m, err)) { std::printf("parse: %s\n", err.c_str()); std::exit(1); }
    return m;
}

int main()
{
    // soba 3x3 slobodnih æelija u zidu
    const Map room = mustParse(
        "#####\n"
        "#S..#\n"
        "#...#\n"
        "#..G#\n"
        "#####\n");

    CHECK(room.w == 5 && room.h == 5);
    CHECK(room.isWall(0, 0));
    CHECK(!room.isWall(1, 1));
    CHECK(room.isWall(-1, 2) && room.isWall(2, 99));        // izvan mape = zid
    CHECK(room.blocked(0.5f, 0.5f));                        // unutar zida
    CHECK(!room.blocked(2.5f, 2.5f));                       // slobodno
    CHECK(room.blocked(-0.1f, 2.0f));                       // izvan mape
    CHECK(room.segments.size() == 12);                      // 4 strane x 3 ivice

    // orijentacija: taèka malo pomaknuta od sredine segmenta u smjeru
    // slobodne strane mora biti slobodna, a u suprotnom smjeru zid
    for (const Segment& s : room.segments)
    {
        const Vec2 d = s.b - s.a;
        const Vec2 mid = (s.a + s.b) * 0.5f;
        // lijeva normala u smislu det(d, p - a) > 0
        const Vec2 n{-d.y, d.x};
        const float eps = 0.1f;
        CHECK(!room.blocked(mid.x + n.x * eps, mid.y + n.y * eps));
        CHECK(room.blocked(mid.x - n.x * eps, mid.y - n.y * eps));
    }

    // neispravne mape
    Map m; std::string err;
    CHECK(!Map::parse("###\n##\n", m, err));                // nejednake dužine
    CHECK(!Map::parse("#S#\n#X#\n#G#\n", m, err));          // nepoznat znak
    CHECK(!Map::parse("###\n#.#\n###\n", m, err));          // nema S i G
    CHECK(!Map::parse("", m, err));

    std::printf("test_map OK\n");
    return 0;
}