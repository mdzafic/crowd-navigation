#include <cmath>
#include <cstdio>
#include "map.hpp"
#include "orca.hpp"
using namespace crowd;

int main()
{
    Params p;
    Map m; std::string err;
    if (!Map::load("maps/slovo_l.txt", m, err)) { std::printf("%s\n", err.c_str()); return 1; }

    // red 10 je prvi pun hodnik; iznad (red 9, x >= 5) je zid
    Agents a; a.resize(1);
    a.x[0] = 25.5f; a.y[0] = 26.5f;     // 0.5 ispod lica zida (zid ima y 25..26)
    a.gx[0] = 35.5f; a.gy[0] = 20.0f;   // cilj dijagonalno "kroz" zid

    Line buf[2 * kMaxWallLines];
    const Vec2 v = computeNewVelocity(0, a, nullptr, 0, buf, p, &m);

    const float limit = -(0.5f - kWallRadiusFactor * p.radius) / p.horizon_T;
    bool ok = v.y >= limit - 1e-4f;      // ne ulazi u zid
    ok = ok && v.x > 0.5f;               // klizi uz zid
    std::printf("v = (%.3f, %.3f), limit y >= %.3f -> %s\n", v.x, v.y, limit, ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}