// tests/test_validation.cpp
#include "orca.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace crowd;

static int g_failures = 0;
#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

// ---------------------------------------------------------------- helpers

static Params makeParams()
{
    return Params{}; // v_max 1.4, radius 0.5, horizon_T 2.0, dt 0.25
}

static Agents makeAgents(const std::vector<Vec2>& pos, const std::vector<Vec2>& goal)
{
    Agents a;
    a.resize(pos.size()); // nulira brzine
    for (std::size_t i = 0; i < pos.size(); ++i)
    {
        a.x[i]  = pos[i].x;  a.y[i]  = pos[i].y;
        a.gx[i] = goal[i].x; a.gy[i] = goal[i].y;
    }
    return a;
}

// Jedan korak: cita iz cur, pise u next, pa zamijeni.
// Mini-simulacija: svaki agent vidi sve ostale (bez top-k).
static void step(Agents& cur, Agents& next, const Params& p, std::size_t n)
{
    std::vector<int>  neighbors;
    std::vector<Line> buffer(static_cast<std::size_t>(lineBufferSize(static_cast<int>(n))));

    next = cur; // prenosi ciljeve
    for (std::size_t i = 0; i < n; ++i)
    {
        neighbors.clear();
        for (std::size_t j = 0; j < n; ++j)
            if (j != i) neighbors.push_back(static_cast<int>(j));

        const Vec2 v = computeNewVelocity(static_cast<int>(i), cur, neighbors.data(),
                                          static_cast<int>(neighbors.size()),
                                          buffer.data(), p);
        next.vx[i] = v.x;
        next.vy[i] = v.y;
        next.x[i]  = cur.x[i] + v.x * p.dt;
        next.y[i]  = cur.y[i] + v.y * p.dt;
    }
    std::swap(cur, next);
}

static float pairDistance(const Agents& a, std::size_t i, std::size_t j)
{
    return std::sqrt(lengthSq(Vec2{a.x[i] - a.x[j], a.y[i] - a.y[j]}));
}

static float minPairDistance(const Agents& a, std::size_t n)
{
    float best = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j)
            best = std::min(best, pairDistance(a, i, j));
    return best;
}

static float goalDistance(const Agents& a, std::size_t i)
{
    return std::sqrt(lengthSq(Vec2{a.gx[i] - a.x[i], a.gy[i] - a.y[i]}));
}

// ------------------------------------------------- Test 0: makeOrcaLine

static void testLineProperties()
{
    std::printf("Test 0: svojstva makeOrcaLine\n");
    const float r = 0.5f, T = 5.0f, dt = 0.1f;

    // Reciprocitet: linije za A i B moraju biti zrcalne.
    const Vec2 pA{0, 0}, vA{1.0f, 0.2f}, pB{4, 0.7f}, vB{-1.0f, 0.1f};
    const Line la = makeOrcaLine(pA, vA, r, pB, vB, r, T, dt);
    const Line lb = makeOrcaLine(pB, vB, r, pA, vA, r, T, dt);

    CHECK_NEAR((la.point - vA).x, -(lb.point - vB).x, 1e-5f);
    CHECK_NEAR((la.point - vA).y, -(lb.point - vB).y, 1e-5f);
    CHECK_NEAR(la.direction.x, -lb.direction.x, 1e-5f);
    CHECK_NEAR(la.direction.y, -lb.direction.y, 1e-5f);

    // Direction mora biti jedinicni vektor.
    CHECK_NEAR(lengthSq(la.direction), 1.0f, 1e-4f);

    // Kolizijski slucaj (preklapanje).
    const Line lc = makeOrcaLine({0, 0}, {0, 0}, r, {0.5f, 0}, {0, 0}, r, T, dt);
    CHECK_NEAR(lengthSq(lc.direction), 1.0f, 1e-4f);
    CHECK(std::isfinite(lc.point.x) && std::isfinite(lc.point.y));

    // Identicne pozicije ne smiju proizvesti NaN.
    const Line ld = makeOrcaLine({1, 1}, {0, 0}, r, {1, 1}, {0, 0}, r, T, dt);
    CHECK(std::isfinite(ld.point.x) && std::isfinite(ld.direction.x));

    // Neispravni argumenti.
    bool threw = false;
    try { makeOrcaLine(pA, vA, r, pB, vB, r, 0.0f, dt); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { makeOrcaLine(pA, vA, r, pB, vB, r, T, -1.0f); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}

// ------------------------------------------------ Test 1: licem u lice

static void runHeadOn(float yOffset)
{
    std::printf("Test 1: dva agenta licem u lice (yOffset = %g)\n", yOffset);
    const Params p = makeParams();
    Agents cur  = makeAgents({{-5, 0}, {5, yOffset}}, {{5, 0}, {-5, yOffset}});
    Agents next = cur;
    cur.vx[0] =  p.v_max;
    cur.vx[1] = -p.v_max;

    float minDist = std::numeric_limits<float>::max();
    float maxAbsY = 0.0f;
    for (int s = 0; s < 400; ++s)
    {
        step(cur, next, p, 2);
        minDist = std::min(minDist, minPairDistance(cur, 2));
        maxAbsY = std::max({maxAbsY, std::fabs(cur.y[0]), std::fabs(cur.y[1])});
        const float speed0 = std::sqrt(lengthSq(Vec2{cur.vx[0], cur.vy[0]}));
        const float speed1 = std::sqrt(lengthSq(Vec2{cur.vx[1], cur.vy[1]}));
        CHECK(speed0 <= p.v_max * 1.0001f);
        CHECK(speed1 <= p.v_max * 1.0001f);
    }

    std::printf("  najmanji razmak: %.3f (2r = %.3f), max |y|: %.3f\n",
                minDist, 2.0f * p.radius, maxAbsY);
    CHECK(minDist >= 2.0f * p.radius - 0.01f); // nema preklapanja
    CHECK(maxAbsY > 0.1f);                     // stvarno su se izmakli
    CHECK(cur.x[0] > cur.x[1]);                // zamijenili su strane
    CHECK(goalDistance(cur, 0) < 0.1f);
    CHECK(goalDistance(cur, 1) < 0.1f);
}

// ------------------------------------------- Test 2: antipodalna guzva

static Agents makeAntipodal(std::size_t n, float R)
{
    std::vector<Vec2> pos(n), goal(n);
    const float twoPi = 6.28318530718f;
    for (std::size_t i = 0; i < n; ++i)
    {
        const float a = twoPi * static_cast<float>(i) / static_cast<float>(n);
        pos[i]  = {R * std::cos(a),  R * std::sin(a)};
        goal[i] = {-pos[i].x, -pos[i].y};
    }
    return makeAgents(pos, goal);
}

static void testAntipodal()
{
    std::printf("Test 2: antipodalna guzva\n");
    const Params p = makeParams();
    const std::size_t n = 16;
    Agents cur  = makeAntipodal(n, 8.0f);
    Agents next = cur;

    const float tol = 0.15f * 2.0f * p.radius; // dopustena "penetracija" (15 %)
    int overlapSteps = 0;
    float worst = std::numeric_limits<float>::max();

    for (int s = 0; s < 800; ++s)
    {
        step(cur, next, p, n);
        const float m = minPairDistance(cur, n);
        worst = std::min(worst, m);
        if (m < 2.0f * p.radius - tol) ++overlapSteps;
    }

    std::printf("  najmanji razmak: %.3f (2r = %.3f), koraka s preklapanjem: %d\n",
                worst, 2.0f * p.radius, overlapSteps);
    CHECK(overlapSteps == 0);

    int reached = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (goalDistance(cur, i) < p.radius) ++reached;
    std::printf("  stiglo do cilja: %d / %zu\n", reached, n);
    CHECK(reached == static_cast<int>(n));
}

// ---------------------------------------------------- Test 3: jedan agent

static void testSingleAgent()
{
    std::printf("Test 3: jedan agent\n");
    const Params p = makeParams();

    // Bez susjeda: direktno prema cilju, brzina v_max.
    {
        Agents a = makeAgents({{0, 0}}, {{10, 0}});
        const Vec2 v = computeNewVelocity(0, a, nullptr, 0, nullptr, p);
        CHECK_NEAR(v.x, p.v_max, 1e-5f);
        CHECK_NEAR(v.y, 0.0f, 1e-5f);
    }
    // Blizu cilja: brzina = pomak do cilja (trenutno ponasanje).
    {
        Agents a = makeAgents({{0, 0}}, {{0.5f, 0}});
        const Vec2 v = computeNewVelocity(0, a, nullptr, 0, nullptr, p);
        CHECK_NEAR(v.x, 0.5f, 1e-5f);
        CHECK_NEAR(v.y, 0.0f, 1e-5f);
    }
    // Na cilju: nula, bez NaN.
    {
        Agents a = makeAgents({{3, 3}}, {{3, 3}});
        const Vec2 v = computeNewVelocity(0, a, nullptr, 0, nullptr, p);
        CHECK(v.x == 0.0f && v.y == 0.0f);
    }
    // Cijela simulacija: ravna putanja, stize do cilja.
    {
        Agents cur = makeAgents({{0, 0}}, {{6, 8}}), next = cur;
        const float slope = 8.0f / 6.0f;
        for (int s = 0; s < 400; ++s)
        {
            step(cur, next, p, 1);
            CHECK_NEAR(cur.y[0], slope * cur.x[0], 1e-3f); // ostaje na pravcu
        }
        CHECK(goalDistance(cur, 0) < 0.05f);
    }
}

// ---------------------------------------------------- Test 4: determinizam

static Agents runScenario(int steps)
{
    const Params p = makeParams();
    const std::size_t n = 16;
    Agents cur = makeAntipodal(n, 8.0f), next = cur;
    for (int s = 0; s < steps; ++s) step(cur, next, p, n);
    return cur;
}

static void testDeterminism()
{
    std::printf("Test 4: determinizam\n");
    const Agents a = runScenario(300);
    const Agents b = runScenario(300);
    CHECK(a.x == b.x);   CHECK(a.y == b.y);
    CHECK(a.vx == b.vx); CHECK(a.vy == b.vy);
}

// ------------------------------------------------------------------ main

int main()
{
    testLineProperties();
    runHeadOn(0.0f);   // tocno licem u lice (degenerirani slucaj)
    runHeadOn(0.05f);  // lagani pomak
    testAntipodal();
    testSingleAgent();
    testDeterminism();

    if (g_failures == 0) std::printf("\nSVI TESTOVI PROSLI\n");
    else                 std::printf("\nNEUSPJELO: %d provjera\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}