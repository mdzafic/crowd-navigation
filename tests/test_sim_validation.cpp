#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include "metrics.hpp"
#include "scenario.hpp"
#include "sim.hpp"

using namespace crowd;

static int g_failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

struct Result
{
    float worst_min_dist;   // najmanji razmak u cijeloj simulaciji
    std::size_t reached;    // koliko agenata je na cilju na kraju
    int steps_to_all;       // korak kad su svi stigli (-1 ako nisu)
    float max_speed;
};



static Result run(std::size_t n, std::size_t max_neighbors, int max_steps)
{
    Params p;
    p.max_neighbors = max_neighbors;
    const Scenario sc = make_antipodal_disk(n, p);
    Sim sim(sc.agents, sc.world, p);

    Result r{ 1e30f, 0, -1, 0.f };
    for (int s = 0; s < max_steps; ++s)
    {
        sim.step();
        const Agents& a = sim.state();
        r.worst_min_dist = std::min(r.worst_min_dist, minPairDistance(a));
        r.max_speed = std::max(r.max_speed, maxSpeed(a));
        if (r.steps_to_all < 0 && goalReachedCount(a, p.radius) == n)
            r.steps_to_all = s + 1;
    }
    r.reached = goalReachedCount(sim.state(), p.radius);
    return r;
}

static void testCrowd(std::size_t n, int max_steps)
{
    Params p;
    std::printf("Antipodalna guzva, N=%zu, max_neighbors=%zu\n", n, p.max_neighbors);
    const Result r = run(n, p.max_neighbors, max_steps);
    std::printf("  najmanji razmak: %.3f (2r = %.3f), stiglo: %zu/%zu, svi stigli u koraku: %d\n",
        r.worst_min_dist, 2.f * p.radius, r.reached, n, r.steps_to_all);

    CHECK(r.worst_min_dist >= 0.5f * 2.f * p.radius);   // dopustena penetracija do 50 %
    CHECK(r.max_speed <= p.v_max * 1.0001f);
    CHECK(r.reached * 100 >= n * 98);                   // najmanje 98 % agenata stiglo
}

static void testVelocityNeverExceedsMax()
{
    std::printf("Provjera maksimalne brzine agenata\n");

    Params p;
    Scenario sc = make_antipodal_disk(10, p);
    Sim sim(sc.agents, sc.world, p);

    for (int step = 0; step < 100; ++step)
        sim.step();

    const Agents& a = sim.state();

    for (std::size_t i = 0; i < a.size(); ++i)
    {
        float v = std::sqrt(
            a.vx[i] * a.vx[i] +
            a.vy[i] * a.vy[i]
        );

        CHECK(v <= p.v_max + 1e-4f);
    }
}


static void testTopK(std::size_t n, int max_steps)
{
    std::printf("Utjecaj top-k, N=%zu\n", n);
    for (std::size_t k : {5u, 10u, 20u})
    {
        const Result r = run(n, k, max_steps);
        std::printf("  k=%2zu: min razmak %.3f, stiglo %zu/%zu, svi stigli u koraku %d\n",
            k, r.worst_min_dist, r.reached, n, r.steps_to_all);
    }
}

static void testDeterminism(std::size_t n, int steps)
{
    std::printf("Determinizam Sim, N=%zu\n", n);
    Params p;
    const Scenario sc = make_antipodal_disk(n, p);
    Sim a(sc.agents, sc.world, p), b(sc.agents, sc.world, p);
    for (int s = 0; s < steps; ++s) { a.step(); b.step(); }
    CHECK(a.state().x == b.state().x);
    CHECK(a.state().y == b.state().y);
    CHECK(a.state().vx == b.state().vx);
    CHECK(a.state().vy == b.state().vy);
}

int main()
{
    testCrowd(100, 1500);
    testCrowd(500, 3000);
    testTopK(200, 2000);
    testDeterminism(200, 300);
    testVelocityNeverExceedsMax();
    if (g_failures == 0) std::printf("\nSVI TESTOVI PROSLI\n");
    else                 std::printf("\nNEUSPJELO: %d provjera\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}