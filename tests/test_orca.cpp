#include <cmath>
#include <cstdio>
#include "orca.hpp"

using namespace crowd;

static bool near(float a, float b, float tolerance = 1.0e-5f)
{
    return std::fabs(a - b) <= tolerance;
}

int main()
{
    Params p;
    p.v_max = 1.4f;
    p.radius = 0.5f;

    Agents agents;
    agents.resize(2);
    agents.x = { -1.0f, 1.0f };
    agents.y = { 0.0f, 0.0f };
    agents.vx = { 1.4f, -1.4f };
    agents.vy = { 0.0f, 0.0f };
    agents.gx = { 4.0f, -4.0f };
    agents.gy = { 0.0f, 0.0f };

    // Test 1: No-neighbor preferred velocity
    const Vec2 preferred = computeNewVelocity(0, agents, nullptr, 0, nullptr, p);
    if (!near(preferred.x, p.v_max) || !near(preferred.y, 0.0f))
    {
        std::puts("FAIL: no-neighbor preferred velocity");
        return 1;
    }

    // Test 2: Head-on avoidance
    Line lineBuffer[lineBufferSize(1)];
    const int neighbor = 1;
    const Vec2 avoidance = computeNewVelocity(0, agents, &neighbor, 1, lineBuffer, p);

    if (std::fabs(avoidance.y) < 1.0e-4f ||
        lengthSq(avoidance) > p.v_max * p.v_max + 1.0e-5f)
    {
        std::puts("FAIL: head-on avoidance must be lateral and speed-limited");
        return 1;
    }

    // Test 3: Deterministic ORCA result
    agents.x[1] = -0.5f;
    agents.y[1] = 0.4f;
    agents.vx[1] = 0.0f;
    agents.vy[1] = 0.0f;
    agents.gx[0] = 5.0f;
    agents.gy[0] = 2.0f;

    const Vec2 first = computeNewVelocity(0, agents, &neighbor, 1, lineBuffer, p);
    const Vec2 second = computeNewVelocity(0, agents, &neighbor, 1, lineBuffer, p);

    if (!near(first.x, second.x) || !near(first.y, second.y))
    {
        std::puts("FAIL: ORCA result is not deterministic");
        return 1;
    }

    if (lengthSq(first) > p.v_max * p.v_max + 1.0e-5f)
    {
        std::puts("FAIL: result exceeds maximum speed");
        return 1;
    }

    // Test 4: Agents start overlapping
    Vec2 posA{ 0.0f, 0.0f };
    Vec2 posB{ 0.0f, 0.0f };
    Vec2 velA{ 0.0f, 0.0f };
    Vec2 velB{ 0.0f, 0.0f };

    float rA = p.radius;
    float rB = p.radius;
    float T = p.horizon_T;
    float dt = p.dt;

    Line orcaLine = makeOrcaLine(posA, velA, rA, posB, velB, rB, T, dt);

    // Direction should have unit length
    float directionLengthSq =
        orcaLine.direction.x * orcaLine.direction.x +
        orcaLine.direction.y * orcaLine.direction.y;

    if (!near(directionLengthSq, 1.0f, 1.0e-5f))
    {
        std::puts("FAIL: overlapping agents produce invalid ORCA direction");
        return 1;
    }

    // Separating velocity should not be zero
    float separatingVelocity =
        std::fabs(orcaLine.point.x - velA.x) +
        std::fabs(orcaLine.point.y - velA.y);

    if (separatingVelocity <= 0.0f)
    {
        std::puts("FAIL: overlapping agents produce zero separating velocity");
        return 1;
    }

    // Test 5: Static wall constraint
    agents.resize(1);
    agents.x[0] = 0.0f;
    agents.y[0] = 0.0f;
    agents.vx[0] = 0.0f;
    agents.vy[0] = 0.0f;
    agents.gx[0] = 10.0f;
    agents.gy[0] = 0.0f;
    const Line wall = makeWallOrcaLine(
        {0.0f, 0.0f}, {0.0f, 0.0f}, p.radius,
        {1.0f, -1.0f}, {1.0f, 1.0f}, p.horizon_T, p.dt);
    Line wallBuffer[lineBufferSize(1)];
    const Vec2 wallAvoidance = computeNewVelocity(
        0, agents, nullptr, 0, &wall, 1, wallBuffer, p);
    if (wallAvoidance.x >= p.v_max - 1.0e-4f ||
        det(wall.direction, wall.point - wallAvoidance) > 1.0e-4f)
    {
        std::puts("FAIL: wall ORCA constraint does not block motion into wall");
        return 1;
    }

    std::puts("OK orca_velocity");
    return 0;
}
