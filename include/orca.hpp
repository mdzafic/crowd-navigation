#pragma once

#include "agents.hpp"
#include "params.hpp"

namespace crowd
{
    struct Map;   // forward; map.hpp ukljuèuje orca.hpp

    constexpr int   kMaxWallLines = 32;
    constexpr float kWallRadiusFactor = 0.9f;

    struct Vec2
    {
        float x;
        float y;

        constexpr Vec2 operator+(Vec2 other) const noexcept { return {x + other.x, y + other.y}; }
        constexpr Vec2 operator-(Vec2 other) const noexcept { return {x - other.x, y - other.y}; }
        constexpr Vec2 operator-() const noexcept { return {-x, -y}; }
        constexpr Vec2 operator*(float scalar) const noexcept { return {x * scalar, y * scalar}; }
    };

    constexpr float dot(Vec2 a, Vec2 b) noexcept { return a.x * b.x + a.y * b.y; }
    constexpr float det(Vec2 a, Vec2 b) noexcept { return a.x * b.y - a.y * b.x; }
    constexpr float lengthSq(Vec2 value) noexcept { return dot(value, value); }

    struct Line
    {
        Vec2 point;
        Vec2 direction;
    };

    Line makeOrcaLine(Vec2 positionA, Vec2 velocityA, float radiusA,
                      Vec2 positionB, Vec2 velocityB, float radiusB,
                      float horizon_T, float dt);

    constexpr int lineBufferSize(int neighborCount, int maxWallLines = 0) noexcept
    {
        return (neighborCount + maxWallLines) > 0 ? 2 * (neighborCount + maxWallLines) : 0;
    }

    Vec2 computeNewVelocity(int agentIndex, const Agents& cur, const int* neighbors,
                            int neighborCount, Line* lineBuffer, const Params& p,
                            const Map* map = nullptr);
}