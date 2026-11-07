#include "orca.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace crowd
{
    namespace
    {
        constexpr float epsilon = 1.0e-6f;

        Vec2 normalized(Vec2 value) noexcept
        {
            const float length2 = lengthSq(value);
            if (length2 <= epsilon * epsilon)
                return {1.0f, 0.0f};
            const float inverseLength = 1.0f / std::sqrt(length2);
            return value * inverseLength;
        }

        bool linearProgram1(const Line* lines, int lineNo, float radius,
                            Vec2 optVelocity, bool directionOpt, Vec2& result) noexcept
        {
            const Line& line = lines[lineNo];
            const float dotProduct = dot(line.point, line.direction);
            const float discriminant = dotProduct * dotProduct + radius * radius -
                                       lengthSq(line.point);
            if (discriminant < 0.0f)
                return false;

            const float sqrtDiscriminant = std::sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                const float denominator = det(line.direction, lines[i].direction);
                const float numerator = det(lines[i].direction, line.point - lines[i].point);
                if (std::fabs(denominator) <= epsilon)
                {
                    if (numerator < 0.0f)
                        return false;
                    continue;
                }

                const float t = numerator / denominator;
                if (denominator >= 0.0f)
                    tRight = std::min(tRight, t);
                else
                    tLeft = std::max(tLeft, t);

                if (tLeft > tRight)
                    return false;
            }

            if (directionOpt)
                result = line.point + line.direction *
                         (dot(optVelocity, line.direction) > 0.0f ? tRight : tLeft);
            else
            {
                const float t = dot(line.direction, optVelocity - line.point);
                result = line.point + line.direction * std::clamp(t, tLeft, tRight);
            }
            return true;
        }

        int linearProgram2(const Line* lines, int lineCount, float radius,
                           Vec2 optVelocity, bool directionOpt, Vec2& result) noexcept
        {
            if (directionOpt)
                result = optVelocity * radius;
            else if (lengthSq(optVelocity) > radius * radius)
                result = normalized(optVelocity) * radius;
            else
                result = optVelocity;

            for (int i = 0; i < lineCount; ++i)
            {
                if (det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    const Vec2 previousResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result))
                    {
                        result = previousResult;
                        return i;
                    }
                }
            }
            return lineCount;
        }

        void linearProgram3(const Line* lines, int lineCount, int beginLine, float radius,
                            Line* projectedLines, Vec2& result) noexcept
        {
            float distance = 0.0f;
            for (int i = beginLine; i < lineCount; ++i)
            {
                const float violation = det(lines[i].direction, lines[i].point - result);
                if (violation <= distance)
                    continue;

                int projectedCount = 0;
                for (int j = 0; j < i; ++j)
                {
                    Line projected{};
                    const float determinant = det(lines[i].direction, lines[j].direction);
                    if (std::fabs(determinant) <= epsilon)
                    {
                        if (dot(lines[i].direction, lines[j].direction) > 0.0f)
                            continue;
                        projected.point = (lines[i].point + lines[j].point) * 0.5f;
                    }
                    else
                    {
                        projected.point = lines[i].point +
                            lines[i].direction *
                            (det(lines[j].direction, lines[i].point - lines[j].point) /
                             determinant);
                    }
                    projected.direction = normalized(lines[j].direction - lines[i].direction);
                    projectedLines[projectedCount++] = projected;
                }

                const Vec2 previousResult = result;
                const Vec2 optVelocity{-lines[i].direction.y, lines[i].direction.x};
                const int failedLine = linearProgram2(projectedLines, projectedCount, radius,
                                                      optVelocity, true, result);
                if (static_cast<float>(failedLine) < distance)
                    result = previousResult;
                else
                    distance = det(lines[i].direction, lines[i].point - result);
            }
        }
    }

    Line makeOrcaLine(Vec2 positionA, Vec2 velocityA, float radiusA,
                      Vec2 positionB, Vec2 velocityB, float radiusB,
                      float horizon_T, float dt)
    {
        if (!(horizon_T > 0.0f) || !(dt > 0.0f))
            throw std::invalid_argument("ORCA horizon_T and dt must be positive");

        const Vec2 relativePosition = positionB - positionA;
        const Vec2 relativeVelocity = velocityA - velocityB;
        const float distanceSq = lengthSq(relativePosition);
        const float combinedRadius = radiusA + radiusB;
        const float combinedRadiusSq = combinedRadius * combinedRadius;

        Vec2 u;
        Vec2 normal;
        if (distanceSq > combinedRadiusSq)
        {
            const Vec2 w = relativeVelocity - relativePosition * (1.0f / horizon_T);
            const float wLengthSq = lengthSq(w);
            const float dotProduct = dot(w, relativePosition);
            if (dotProduct < 0.0f && dotProduct * dotProduct > combinedRadiusSq * wLengthSq)
            {
                const float wLength = std::sqrt(wLengthSq);
                normal = wLength > epsilon ? w * (1.0f / wLength) : normalized(-relativePosition);
                const Vec2 direction{normal.y, -normal.x};
                u = normal * (combinedRadius / horizon_T - wLength);
                return {velocityA + u * 0.5f, direction};
            }

            const float leg = std::sqrt(std::max(0.0f, distanceSq - combinedRadiusSq));
            Vec2 direction;
            if (det(relativePosition, w) > 0.0f)
                direction = {relativePosition.x * leg - relativePosition.y * combinedRadius,
                             relativePosition.x * combinedRadius + relativePosition.y * leg};
            else
                direction = {-relativePosition.x * leg - relativePosition.y * combinedRadius,
                             relativePosition.x * combinedRadius - relativePosition.y * leg};
            direction = direction * (1.0f / distanceSq);
            u = direction * dot(relativeVelocity, direction) - relativeVelocity;
            return {velocityA + u * 0.5f, direction};
        }

        const Vec2 w = relativeVelocity - relativePosition * (1.0f / dt);
        const float wLength = std::sqrt(lengthSq(w));
        normal = wLength > epsilon ? w * (1.0f / wLength) : normalized(-relativePosition);
        u = normal * (combinedRadius / dt - wLength);
        return {velocityA + u * 0.5f, {normal.y, -normal.x}};
    }

    Vec2 computeNewVelocity(int agentIndex, const Agents& cur, const int* neighbors,
                            int neighborCount, Line* lineBuffer, const Params& p)
    {
        const std::size_t agent = static_cast<std::size_t>(agentIndex);
        const Vec2 position{cur.x[agent], cur.y[agent]};
        const Vec2 goalOffset{cur.gx[agent] - position.x, cur.gy[agent] - position.y};
        Vec2 preferredVelocity = goalOffset;
        const float goalDistanceSq = lengthSq(goalOffset);
        const float maxSpeedSq = p.v_max * p.v_max;
        if (goalDistanceSq > maxSpeedSq)
            preferredVelocity = normalized(goalOffset) * p.v_max;

        if (neighborCount <= 0)
            return preferredVelocity;

        for (int i = 0; i < neighborCount; ++i)
        {
            const std::size_t neighbor = static_cast<std::size_t>(neighbors[i]);
            lineBuffer[i] = makeOrcaLine(
                position, {cur.vx[agent], cur.vy[agent]}, p.radius,
                {cur.x[neighbor], cur.y[neighbor]}, {cur.vx[neighbor], cur.vy[neighbor]},
                p.radius, p.horizon_T, p.dt);
        }

        Vec2 result;
        const int failedLine = linearProgram2(lineBuffer, neighborCount, p.v_max,
                                              preferredVelocity, false, result);
        if (failedLine < neighborCount)
            linearProgram3(lineBuffer, neighborCount, failedLine, p.v_max,
                           lineBuffer + neighborCount, result);
        return result;
    }
}
