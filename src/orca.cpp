#include "orca.hpp"
#include "map.hpp"
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

        void linearProgram3(const Line* lines, int lineCount, int numObstLines, int beginLine,
                    float radius, Line* projectedLines, Vec2& result) noexcept
        {
            float distance = 0.0f;
            for (int i = beginLine; i < lineCount; ++i)
            {
                const float violation = det(lines[i].direction, lines[i].point - result);
                if (violation <= distance)
                    continue;

                int projectedCount = 0;
                for (int j = 0; j < numObstLines; ++j)          // zidovi ostaju tvrdi
                    projectedLines[projectedCount++] = lines[j];

                for (int j = numObstLines; j < i; ++j)
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
                            (det(lines[j].direction, lines[i].point - lines[j].point) / determinant);
                    }
                    projected.direction = normalized(lines[j].direction - lines[i].direction);
                    projectedLines[projectedCount++] = projected;
                }

                const Vec2 previousResult = result;
                const Vec2 optVelocity{-lines[i].direction.y, lines[i].direction.x};
                const int failedLine = linearProgram2(projectedLines, projectedCount, radius,
                                                      optVelocity, true, result);
                if (failedLine < projectedCount)
                    result = previousResult;
                distance = det(lines[i].direction, lines[i].point - result);
            }
        }
        // Zidne ORCA linije za agenta na poziciji pos. Vraæa broj linija (<= kMaxWallLines).
        int buildWallLines(const Map& map, Vec2 pos, const Params& p, Line* out) noexcept
        {
            const float wr = kWallRadiusFactor * p.radius;
            const float range = p.v_max * p.horizon_T + wr;     // dalje od ovoga zid ne ogranièava
            const float rangeSq = range * range;
            const float cell = map.cell;

            const int cx0 = static_cast<int>(std::floor((pos.x - range) / cell));
            const int cx1 = static_cast<int>(std::floor((pos.x + range) / cell));
            const int cy0 = static_cast<int>(std::floor((pos.y - range) / cell));
            const int cy1 = static_cast<int>(std::floor((pos.y + range) / cell));

            float dist2[kMaxWallLines];
            int count = 0;

            for (int cy = cy0; cy <= cy1; ++cy)
                for (int cx = cx0; cx <= cx1; ++cx)
                {
                    if (!map.isWall(cx, cy)) continue;

                    const float x0 = cx * cell, x1 = x0 + cell;
                    const float y0 = cy * cell, y1 = y0 + cell;
                    const int sx = pos.x < x0 ? -1 : (pos.x > x1 ? 1 : 0);
                    const int sy = pos.y < y0 ? -1 : (pos.y > y1 ? 1 : 0);

                    // Preskoèi æelije koje nisu izložene agentu (sakrivene iza druge æelije zida)
                    // i uglove koji nisu pravi konveksni ugao (lažne "krajnje taèke").
                    if (sx != 0 && sy != 0)
                    {
                        if (map.isWall(cx + sx, cy) || map.isWall(cx, cy + sy) ||
                            map.isWall(cx + sx, cy + sy)) continue;
                    }
                    else if (sx != 0) { if (map.isWall(cx + sx, cy)) continue; }
                    else if (sy != 0) { if (map.isWall(cx, cy + sy)) continue; }

                    const float qx = std::fmin(std::fmax(pos.x, x0), x1);
                    const float qy = std::fmin(std::fmax(pos.y, y0), y1);
                    const float dxv = pos.x - qx, dyv = pos.y - qy;
                    const float d2 = dxv * dxv + dyv * dyv;
                    if (d2 >= rangeSq) continue;

                    float d = std::sqrt(d2);
                    Vec2 n;
                    if (d > epsilon)
                        n = {dxv / d, dyv / d};
                    else   // centar agenta unutar æelije zida (ne bi se smjelo desiti)
                    {
                        n = normalized({pos.x - (x0 + 0.5f * cell), pos.y - (y0 + 0.5f * cell)});
                        d = 0.0f;
                    }

                    // slobodno: v·n >= -(d-r)/T ; kolizija: v·n >= (r-d)/dt
                    const float c = d > wr ? -(d - wr) / p.horizon_T : (wr - d) / p.dt;
                    const Line line{n * c, Vec2{n.y, -n.x}};

                    if (count < kMaxWallLines)
                    {
                        out[count] = line;
                        dist2[count] = d2;
                        ++count;
                    }
                    else   // pun bafer: zamijeni najdalju liniju ako je ova bliža
                    {
                        int worst = 0;
                        for (int k = 1; k < count; ++k)
                            if (dist2[k] > dist2[worst]) worst = k;
                        if (d2 < dist2[worst]) { out[worst] = line; dist2[worst] = d2; }
                    }
                }
            return count;
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
                        int neighborCount, Line* lineBuffer, const Params& p,
                        const Map* map)
    {
        const std::size_t agent = static_cast<std::size_t>(agentIndex);
        const Vec2 position{cur.x[agent], cur.y[agent]};
        const Vec2 goalOffset{cur.gx[agent] - position.x, cur.gy[agent] - position.y};
        Vec2 preferredVelocity = goalOffset;
        const float goalDistanceSq = lengthSq(goalOffset);
        const float maxSpeedSq = p.v_max * p.v_max;
        if (goalDistanceSq > maxSpeedSq)
            preferredVelocity = normalized(goalOffset) * p.v_max;

        // zidne linije idu PRVE u bafer
        const int numWalls = map ? buildWallLines(*map, position, p, lineBuffer) : 0;

        if (neighborCount <= 0 && numWalls == 0)
            return preferredVelocity;

        if (neighborCount > 0)
        {
            constexpr float kSymmetryAngle = 0.02f;
            const float c = std::cos(kSymmetryAngle);
            const float s = std::sin(kSymmetryAngle);
            preferredVelocity = { c * preferredVelocity.x - s * preferredVelocity.y,
                                  s * preferredVelocity.x + c * preferredVelocity.y };
        }

        for (int i = 0; i < neighborCount; ++i)
        {
            const std::size_t neighbor = static_cast<std::size_t>(neighbors[i]);
            lineBuffer[numWalls + i] = makeOrcaLine(
                position, {cur.vx[agent], cur.vy[agent]}, p.radius,
                {cur.x[neighbor], cur.y[neighbor]}, {cur.vx[neighbor], cur.vy[neighbor]},
                p.radius, p.horizon_T, p.dt);
        }

        const int total = numWalls + neighborCount;
        Vec2 result;
        const int failedLine = linearProgram2(lineBuffer, total, p.v_max,
                                              preferredVelocity, false, result);
        if (failedLine < total)
            linearProgram3(lineBuffer, total, numWalls, failedLine, p.v_max,
                           lineBuffer + total, result);
        return result;
    }
}
