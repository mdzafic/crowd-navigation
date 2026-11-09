#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include "map.hpp"
#include "metrics.hpp"
#include "raylib.h"
#include "sim.hpp"
#include "scenario.hpp"
#include "navfield.hpp"

using namespace crowd;

static Color color_for_goal(float x, float y, float gx, float gy)
{
    float ang = std::atan2(gy - y, gx - x);
    float hue = (ang + 3.14159265f) * (180.f / 3.14159265f);
    return ColorFromHSV(hue, 0.75f, 0.95f);
}

static Scenario make_head_on(const Params& p, float y_offset)
{
    Scenario sc = make_antipodal_disk(2, p);
    sc.agents.x[0] = -5.f; sc.agents.y[0] = 0.f;
    sc.agents.gx[0] = 5.f; sc.agents.gy[0] = 0.f;
    sc.agents.x[1] = 5.f;  sc.agents.y[1] = y_offset;
    sc.agents.gx[1] = -5.f; sc.agents.gy[1] = y_offset;
    sc.agents.vx[0] = p.v_max;  sc.agents.vy[0] = 0.f;
    sc.agents.vx[1] = -p.v_max; sc.agents.vy[1] = 0.f;
    return sc;
}

int main(int argc, char** argv)
{
    Params p;

    // --map <fajl>        : samo prikaz mape (korak 1)
    // --sim <fajl> [N]    : simulacija na mapi (korak 2)
    Map map;
    bool map_mode = false, map_sim = false;
    if (argc > 2 && (std::string(argv[1]) == "--map" || std::string(argv[1]) == "--sim"))
    {
        std::string err;
        if (!Map::load(argv[2], map, err))
        {
            std::fprintf(stderr, "Greska pri ucitavanju mape: %s\n", err.c_str());
            return 1;
        }
        map_mode = std::string(argv[1]) == "--map";
        map_sim = !map_mode;
    }
    const bool show_map = map_mode || map_sim;

    const bool head_on = (!show_map && argc > 1 && std::string(argv[1]) == "headon");
    const std::size_t n = map_mode ? 1
        : map_sim ? ((argc > 3) ? std::strtoull(argv[3], nullptr, 10) : 30)
        : head_on ? 2
        : (argc > 1) ? std::strtoull(argv[1], nullptr, 10) : 500;

    // u map modu je Sim samo placeholder (1 agent), ne crta se i ne pokrece
    const Scenario sc = map_sim ? make_map_scenario(map, n, p)
        : head_on ? make_head_on(p, 0.0f)
        : make_antipodal_disk(n, p);
    const Map* mp = map_sim ? &map : nullptr;
    Sim sim(sc.agents, sc.world, p, mp);

    const int W = 1280, H = 800;
    InitWindow(W, H, "crowd_viz");
    SetTargetFPS(60);

    float min_x = sc.world.min_x, max_x = sc.world.max_x;
    float min_y = sc.world.min_y, max_y = sc.world.max_y;
    if (show_map)
    {
        min_x = 0.f; min_y = 0.f;
        max_x = map.widthWorld(); max_y = map.heightWorld();
    }

    Camera2D cam{};
    cam.offset = {W / 2.f, H / 2.f};
    cam.target = {(min_x + max_x) / 2.f, (min_y + max_y) / 2.f};
    const float ext = std::max(max_x - min_x, max_y - min_y);
    cam.zoom = 0.9f * std::min(W, H) / ext;
    const Camera2D cam0 = cam;

    bool paused = false, show_goals = false, show_segments = true, show_field = false;
    int steps_per_frame = 1;

    while (!WindowShouldClose())
    {
        // ---- ulaz ----
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_G)) show_goals = !show_goals;
        if (IsKeyPressed(KEY_L)) show_segments = !show_segments;
        if (IsKeyPressed(KEY_F)) show_field = !show_field;
        if (IsKeyPressed(KEY_UP))   steps_per_frame = std::min(steps_per_frame * 2, 64);
        if (IsKeyPressed(KEY_DOWN)) steps_per_frame = std::max(steps_per_frame / 2, 1);
        if (IsKeyPressed(KEY_HOME)) cam = cam0;
        if (IsKeyPressed(KEY_R))    { sim = Sim(sc.agents, sc.world, p, mp); paused = true; }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 d = GetMouseDelta();
            cam.target.x -= d.x / cam.zoom;
            cam.target.y -= d.y / cam.zoom;
        }
        float wheel = GetMouseWheelMove();
        if (wheel != 0.f)
        {
            Vector2 before = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.zoom = std::clamp(cam.zoom * (1.f + 0.15f * wheel), 0.5f, 200.f);
            Vector2 after = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.target.x += before.x - after.x;
            cam.target.y += before.y - after.y;
        }

        // ---- simulacija; u map modu se ne pokrece ----
        if (!map_mode)
        {
            if (!paused)
                for (int k = 0; k < steps_per_frame; ++k) sim.step();
            else if (IsKeyPressed(KEY_RIGHT))
                sim.step();
        }

        // ---- crtanje ----
        const Agents& a = sim.state();
        BeginDrawing();
        ClearBackground({18, 18, 24, 255});
        BeginMode2D(cam);

        if (show_map)
        {
            for (int cy = 0; cy < map.h; ++cy)
                for (int cx = 0; cx < map.w; ++cx)
                {
                    const std::size_t i = static_cast<std::size_t>(cy) * map.w + cx;
                    Color c = {45, 47, 58, 255};
                    if (map.wall[i])  c = {20, 20, 26, 255};
                    DrawRectangleV({cx * map.cell, cy * map.cell},
                                   {map.cell, map.cell}, c);
                }
            if (show_segments)
                for (const Segment& s : map.segments)
                    DrawLineV({s.a.x, s.a.y}, {s.b.x, s.b.y}, YELLOW);

            // strelice navigacijskog polja za cilj agenta 0
            if (map_sim && show_field && a.size() > 0)
            {
                const NavField& f = sim.nav().at(sim.agent_field(0));
                for (int cy = 0; cy < map.h; ++cy)
                    for (int cx = 0; cx < map.w; ++cx)
                    {
                        const std::size_t i = static_cast<std::size_t>(cy) * map.w + cx;
                        if (f.cost[i] >= 1e29f) continue;
                        const float px = (cx + 0.5f) * map.cell, py = (cy + 0.5f) * map.cell;
                        DrawLineV({px, py}, {px + f.dirx[i] * 0.4f * map.cell,
                                             py + f.diry[i] * 0.4f * map.cell}, SKYBLUE);
                    }
            }
        }
        else
        {
            DrawCircleLinesV({0.f, 0.f}, 0.1f, GRAY);   // ishodiste
        }

        if (!map_mode)
        {
            for (std::size_t i = 0; i < a.size(); ++i)
            {
                Color c = color_for_goal(sc.agents.x[i], sc.agents.y[i],
                                         a.gx[i], a.gy[i]);
                if (show_goals)
                    DrawLineV({a.x[i], a.y[i]}, {a.gx[i], a.gy[i]}, Fade(c, 0.15f));
                DrawCircleV({a.x[i], a.y[i]}, p.radius, c);
            }
        }
        EndMode2D();

        if (map_mode)
        {
            DrawText(TextFormat("mapa %dx%d  segmenata=%zu  FPS=%d",
                                map.w, map.h, map.segments.size(), GetFPS()),
                     10, 10, 20, RAYWHITE);
            DrawText("L konture zidova | HOME kamera | tocak zoom | LMB pan",
                     10, H - 24, 16, GRAY);
        }
        else
        {
            DrawText(TextFormat("N=%zu  preklapanja=%zu  min udaljenost=%.2f  na cilju=%zu  FPS=%d",
                                a.size(), overlapCount(a, p.radius), minPairDistance(a),
                                goalReachedCount(a, 0.25f), GetFPS()),
                     10, 10, 20, RAYWHITE);
            DrawText("SPACE pauza | RIGHT korak | UP/DOWN brzina | G ciljevi | F polje | L zidovi | R reset | HOME kamera | tocak zoom | LMB pan",
                     10, H - 24, 16, GRAY);
        }
        EndDrawing();
    }
    CloseWindow();
    return 0;
}